/*
 */
#include <math.h>
#include "quicly/cc.h"
#include "quicly.h"

#define QUICLY_MIN_CWND 2 

typedef double cubic_float_t;
#define QUICLY_CUBIC_C ((cubic_float_t)0.4)
#define QUICLY_CUBIC_BETA ((cubic_float_t)0.7)


/* Calculates the time elapsed since the last congestion event (parameter t) */
static cubic_float_t calc_cubic_t(const quicly_cc_t *cc, int64_t now)
{
    cubic_float_t clock_delta = now - cc->state.cubic.avoidance_start;
    return clock_delta / 1000; /* ms -> s */
}

/* RFC 8312, Equation 1; using bytes as unit instead of MSS */
static uint32_t calc_w_cubic(const quicly_cc_t *cc, cubic_float_t t_sec, uint32_t max_udp_payload_size)
{
    cubic_float_t tk = t_sec - cc->state.cubic.k;
    return (QUICLY_CUBIC_C * (tk * tk * tk) * max_udp_payload_size) + cc->state.cubic.w_max;
}

/* RFC 8312, Equation 2 */
/* K depends solely on W_max, so we update both together on congestion events */
static void update_cubic_k(quicly_cc_t *cc, uint32_t max_udp_payload_size)
{
    cubic_float_t w_max_mss = cc->state.cubic.w_max / (cubic_float_t)max_udp_payload_size;
    cc->state.cubic.k = cbrt(w_max_mss * ((1 - QUICLY_CUBIC_BETA) / QUICLY_CUBIC_C));
}

/* RFC 8312, Equation 4; using bytes as unit instead of MSS */
static uint32_t calc_w_est(const quicly_cc_t *cc, cubic_float_t t_sec, cubic_float_t rtt_sec, uint32_t max_udp_payload_size) 
{ //TODO: Make this into a linear function to make use of RTT
    return (cc->state.cubic.w_max * QUICLY_CUBIC_BETA) +
           ((3 * (1 - QUICLY_CUBIC_BETA) / (1 + QUICLY_CUBIC_BETA)) * (t_sec / rtt_sec) * max_udp_payload_size);
}

/* TODO: Avoid increase if sender was application limited. */
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

static uint32_t min_clamp = 1 * 1024 * 1024;
static uint32_t max_clamp = 64 * 1024 * 1024;

static inline uint32_t get_rtt_high_watermarker(int32_t rtt_ms)
{
	return min(2500, rtt_ms * 4);
}

static inline uint32_t get_rtt_low_watermarker(int32_t rtt_ms)
{
	return max(550, rtt_ms); 
}

static inline float get_maxcwnd_rtt_slope(const int rtt_highvaluemarker, const int rtt_lowvaluemarker)
{
	const int ydiff = min_clamp - max_clamp;
	const int xdiff = rtt_highvaluemarker - rtt_lowvaluemarker;

        if ((max_clamp <= min_clamp)
	    || (rtt_highvaluemarker <= rtt_lowvaluemarker))
		return 0;

	return (ydiff * 1.0 / xdiff);
}

static inline uint32_t get_cwnd_lbound_sgmnts(uint32_t mss)
{
	return (min_clamp / mss);
}


static inline uint32_t get_cwnd_ubound_sgmnts(uint32_t mss)
{
	return (max_clamp / mss);
}

static uint32_t get_cwnd_inbound_sgmnts(quicly_cc_t *cc, uint32_t crntrtt, uint32_t mss)
{
	const uint32_t rtt_high = get_rtt_high_watermarker(crntrtt); 
	const uint32_t rtt_low = get_rtt_low_watermarker(crntrtt); 
	const int32_t maxcwnd_rttslope = get_maxcwnd_rtt_slope(rtt_high, rtt_low);

	if (crntrtt >= rtt_high)
		return get_cwnd_lbound_sgmnts(mss);
	else if (crntrtt <= rtt_low)
		return get_cwnd_ubound_sgmnts(mss);
	else {
		uint32_t cwnd_highbound = 14 * 1024 * 1024; 
		uint32_t rtt_lowvaluemarker = 550; 
		uint32_t crntcwnd_bytes = cwnd_highbound + 
				((crntrtt - rtt_lowvaluemarker) * maxcwnd_rttslope);
		return (crntcwnd_bytes / mss);
	}

}

static void wrc_on_acked(quicly_cc_t *cc, const quicly_loss_t *loss, uint32_t bytes_acked, 
                         uint64_t largest_acked, uint32_t inflight, uint64_t next_pn, 
                         int64_t now, uint32_t max_udp_payload_size) 
{
    uint32_t rtt_ms = loss->rtt.latest;
    //uint32_t srtt_ms = loss->rtt.smoothed;

    cc->state.wrc.rtt_cnt += 1;

    cc->state.wrc.snd_cwnd_clamp_max = get_cwnd_inbound_sgmnts(cc, rtt_ms, max_udp_payload_size); 

}


static void wrc_on_lost(quicly_cc_t *cc, const quicly_loss_t *loss, uint32_t bytes, uint64_t lost_pn, uint64_t next_pn,
                          int64_t now, uint32_t max_udp_payload_size)
{
    /* Nothing to do if loss is in recovery window. */
    if (lost_pn < cc->recovery_end)
        return;
    cc->recovery_end = next_pn;

    ++cc->num_loss_episodes;
    if (cc->cwnd_exiting_slow_start == 0)
        cc->cwnd_exiting_slow_start = cc->cwnd;

    cc->state.cubic.avoidance_start = now;
    cc->state.cubic.w_max = cc->cwnd;

    /* RFC 8312, Section 4.6; Fast Convergence */
    /* w_last_max is initialized to zero; therefore this condition is false when exiting slow start */
    if (cc->state.cubic.w_max < cc->state.cubic.w_last_max) {
        cc->state.cubic.w_last_max = cc->state.cubic.w_max;
        cc->state.cubic.w_max *= (1.0 + QUICLY_CUBIC_BETA) / 2.0;
    } else {
        cc->state.cubic.w_last_max = cc->state.cubic.w_max;
    }

    update_cubic_k(cc, max_udp_payload_size);

    /* RFC 8312, Section 4.5; Multiplicative Decrease */
    cc->cwnd *= QUICLY_CUBIC_BETA;
    if (cc->cwnd < QUICLY_MIN_CWND * max_udp_payload_size)
        cc->cwnd = QUICLY_MIN_CWND * max_udp_payload_size;
    cc->ssthresh = cc->cwnd;

    if (cc->cwnd_minimum > cc->cwnd)
        cc->cwnd_minimum = cc->cwnd;
}



static void wrc_on_persistent_congestion(quicly_cc_t *cc, const quicly_loss_t *loss, int64_t now)
{
    /* TODO */
}

static void wrc_on_sent(quicly_cc_t *cc, const quicly_loss_t *loss, uint32_t bytes, int64_t now)
{
    /* Prevent extreme cwnd growth following an idle period caused by application limit.
     * This fixes the W_cubic/W_est calculations by effectively subtracting the idle period
     * The sender is coming out of quiescence if the current packet is the only one in flight.
     * (see https://github.com/torvalds/linux/commit/30927520dbae297182990bb21d08762bcc35ce1d). */
    if (loss->sentmap.bytes_in_flight <= bytes && cc->state.wrc.avoidance_start != 0 && cc->state.wrc.last_sent_time != 0) {
        int64_t delta = now - cc->state.wrc.last_sent_time;
        if (delta > 0)
            cc->state.wrc.avoidance_start += delta;
    }

    cc->state.wrc.last_sent_time = now;
}

static void wrc_reset(quicly_cc_t *cc, uint32_t initcwnd)
{
    const uint32_t rtt_min = 550; 
    const uint32_t rtt_max = 2500; 

    memset(cc, 0, sizeof(quicly_cc_t));
    cc->type = &quicly_cc_type_wrc;
    cc->cwnd = cc->cwnd_initial = cc->cwnd_maximum = initcwnd;
    cc->ssthresh = cc->cwnd_minimum = UINT32_MAX;

    cc->state.wrc.snd_cwnd_clamp_max = max_clamp;
    cc->state.wrc.snd_cwnd_clamp_min = min_clamp;
    cc->state.wrc.rtt_high_water_marker = rtt_max; 
    cc->state.wrc.rtt_low_water_marker = rtt_min; 

    cc->state.wrc.rtt_init = 0;
    cc->state.wrc.rtt_minimum = UINT32_MAX;
    cc->state.wrc.rtt_cnt = 0;

}

static int wrc_on_switch(quicly_cc_t *cc)
{
    if (cc->type == &quicly_cc_type_wrc)
        return 1;

    if (cc->type == &quicly_cc_type_reno || cc->type == &quicly_cc_type_pico) {
        /* When in slow start, state can be reused as-is; otherwise, restart. */
        if (cc->cwnd_exiting_slow_start == 0) {
            cc->type = &quicly_cc_type_wrc;
        } else {
            wrc_reset(cc, cc->cwnd_initial);
        }
        return 1;
    }

    return 0;
}

static void wrc_init(quicly_init_cc_t *self, quicly_cc_t *cc, uint32_t initcwnd, int64_t now)
{
    wrc_reset(cc, initcwnd);
}

quicly_cc_type_t quicly_cc_type_wrc = {
    "wrc", 
    &quicly_cc_wrc_init, 
    wrc_on_acked, 
    wrc_on_lost, 
    wrc_on_persistent_congestion, 
    wrc_on_sent, 
    wrc_on_switch
};

quicly_init_cc_t quicly_cc_wrc_init = {wrc_init};
