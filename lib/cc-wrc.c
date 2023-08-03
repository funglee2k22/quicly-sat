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

static inline u32 get_rtt_high_watermarker(u32 rtt_ms) {
	const u32 eleven_rtt = rtt_ms * rtt_high_factor;

	if (eleven_rtt < rtt_max_floor) {
		return rtt_max_floor;
	}

	if (eleven_rtt >= rtt_max_configurable) {
		return rtt_max_configurable;
	}

	return eleven_rtt;

}

static inline u32 get_rtt_low_watermarker(u32 rtt_ms) {
	const u32 adjust_rtt = rtt_ms;

        if (adjust_rtt < rtt_min_floor) {
		return rtt_min_floor;
	}

	if (adjust_rtt >= rtt_min) {
		return rtt_min;
	}

	return adjust_rtt;
}

static inline int get_maxcwnd_rtt_slope(const int rtt_highvaluemarker, const int rtt_lowvaluemarker)
{
	const int ydiff = min_clamp - max_clamp;
	const int xdiff = rtt_highvaluemarker - rtt_lowvaluemarker;

        if ((max_clamp <= min_clamp)
	    || (rtt_highvaluemarker <= rtt_lowvaluemarker))
		return 0;

	return (ydiff / xdiff);
}


static inline void rbc_calculate_boundary(quicly_cc_t *cc, int32_t rtt_ms){
     cc->state.wrc.rtt_high_water_marker = get_rtt_high_watermarker(rtt_ms);
	 cc->state.wrc.rtt_low_water_marker = get_rtt_low_watermarker(rtt_ms);
	 cc->state.wrc.cwndslope = get_maxcwnd_rtt_slope(cc->state.wrc.rtt_high_water_marker, cc->state.wrc.rtt_low_water_marker);
	return;
}

static inline u32 get_cwnd_lbound_sgmnts(u32 mss)
{
	return (min_clamp / mss);
}

static u32 get_cwnd_inbound_sgmnts(quicly_cc_t *cc, u32 crntrtt, u32 mss)
{
	const u32 rtt_highvaluemarker =  cc->state.wrc.rtt_high_water_marker;
	const u32 rtt_lowvaluemarker = cc->state.wrc.rtt_low_water_marker;
	const s32 maxcwnd_rttslope = cc->state.wrc.cwndslope;

	if (crntrtt >= rtt_highvaluemarker)
		return get_cwnd_lbound_sgmnts(mss);
	else if (crntrtt <= rtt_lowvaluemarker)
		return get_cwnd_ubound_sgmnts(mss);
	else {
		u32 crntcwnd_bytes =
			cwnd_highbound +
			((crntrtt - rtt_lowvaluemarker) * maxcwnd_rttslope);

		return (crntcwnd_bytes / mss);
	}
}

static inline u32 adjust_cwnd_inbound_sgmnts(quicly_cc_t *cc) {
	u32 snd_cwnd_clamp = cc->state.wrc.snd_cwnd_clamp_max; /* current snd_cwnd_clamp */
	const u32 mss_cache = tp->mss_cache; //Figure this out
	u32 snd_cwnd_clamp_bytes = snd_cwnd_clamp * mss_cache;

	/* advertised window size from remote peer in bytes */
	const u32 snd_wnd = cc->cwnd;
	/* estimate the remote's peer's rwin = adv_win + packets_in_flight */
	const u32 estimated_rwin = snd_wnd + tp->packets_out * tp->mss_cache;
	const u32 reduced_rwnd = (estimated_rwin * cwnd_safe_factor) / 100;

	if (reduced_rwnd < snd_cwnd_clamp_bytes) {
		snd_cwnd_clamp_bytes = max(reduced_rwnd, cc->cwnd_initial * mss_cache);
		snd_cwnd_clamp = (snd_cwnd_clamp_bytes / mss_cache);
	}

	return snd_cwnd_clamp;
}

static void wrc_on_acked(quicly_cc_t *cc, const quicly_loss_t *loss, uint32_t bytes_acked, 
                         uint64_t largest_acked, uint32_t inflight, uint64_t next_pn, 
                         int64_t now, uint32_t max_udp_payload_size) 
{
    /* Variables that replace the ones from the tcp function */
    uint32_t srtt_crnt = 0; //current smooth RTT
    int32_t rtt_ms = loss->rtt.latest / 1000; // Assuming latest is in microseconds
    uint32_t srtt_prev = loss->rtt.smoothed / 1000; //previous smooth RTT

    /* If the rtt is not valid, go to debug info */
    if (now <= 0) {
        goto debug_info;
    }

    cc->state.wrc.rtt_cnt = cc->state.wrc.rtt_cnt + 1;

    /* Replacing the min_rtt_check_threshold with a constant value of 10 */
    if (cc->state.wrc.rtt_cnt <= 10) {
        int flag = 0;
        if (cc->state.wrc.init_rtt == 0) {
            cc->state.wrc.init_rtt = loss->rtt.smoothed = cc->state.wrc.min_rtt = rtt_ms;
            srtt_prev = rtt_ms;
            flag = 1;
        } else if (rtt_ms < cc->state.wrc.min_rtt) {
            cc->state.wrc.min_rtt = rtt_ms;
            flag = 1;
        }

        /* Ignoring the part where 
    rbc_calculate_boundary() is called as it is not provided */
        if (flag) {
			// init RTT or smaller min_rtt detected.
            rbc_calculate_boundary(cc, rtt_ms); //TODO Change for the implementation of QUIC
		}

        // if (flag) {
        //     printf("Minimum RTT detected: %u\n", cc->state.wrc.rtt_low_water_marker);
        // }
    }

    srtt_crnt = rbc_lpf_srtt(rtt_ms, srtt_prev); //Don't need this // Assuming lpf_srtt() function exists and operates similarly to tcp_rbc_lpf_srtt()

    if (srtt_crnt > 0)
        loss->rtt.smooth = srtt_crnt;
    else
        loss->rtt.smooth = rtt_ms;

    cc->state.wrc.snd_cwnd_clamp_max =
        get_cwnd_inbound_sgmnts(cc, rtt_ms, mss); // Assuming get_cwnd_inbound_sgmnts() function exists and operates similarly

    cc->state.wrc.snd_cwnd_clamp_max = adjust_cwnd_inbound_sgmnts(); // Assuming adjust_cwnd_inbound_sgmnts() function exists and operates similarly

    debug_info:
        printf("Debug Info\n");

    //TODO: Quick start/slow start/Congestion Avoidance. 

}


static void cubic_on_lost(quicly_cc_t *cc, const quicly_loss_t *loss, uint32_t bytes, uint64_t lost_pn, uint64_t next_pn,
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



static void cubic_on_persistent_congestion(quicly_cc_t *cc, const quicly_loss_t *loss, int64_t now)
{
    /* TODO */
}

static void cubic_on_sent(quicly_cc_t *cc, const quicly_loss_t *loss, uint32_t bytes, int64_t now)
{
    /* Prevent extreme cwnd growth following an idle period caused by application limit.
     * This fixes the W_cubic/W_est calculations by effectively subtracting the idle period
     * The sender is coming out of quiescence if the current packet is the only one in flight.
     * (see https://github.com/torvalds/linux/commit/30927520dbae297182990bb21d08762bcc35ce1d). */
    if (loss->sentmap.bytes_in_flight <= bytes && cc->state.cubic.avoidance_start != 0 && cc->state.cubic.last_sent_time != 0) {
        int64_t delta = now - cc->state.cubic.last_sent_time;
        if (delta > 0)
            cc->state.cubic.avoidance_start += delta;
    }

    cc->state.cubic.last_sent_time = now;
}

static void wrc_reset(quicly_cc_t *cc, uint32_t initcwnd)
{
    memset(cc, 0, sizeof(quicly_cc_t));
    cc->type = &quicly_cc_type_wrc;
    cc->cwnd = cc->cwnd_initial = cc->cwnd_maximum = initcwnd;
    cc->ssthresh = cc->cwnd_minimum = UINT32_MAX;

    cc->state.wrc.snd_cwnd_clamp_max = max_clamp;
    cc->state.wrc.snd_cwnd_clamp_min = min_clamp;
    cc->state.wrc.rtt_high_water_marker = rtt_max_configurable;
    cc->state.wrc.rtt_low_water_marker = rtt_min; 

    cc->state.wrc.init_rtt = 0;
    cc->state.wrc.min_rtt = rtt_max_configurable;
    cc->state.wrc.rtt = 0;
    cc->state.wrc.enter_ca_tm = 0;
    cc->state.wrc.prev_cwnd = cc->cwnd;
    // cc->state.wrc.rwin_booster = 1; 

}

static int cubic_on_switch(quicly_cc_t *cc)
{
    if (cc->type == &quicly_cc_type_wrc)
        return 1;

    if (cc->type == &quicly_cc_type_reno || cc->type == &quicly_cc_type_pico) {
        /* When in slow start, state can be reused as-is; otherwise, restart. */
        if (cc->cwnd_exiting_slow_start == 0) {
            cc->type = &quicly_cc_type_wrc;
        } else {
            cubic_reset(cc, cc->cwnd_initial);
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
