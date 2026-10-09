#include "dvrmc/types.hpp"
#include <climits>
namespace dvrmc {
int32_t signed_word(uint32_t v) {
    return v <= uint32_t(INT32_MAX) ? int32_t(v) : int32_t(int64_t(v)-4294967296LL);
}
static int32_t clamp(int64_t x, int32_t lo, int32_t hi) {
    return x < lo ? lo : (x > hi ? hi : int32_t(x));
}
bool valid(const MotorConfig& c) {
    using namespace field;
    if (c[ENABLE]<0 || c[ENABLE]>1 || c[ENCODER_PPR]<1 || c[ENCODER_PPR]>10000 ||
        c[GEAR_NUM]<1 || c[GEAR_DEN]<1 || c[ENCODER_SCALE_PPM]<100000 ||
        c[ENCODER_SCALE_PPM]>10000000 || c[COUNTS_PER_REV_Q16]<one ||
        c[WHEEL_RADIUS_UM]<0 || c[NOMINAL_VOLTAGE_MV]<0 || c[CURRENT_LIMIT_MA]<0 ||
        c[MAX_SPEED_MRPM]<1 || c[MAX_SPEED_MRPM]>1000000 ||
        c[TARGET_MRPM] < -c[MAX_SPEED_MRPM] || c[TARGET_MRPM]>c[MAX_SPEED_MRPM] ||
        c[MAX_ACCEL_MRPM_S]<1 || c[MAX_ACCEL_MRPM_S]>10000000 ||
        (c[ENCODER_SIGN]!=1 && c[ENCODER_SIGN]!=-1) ||
        (c[DRIVE_SIGN]!=1 && c[DRIVE_SIGN]!=-1) ||
        c[PWM_HZ]<1 || c[SAMPLE_HZ]<1 || c[FILTER_SAMPLES]<1 || c[FILTER_SAMPLES]>32 ||
        c[WATCHDOG_US]<100 || c[WATCHDOG_US]>10000000 ||
        c[MAX_DELTA_COUNTS]<1 || c[MAX_DELTA_COUNTS]>65535) return false;
    for (unsigned i=KP_Q16; i<=KFF_Q16; ++i)
        if (c.words[i]<0 || c.words[i]>16*one) return false;
    for (unsigned i=D_ALPHA_Q16; i<=DUTY_LIMIT_Q16; ++i)
        if (c.words[i]<0 || c.words[i]>one) return false;
    if (c[SPEED_ALPHA_Q16]<1 || c[SPEED_ALPHA_Q16]>one) return false;
    for (unsigned i=PWM_GAIN_FWD_Q16; i<=PWM_GAIN_REV_Q16; ++i)
        if (c.words[i]<=0 || c.words[i]>2*one) return false;
    for (unsigned i=PWM_OFFSET_FWD_Q16; i<=PWM_OFFSET_REV_Q16; ++i)
        if (c.words[i]<0 || c.words[i]>one) return false;
    return true;
}
static int64_t calibrate(int64_t u, const MotorConfig& c) {
    using namespace field;
    if (u==0) return 0;
    return u>0 ? u*c[PWM_GAIN_FWD_Q16]/one+c[PWM_OFFSET_FWD_Q16]
               : u*c[PWM_GAIN_REV_Q16]/one-c[PWM_OFFSET_REV_Q16];
}
void wheel_step(const MotorConfig& c, const EncoderSnapshot& e,
                uint32_t dt, bool timed_out, WheelState& s) {
    using namespace field;
    if (!valid(c) || dt<100 || dt>100000) {
        s.faults |= BAD_CONFIG; s.duty_q16=0; return;
    }
    int64_t delta=0;
    if (s.primed) delta=int64_t(signed_word(e.position-s.previous_position))*c[ENCODER_SIGN];
    else s.primed=true;
    s.previous_position=e.position;
    s.encoder_errors=e.errors;
    if (delta>c[MAX_DELTA_COUNTS] || delta<-int64_t(c[MAX_DELTA_COUNTS])) {
        s.faults |= BAD_FEEDBACK; delta=0;
    }
    // Saturating accumulated position; unsigned hardware count can wrap.
    if (delta>0 && s.position>INT64_MAX-delta) s.position=INT64_MAX;
    else if (delta<0 && s.position<INT64_MIN-delta) s.position=INT64_MIN;
    else s.position+=delta;
    const int64_t counts_per_minute=delta*60000000LL/dt;
    const int32_t measured=clamp(counts_per_minute*1000LL*one/c[COUNTS_PER_REV_Q16],
                                 -1000000,1000000);
    s.speed_mrpm += int32_t((int64_t(measured)-s.speed_mrpm)*c[SPEED_ALPHA_Q16]/one);
    if (timed_out && c[ENABLE]) s.faults |= WATCHDOG;
    if (!c[ENABLE] || s.faults) {
        s.duty_q16=0; s.integral=0; s.derivative=0; s.target_mrpm=0;
        s.previous_speed=s.speed_mrpm; s.ramp_fraction=0; return;
    }
    s.ramp_fraction += uint64_t(c[MAX_ACCEL_MRPM_S])*dt;
    const int32_t step=int32_t(s.ramp_fraction/1000000);
    s.ramp_fraction %= 1000000;
    const int32_t target_change=clamp(int64_t(c[TARGET_MRPM])-s.target_mrpm,-step,step);
    s.target_mrpm+=target_change;
    const int32_t error=s.target_mrpm-s.speed_mrpm;
    // Derivative on measured speed avoids a command-step derivative kick.
    const int64_t d_raw=-int64_t(c[KD_Q16])*(s.speed_mrpm-s.previous_speed)*1000000LL/(1000LL*dt);
    const int32_t d_limited=clamp(d_raw,-16*one,16*one);
    s.derivative += int32_t((int64_t(d_limited)-s.derivative)*c[D_ALPHA_Q16]/one);
    s.previous_speed=s.speed_mrpm;
    const int32_t i_candidate=clamp(int64_t(s.integral)+int64_t(c[KI_Q16])*error*dt/1000000000LL,
                                    -c[I_LIMIT_Q16],c[I_LIMIT_Q16]);
    const int64_t base=int64_t(c[KP_Q16])*error/1000+
                       int64_t(c[KFF_Q16])*s.target_mrpm/1000+s.derivative;
    const int64_t candidate=calibrate(base+i_candidate,c);
    // Conditional integration accounts for calibration-induced saturation.
    if ((candidate<=c[DUTY_LIMIT_Q16] && candidate>=-c[DUTY_LIMIT_Q16]) ||
        (candidate>c[DUTY_LIMIT_Q16] && error<0) ||
        (candidate<-c[DUTY_LIMIT_Q16] && error>0)) s.integral=i_candidate;
    s.duty_q16=clamp(calibrate(base+s.integral,c),-c[DUTY_LIMIT_Q16],c[DUTY_LIMIT_Q16])*c[DRIVE_SIGN];
}
}
