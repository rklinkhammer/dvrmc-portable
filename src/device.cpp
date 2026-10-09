#include "dvrmc/device.hpp"
namespace dvrmc {
void Device::reset() { *this=Device{}; }
void Device::write(uint32_t a, uint32_t v) {
    if (a&3) return;
    if (a==reg::STOP) {
        status_=reg::STOPPED; // cancel pending transactions
        for (unsigned i=0;i<motor_count;++i) {
            active_[i][field::ENABLE]=0; state_[i].duty_q16=0;
            state_[i].integral=0; state_[i].derivative=0; state_[i].target_mrpm=0;
        }
    } else if (a==reg::PERIOD_US) period_=v;
    else if (a==reg::KEEPALIVE) age_=0;
    else if (a==reg::COMMIT) {
        if (status_&reg::PENDING) { status_|=reg::REJECTED; return; }
        bool ok=period_>=100 && period_<=100000;
        for (unsigned i=0;i<motor_count;++i) ok=ok && valid(shadow_[i]);
        if (!ok) { status_|=reg::REJECTED; return; }
        for (unsigned i=0;i<motor_count;++i) pending_[i]=shadow_[i];
        pending_period_=period_; token_=v;
        status_=(status_&reg::STOPPED)|reg::PENDING;
    } else if (a==reg::SNAPSHOT) {
        for (unsigned i=0;i<motor_count;++i) snapshot_[i]=state_[i];
        snapshot_sequence_=tick_sequence_;
    } else if (a>=reg::CONFIG_BASE && a<reg::CONFIG_BASE+motor_count*reg::CONFIG_STRIDE) {
        unsigned m=(a-reg::CONFIG_BASE)/reg::CONFIG_STRIDE;
        unsigned f=((a-reg::CONFIG_BASE)%reg::CONFIG_STRIDE)/4;
        if (f<field::COUNT) shadow_[m].words[f]=signed_word(v);
    }
}
uint32_t Device::read(uint32_t a) const {
    if (a&3) return 0;
    switch(a) {
        case reg::ID:return reg::MAGIC;
        case reg::ABI:return reg::ABI_VERSION;
        case reg::MOTORS:return motor_count;
        case reg::PERIOD_US:return period_;
        case reg::APPLIED:return applied_;
        case reg::STATUS:return status_;
        case reg::SNAPSHOT_SEQUENCE:return snapshot_sequence_;
    }
    if (a>=reg::CONFIG_BASE && a<reg::CONFIG_BASE+motor_count*reg::CONFIG_STRIDE) {
        unsigned m=(a-reg::CONFIG_BASE)/reg::CONFIG_STRIDE;
        unsigned f=((a-reg::CONFIG_BASE)%reg::CONFIG_STRIDE)/4;
        return f<field::COUNT ? uint32_t(shadow_[m].words[f]) : 0;
    }
    if (a>=reg::TELEMETRY_BASE && a<reg::TELEMETRY_BASE+motor_count*reg::TELEMETRY_STRIDE) {
        unsigned m=(a-reg::TELEMETRY_BASE)/reg::TELEMETRY_STRIDE;
        const WheelState& s=snapshot_[m];
        switch((a-reg::TELEMETRY_BASE)%reg::TELEMETRY_STRIDE) {
            case reg::POS_LO:return uint32_t(s.position);
            case reg::POS_HI:return uint32_t(uint64_t(s.position)>>32);
            case reg::SPEED:return uint32_t(s.speed_mrpm);
            case reg::DUTY:return uint32_t(s.duty_q16);
            case reg::FAULTS:return s.faults;
            case reg::ENCODER_ERRORS:return s.encoder_errors;
            case reg::RAMPED_TARGET:return uint32_t(s.target_mrpm);
        }
    }
    return 0;
}
void Device::tick(const EncoderSnapshot feedback[motor_count]) {
    if (status_&reg::PENDING) {
        for (unsigned i=0;i<motor_count;++i) {
            // Encoder scaling changes create a new position epoch.
            if (active_[i][field::COUNTS_PER_REV_Q16]!=pending_[i][field::COUNTS_PER_REV_Q16] ||
                active_[i][field::ENCODER_SIGN]!=pending_[i][field::ENCODER_SIGN]) state_[i]=WheelState{};
            // Disable/apply is the explicit acknowledgement of latched faults.
            if (!pending_[i][field::ENABLE]) state_[i].faults=0;
            bool retuned=false;
            for (unsigned f=field::KP_Q16;f<=field::PWM_OFFSET_REV_Q16;++f)
                retuned=retuned || active_[i].words[f]!=pending_[i].words[f];
            if (retuned) { state_[i].integral=0; state_[i].derivative=0; }
            active_[i]=pending_[i];
        }
        active_period_=pending_period_; applied_=token_; status_=0; age_=0;
    }
    for (unsigned i=0;i<motor_count;++i)
        wheel_step(active_[i],feedback[i],active_period_,age_>=uint32_t(active_[i][field::WATCHDOG_US]),state_[i]);
    if (age_<1000000000ULL) age_+=active_period_;
    ++tick_sequence_;
}
}
