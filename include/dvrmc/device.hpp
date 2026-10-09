#pragma once
#include "registers.hpp"
namespace dvrmc {
// Synthesizable register/control engine; independent of CPU bus protocol.
class Device {
public:
    void reset();
    void write(uint32_t address, uint32_t value);
    uint32_t read(uint32_t address) const;
    void tick(const EncoderSnapshot feedback[motor_count]);
    int32_t duty(unsigned motor) const { return state_[motor].duty_q16; }
private:
    MotorConfig shadow_[motor_count], pending_[motor_count], active_[motor_count];
    WheelState state_[motor_count], snapshot_[motor_count];
    uint32_t period_=1000, pending_period_=1000, active_period_=1000;
    uint32_t token_=0, applied_=0, status_=reg::STOPPED;
    uint32_t tick_sequence_=0, snapshot_sequence_=0;
    uint64_t age_=0;
};
}
