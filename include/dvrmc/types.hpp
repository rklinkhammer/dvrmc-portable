#pragma once
#include <cstdint>
namespace dvrmc {
constexpr unsigned motor_count = 2;
constexpr int32_t one = 65536;
namespace field {
enum Index {
#define FIELD(name, value, description) name,
#include "config_fields.def"
#undef FIELD
    COUNT
};
}
struct MotorConfig {
    int32_t words[field::COUNT];
    MotorConfig() {
#define FIELD(name, value, description) words[field::name] = value;
#include "config_fields.def"
#undef FIELD
    }
    int32_t& operator[](field::Index i) { return words[i]; }
    int32_t operator[](field::Index i) const { return words[i]; }
};
struct EncoderSnapshot { uint32_t position = 0, errors = 0; };
enum Fault : uint32_t { NO_FAULT=0, WATCHDOG=1, BAD_FEEDBACK=2, BAD_CONFIG=4 };
struct WheelState {
    bool primed = false;
    uint32_t previous_position = 0;
    int64_t position = 0;
    int32_t speed_mrpm = 0, target_mrpm = 0, integral = 0, derivative = 0;
    int32_t previous_speed = 0, duty_q16 = 0;
    uint32_t faults = 0, encoder_errors = 0;
    uint64_t ramp_fraction = 0;
};
int32_t signed_word(uint32_t value);
bool valid(const MotorConfig& c);
void wheel_step(const MotorConfig& c, const EncoderSnapshot& encoder,
                uint32_t period_us, bool timed_out, WheelState& s);
}
