#pragma once
#include <cstddef>
#include "device.hpp"
namespace dvrmc {
// Ordered 32-bit, little-endian register transactions. Implement in a platform
// driver, UIO mapping, PCIe transport, or the deterministic simulator below.
class RegisterIo {
public:
    virtual ~RegisterIo() = default;
    virtual uint32_t read32(uint32_t offset) = 0;
    virtual void write32(uint32_t offset, uint32_t value) = 0;
};
class ModelIo final : public RegisterIo {
public:
    Device device;
    uint32_t read32(uint32_t a) override { return device.read(a); }
    void write32(uint32_t a,uint32_t v) override { device.write(a,v); }
    void tick(const EncoderSnapshot e[motor_count]) { device.tick(e); }
};
class MappedIo final : public RegisterIo {
public:
    // Mapping must be device/uncached memory. Barrier must implement the
    // platform's MMIO ordering rules; mapping ownership stays with caller.
    MappedIo(volatile uint32_t* base, std::size_t bytes, void (*barrier)());
    uint32_t read32(uint32_t a) override;
    void write32(uint32_t a,uint32_t v) override;
private:
    volatile uint32_t* base_; std::size_t bytes_; void (*barrier_)();
    void check(uint32_t a) const;
};
struct MotorProperties {
    uint32_t encoder_ppr=0, gear_numerator=0, gear_denominator=1;
    uint32_t wheel_radius_um=0, max_speed_mrpm=0, nominal_voltage_mv=0;
    uint32_t current_limit_ma=0;
    int encoder_sign=1, drive_sign=1;
    static MotorProperties jga25_370_6v_130rpm(uint32_t wheel_radius_um);
};
struct PidParameters {
    double kp=0, ki=0, kd=0, feedforward=0;
    double derivative_alpha=1, integral_limit=1, duty_limit=1;
};
struct Calibration {
    double encoder_scale=1;
    double forward_gain=1, reverse_gain=1;
    double forward_offset=0, reverse_offset=0;
};
// Hardware rates, acceleration, watchdog and estimator settings stay explicit
// through MotorConfig fields; this factory fills the physical/PID calibration.
MotorConfig make_config(const MotorProperties&, const PidParameters&, const Calibration&);
struct Telemetry {
    int64_t position=0;
    int32_t speed_mrpm=0, duty_q16=0, target_mrpm=0;
    uint32_t faults=0, encoder_errors=0;
};
struct Snapshot { uint32_t sequence=0; Telemetry motors[motor_count]; };
class MotorController {
public:
    explicit MotorController(RegisterIo& io);
    void configure(unsigned motor, const MotorConfig& config);
    void speed(unsigned motor, int32_t milli_rpm, bool enable=true);
    void period_us(uint32_t period);
    uint32_t commit();
    bool applied(uint32_t token);
    void stop();
    void keepalive();
    Snapshot snapshot();
private:
    RegisterIo& io_; uint32_t next_token_=0;
    void check_motor(unsigned motor) const;
};
class DifferentialDrive {
public:
    DifferentialDrive(MotorController& controller, double track_width_m,
                      double left_radius_m, double right_radius_m,
                      double left_max_rpm, double right_max_rpm);
    // Stages BOTH targets and commits once. Limits preserve turning curvature.
    uint32_t velocity(double metres_per_second, double radians_per_second);
private:
    MotorController& controller_;
    double track_, left_radius_, right_radius_, left_max_, right_max_;
};
}
