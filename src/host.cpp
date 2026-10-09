#include "dvrmc/host.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace dvrmc {
static int32_t quantize(double v,double scale) {
    double x=std::round(v*scale);
    if (!std::isfinite(x) || x<std::numeric_limits<int32_t>::min() ||
        x>std::numeric_limits<int32_t>::max()) throw std::invalid_argument("Parameter is not representable");
    return int32_t(x);
}
MappedIo::MappedIo(volatile uint32_t* p,std::size_t n,void(*b)()):base_(p),bytes_(n),barrier_(b) {
    if (!p || !b || reinterpret_cast<uintptr_t>(p)%4) throw std::invalid_argument("Invalid MMIO mapping/barrier");
}
void MappedIo::check(uint32_t a) const {
    if ((a&3) || std::size_t(a)+4>bytes_) throw std::out_of_range("MMIO offset");
}
uint32_t MappedIo::read32(uint32_t a) { check(a); barrier_(); auto v=base_[a/4]; barrier_(); return v; }
void MappedIo::write32(uint32_t a,uint32_t v) { check(a); barrier_(); base_[a/4]=v; barrier_(); }
MotorProperties MotorProperties::jga25_370_6v_130rpm(uint32_t radius) {
    MotorProperties p;
    p.encoder_ppr=11; p.gear_numerator=468; p.gear_denominator=10;
    p.wheel_radius_um=radius; p.max_speed_mrpm=130000; p.nominal_voltage_mv=6000;
    return p;
}
MotorConfig make_config(const MotorProperties& p,const PidParameters& pid,const Calibration& cal) {
    using namespace field;
    if (!p.gear_denominator) throw std::invalid_argument("Zero gear denominator");
    MotorConfig c;
    c[ENCODER_PPR]=quantize(p.encoder_ppr,1);
    c[GEAR_NUM]=quantize(p.gear_numerator,1); c[GEAR_DEN]=quantize(p.gear_denominator,1);
    c[WHEEL_RADIUS_UM]=quantize(p.wheel_radius_um,1); c[MAX_SPEED_MRPM]=quantize(p.max_speed_mrpm,1);
    c[NOMINAL_VOLTAGE_MV]=quantize(p.nominal_voltage_mv,1); c[CURRENT_LIMIT_MA]=quantize(p.current_limit_ma,1);
    c[ENCODER_SIGN]=p.encoder_sign; c[DRIVE_SIGN]=p.drive_sign;
    c[ENCODER_SCALE_PPM]=quantize(cal.encoder_scale,1000000);
    c[COUNTS_PER_REV_Q16]=quantize(4.0*p.encoder_ppr*p.gear_numerator/p.gear_denominator*
                                  (double(c[ENCODER_SCALE_PPM])/1000000),one);
    c[KP_Q16]=quantize(pid.kp,one); c[KI_Q16]=quantize(pid.ki,one);
    c[KD_Q16]=quantize(pid.kd,one); c[KFF_Q16]=quantize(pid.feedforward,one);
    c[D_ALPHA_Q16]=quantize(pid.derivative_alpha,one);
    c[I_LIMIT_Q16]=quantize(pid.integral_limit,one); c[DUTY_LIMIT_Q16]=quantize(pid.duty_limit,one);
    c[PWM_GAIN_FWD_Q16]=quantize(cal.forward_gain,one); c[PWM_GAIN_REV_Q16]=quantize(cal.reverse_gain,one);
    c[PWM_OFFSET_FWD_Q16]=quantize(cal.forward_offset,one); c[PWM_OFFSET_REV_Q16]=quantize(cal.reverse_offset,one);
    c[MAX_ACCEL_MRPM_S]=p.max_speed_mrpm; // one second to rated speed; configurable
    if (!valid(c)) throw std::invalid_argument("Invalid motor/PID/calibration configuration");
    return c;
}
MotorController::MotorController(RegisterIo& io):io_(io) {
    if (io_.read32(reg::ID)!=reg::MAGIC || io_.read32(reg::ABI)!=reg::ABI_VERSION ||
        io_.read32(reg::MOTORS)!=motor_count) throw std::runtime_error("Unsupported controller ABI");
    next_token_=io_.read32(reg::APPLIED);
}
void MotorController::check_motor(unsigned m) const { if (m>=motor_count) throw std::out_of_range("Motor index"); }
void MotorController::configure(unsigned m,const MotorConfig& c) {
    check_motor(m);
    if (!valid(c)) throw std::invalid_argument("Invalid motor configuration");
    for (unsigned f=0;f<field::COUNT;++f) io_.write32(reg::config(m,field::Index(f)),uint32_t(c.words[f]));
}
void MotorController::speed(unsigned m,int32_t rpm,bool en) {
    check_motor(m);
    auto limit=signed_word(io_.read32(reg::config(m,field::MAX_SPEED_MRPM)));
    if (limit<=0 || int64_t(rpm)>limit || int64_t(rpm)<-int64_t(limit)) throw std::invalid_argument("Speed out of range");
    io_.write32(reg::config(m,field::TARGET_MRPM),uint32_t(rpm));
    io_.write32(reg::config(m,field::ENABLE),en ? 1:0);
}
void MotorController::period_us(uint32_t p) {
    if (p<100 || p>100000) throw std::invalid_argument("Control period out of range");
    io_.write32(reg::PERIOD_US,p);
}
uint32_t MotorController::commit() {
    if (io_.read32(reg::STATUS)&reg::PENDING) throw std::runtime_error("Previous commit is pending");
    uint32_t t=++next_token_; io_.write32(reg::COMMIT,t);
    if (io_.read32(reg::STATUS)&reg::REJECTED) throw std::runtime_error("Configuration rejected");
    return t;
}
bool MotorController::applied(uint32_t t) {
    return !(io_.read32(reg::STATUS)&reg::PENDING) && io_.read32(reg::APPLIED)==t;
}
void MotorController::stop() { io_.write32(reg::STOP,1); }
void MotorController::keepalive() { io_.write32(reg::KEEPALIVE,1); }
Snapshot MotorController::snapshot() {
    io_.write32(reg::SNAPSHOT,1); Snapshot s;
    s.sequence=io_.read32(reg::SNAPSHOT_SEQUENCE);
    for (unsigned m=0;m<motor_count;++m) {
        auto read=[&](uint32_t f){return io_.read32(reg::telemetry(m,f));};
        uint64_t bits=uint64_t(read(reg::POS_LO)) | (uint64_t(read(reg::POS_HI))<<32);
        s.motors[m].position=bits<=uint64_t(INT64_MAX) ? int64_t(bits) : -1-int64_t(~bits);
        s.motors[m].speed_mrpm=signed_word(read(reg::SPEED));
        s.motors[m].duty_q16=signed_word(read(reg::DUTY));
        s.motors[m].target_mrpm=signed_word(read(reg::RAMPED_TARGET));
        s.motors[m].faults=read(reg::FAULTS); s.motors[m].encoder_errors=read(reg::ENCODER_ERRORS);
    }
    return s;
}
DifferentialDrive::DifferentialDrive(MotorController& c,double b,double l,double r,double lm,double rm)
:controller_(c),track_(b),left_radius_(l),right_radius_(r),left_max_(lm),right_max_(rm) {
    for (double v:{b,l,r,lm,rm}) if (!std::isfinite(v) || v<=0) throw std::invalid_argument("Invalid drive geometry/limits");
}
uint32_t DifferentialDrive::velocity(double v,double w) {
    constexpr double pi=3.14159265358979323846;
    double l=(v-w*track_/2)*60/(2*pi*left_radius_);
    double r=(v+w*track_/2)*60/(2*pi*right_radius_);
    if (!std::isfinite(l) || !std::isfinite(r)) throw std::invalid_argument("Invalid velocity");
    double scale=std::max(1.0,std::max(std::abs(l)/left_max_,std::abs(r)/right_max_));
    controller_.speed(0,quantize(l/scale,1000)); controller_.speed(1,quantize(r/scale,1000));
    return controller_.commit();
}
}
