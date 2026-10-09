# Calibration and parameters

Keep separate corrections for distinct physical effects:

1. Encoder scale corrects effective counts per output revolution. Gear ratio is
   a rational numerator/denominator (the supplied motor uses 468/10). Encoder
   scale 1.0 leaves nominal resolution unchanged. This affects speed conversion,
   not the raw integer position count. A software distance conversion uses
   effective counts/revolution and measured wheel radius.
2. Effective wheel radius corrects travel per output revolution. Left and right
   radii may differ. DifferentialDrive accepts both and measured track width.
   It does not infer them from nominal wheel packaging dimensions.
3. PWM calibration corrects drive effort, independently by direction:

   positive: calibrated = effort * forward_gain + forward_offset
   negative: calibrated = effort * reverse_gain - reverse_offset
   zero: calibrated = 0

   Clamp to DUTY_LIMIT, then apply DRIVE_SIGN. ENCODER_SIGN separately normalizes
   feedback polarity. Defaults are gain 1, offset 0. Set encoder/geometry scale
   from measured motion, not as a way to hide a weak motor.
4. PID gains regulate speed under changing load. They are independent of PWM
   trim. Anti-windup uses the calibrated saturation so a large trim cannot make
   the integrator grow indefinitely. The current model has no current loop.

MotorProperties, PidParameters and Calibration are ordinary host C++ types.
make_config converts their physical units to register words and calculates the
x4 encoder resolution. All per-motor words are writable through the shared ABI.
The JGA25-370 preset does not guess wheel radius or current rating. Pass zero
radius for RPM-only operation; DifferentialDrive requires known positive radii.

Use stopped motors for scale/sign/timing changes, and commit both banks together.
PID retuning deliberately resets integral/derivative instead of promising a
bumpless transfer. Target-only commits preserve the integrator. Persist calibrated
settings in the application's configuration; this library has no implicit flash
writes. Confirm returned APPLIED token before relying on a new configuration.

Example API (configuration stages only; commit and await its token):

```cpp
auto motor = dvrmc::MotorProperties::jga25_370_6v_130rpm(measured_radius_um);
dvrmc::PidParameters pid; // zero gains until explicitly tuned
pid.kp = tuned_kp;
pid.ki = tuned_ki;
pid.kd = tuned_kd;
dvrmc::Calibration cal;
cal.forward_gain = measured_forward_trim;
cal.reverse_gain = measured_reverse_trim;
auto cfg = dvrmc::make_config(motor, pid, cal);
controller.configure(0, cfg);
// Configure motor 1 as well, then:
auto token = controller.commit();
// Poll controller.applied(token) with an application timeout.
```

Rated voltage and requested current limit are retained as parameters for the
future bridge/driver layer. They are not enforced by multiplying PWM alone.
Timing/filter registers likewise require the future hardware shell. ABI presence
must never be mistaken for implemented physical protection.
