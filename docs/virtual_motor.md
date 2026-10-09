# Virtual motor bench

The native bench closes the loop through controller duty -> switching PWM ->
DC motor and gearbox -> Hall A/B -> filter and quadrature decoder -> controller.
The controller receives decoded positions, never the model's true velocity.
Two motors have independent physical parameters and fault inputs.

## Run

From the repository root:

```
cmake -S . -B build -DDVRMC_SANITIZE=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/virtual_motor build/motor.csv build/signals.vcd
```

The CSV records each motor every millisecond: ramped target, true and measured
RPM, output turns, current, commanded duty, Hall levels, raw count, encoder errors
and controller faults. VCD is optional and records PWM/reverse/Hall transitions
at 1 us resolution for waveform viewers. CSV Hall samples alone are too sparse
to inspect individual encoder transitions. The scenario is deterministic:

| Time (s) | Action |
|---|---|
| 0 | Command both motors to +60 RPM |
| 1.5 | Add 0.035 Nm opposing output load to the right motor |
| 2.5 | Command both motors to -45 RPM |
| 4.0 | Lock the left rotor |
| 4.5 | Release it |
| 5.2 | Freeze right Hall outputs while the motor continues to move |
| 5.5 | Restore Hall outputs |
| 6.0 | Stop host keepalives; observe watchdog shutdown |
| 7.0 | End |

Edit `examples/virtual_motor.cpp` to issue other commands using `MotorController`,
set per-motor `faults`, or construct `sim::Rig(left_parameters,right_parameters)`.
`advance()` executes microsecond steps. Its optional observer permits logging
signals or connecting a different test harness. Recreate a rig to reset it or
change immutable physical parameters. Faults can be changed between any steps.

## Model and assumptions

`include/dvrmc/sim/plant.hpp` is host/testbench-only C++14. It is deliberately
outside the synthesizable controller. Parameters include supply, resistance,
inductance, torque/back-EMF constants, rotor and output-load inertia, viscous and
Coulomb friction, gear ratio/efficiency, encoder PPR, PWM rate and filter length.

Electrical dynamics use L di/dt = V - R i - Ke omega, integrated exactly over
one step with velocity held constant. Mechanical dynamics use torque Kt i,
viscous friction, Coulomb friction and opposing output load reflected through
the gearbox. Output inertia is reflected by gear ratio squared. Rotor angle
creates the Gray sequence 00,01,11,10 with four edges per channel pulse.
At 11 PPR and 46.8 reduction there are 2059.2 counts per output revolution.

The 6 V supply, 46.8 ratio and 11 PPR come from the specified motor. R=6 ohm,
L=3 mH, Kt=Ke=0.0094, inertias and friction are illustrative estimates, not
measurements or a manufacturer specification. The resulting unloaded speed is
near 130 RPM. Example PID gains are only tuned for this virtual plant.

The bridge uses sign/magnitude PWM and synchronous shorted-winding recirculation
at zero voltage during PWM off time (including zero duty). Duty is latched at
carrier boundaries. At 20 kHz the fixed 1 us grid provides 50 samples/period;
on time is quantized upward to the next sample. Full positive/negative duty
and zero duty are exact. Requested PWM frequencies must divide 1 MHz.
`driver_open` models ideal high impedance with current immediately zero;
it omits the opening inductive transient. There is no transistor dead time,
thermal model, gear backlash, compliance, battery impedance or current limiter.
Do not use its current peaks to size real switching components.

The decoder requires a configurable number of consecutive equal A/B samples.
Two-bit transitions increment its error count without adding position. Counter
arithmetic wraps at 32 bits. This is a simulation reference, not synthesized
encoder/PWM RTL or a model of metastability. Sample/filter/PWM parameters belong
to the rig; controller metadata registers do not dynamically reconfigure them.
The bench fixes control cadence to 1 ms. If changing the controller period,
change the test harness cadence too. HLS calls complete instantaneously in model
time; RTL invocation latency is tested separately by co-simulation, not charged
to the plant's scheduler. Deadline/CDC/hardware shell verification remains work.

## Fault injection and checks

`sim::Faults` exposes locked shaft, open driver, supply scaling, opposing load,
frozen Hall signals, swapped A/B, stuck-low A or B, and an A/B XOR mask. Apply the
mask for a chosen number of microsteps to inject reproducible short glitches or
persistent invalid transitions. Restore flags to remove the injected condition.
Missing Hall pulses can be modeled by freezing the outputs during selected edges.

Automated tests cover PWM duty, electrical stall current, Hall/angle agreement,
filtering, illegal transitions, reversed counting and wraparound, supply sag,
swapped feedback, closed-loop speed tracking, asymmetric load, reverse commands,
locked-rotor recovery, Hall loss, watchdog shutdown and STOP reaching PWM.
The core currently does NOT latch a stall fault or automatically stop on encoder
error count. The stall test explicitly records that limitation; simulated motor
current is observable but is not a feedback/protection input to the controller.
PID internal terms are not exposed by the existing telemetry ABI.

## Vitis HLS

A separate `tests/plant_top_tb.cpp` drives the same virtual rig through actual
`dvrmc_top` register calls. It checks +60 RPM tracking, watchdog shutdown and
STOP. The broader fault suite runs natively above.

```
source /opt/vitis/2026.1/Vitis/settings64.sh
cd platforms/vitis
vitis-run --mode hls --csim --config plant_config.cfg --work_dir ../../work/vitis_virtual_motor
v++ --compile --mode hls --config plant_config.cfg --work_dir ../../work/vitis_virtual_motor
vitis-run --mode hls --cosim --config plant_config.cfg --work_dir ../../work/vitis_virtual_motor
```

In the already-open Vitis workspace (`work`), use the `vitis_virtual_motor`
component created by the first command. The synthesizable source list still
contains only the controller; the plant is testbench code. Co-simulation replaces
only that controller with RTL: Vitis first records the C closed-loop stimulus
and then replays it against RTL and compares outputs. It is not an event-driven
RTL/analog co-simulator. PWM, mechanics and Hall decoding remain C++ models.
The same header/testbench can be used by SmartHLS, but its virtual-plant flow has
not been validated here; its existing short top testbench remains unchanged.
