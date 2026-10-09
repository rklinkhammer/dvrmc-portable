# Architecture and implementation status

The controller has three boundaries:

1. Portable measurement/actuation RTL: asynchronous Hall inputs -> target CDC
   adapter -> configurable filter/decoder -> timestamped counters; signed effort
   -> PWM plus coordinated bridge direction/brake/enable. These blocks and the
   hard fault gate are planned, not implemented in this repository yet.
2. Shared C++ control engine: encoder snapshots, speed estimation, acceleration
   limiting, PID plus feedforward, calibration and output limiting. Implemented.
3. Ordered host register transport: ABI identical on both architectures;
   board-specific AXI/APB integration adapts to HLS invocation/result handshakes.
   Host API and executable register model are implemented; RTL bus adapters and
   OS drivers are planned.

No stream can backpressure encoder capture in the intended architecture.
A common scheduler snapshots both encoder positions on a control tick, invokes
`dvrmc_top`, then latches both returned duties together at a PWM boundary.
The generated HLS execution time may differ between tools. The scheduler must
prioritize ticks over bus traffic, enforce deadlines and drive a fault gate on
missed deadlines. Input clocks and counters are independent of HLS busy state.
STOP zeros the software/model output in its transaction; hardware emergency stop
must bypass a busy HLS engine in the future RTL gate.

The current top combines register processing and the control engine into one
HLS transaction. A write is handled before a requested tick; a read returns
state after that transaction. Reset clears the whole device before either.
The host must hold input arguments for the generated handshake and read outputs
only when complete. Future vendor wrappers must implement the same ordering.

The control period register is 100..100000 us. The board scheduler must use the
committed period, rather than changing the time scale without changing cadence.
Shadow writes do not change cadence. At commit, the future shell must apply
PWM/sample/filter settings atomically with the active C++ configuration; current
limit programming must either be supported by the driver or rejected by that
platform. Nominal voltage is metadata; it is not a voltage/current measurement.

## Control semantics

Speed units are output-shaft milli-RPM. Encoder snapshots are raw unsigned
32-bit quadrature positions; subtraction is modulo 2^32 with a signed delta.
ENCODER_SIGN normalizes forward. A delta exceeding MAX_DELTA_COUNTS latches a
fault and disables effort. The first snapshot primes without movement. Accepted
position deltas accumulate in a saturating signed 64-bit counter.

COUNTS_PER_REV_Q16 represents effective quadrature counts per output revolution.
The host computes it from PPR * 4 * gear ratio * encoder scale. The register is
a derived value and is authoritative in the datapath; low-level clients are
responsible for writing it consistently. Its signed Q16.16 representation limits
counts/revolution to below 32768; larger encoders need an ABI/data-width change.
The x4 quadrature mode is an architectural choice, not an adjustable motor PPR.

The first estimator is counts-per-window plus a configurable first-order filter.
Timestamp/period-based low-speed estimation is a future extension. The estimator
clamps to +/-1000 RPM. The intended supported motor maximum is at most 1000 RPM.
A small fixed-point residual can remain near zero; no standstill detector is
implemented yet. Encoder error count is exposed, not automatically a fault.

PID computes normalized signed effort in Q16.16 (65536 = full duty). Gains:
Kp and feedforward: duty/RPM; Ki: duty/(RPM*s); Kd: duty*s/RPM.
Derivative is taken on filtered measured speed, with another configurable filter.
Integral is bounded and conditionally frozen when the calibrated output saturates.
Changing PID/calibration resets integral and derivative; changing only target
speed preserves them. Disabled/faulted states reset the control terms.
Encoder scale/sign changes start a new position epoch; software must account
for this. Such changes should be made while stopped.

A keepalive or a successfully applied commit refreshes the watchdog age.
WATCHDOG and BAD_FEEDBACK latch until a disabled configuration is committed.
Raw invalid configurations are rejected before application. STOP cancels a
pending commit and disables both active channels, independently of control tick.
A later explicit valid commit can enable again; no automatic recovery occurs.

The shared arithmetic uses bounded int64 intermediates with explicit saturation;
no implementation-defined negative shifts or signed overflow are required.
The two wheel controllers run independently; DifferentialDrive supplies shared
commands and turning-curvature-preserving saturation on the host.

## Follow-on components

Distance/turn trajectory generation, settled-position completion, edge-period
estimation, current feedback, fault-gated PWM, CDC constraints, physical register
bus bridges, firmware interrupts and board pin files are not implemented yet.
The previous encoder project remains available as an AMD-specific reference;
its XPM macro must not be copied into the portable core.

Every new platform should run the common native/reference tests, its own HLS
C/RTL tests, hardware-in-loop load tests and final timing/CDC analysis. Passing
C simulation alone does not establish board safety, timing or portability.
