# Register protocol, ABI 1.0

Addresses are byte offsets. Transactions are aligned 32-bit words. Bus adapters
must define the words little-endian. Signed values use two's complement.
A single software owner is required; callers must serialize staging/commit and
snapshot/read operations. This library is not internally thread-safe.

| Offset | Register | Behavior |
|---|---|---|
| 0x00 | ID | RO 0x4456524d |
| 0x04 | ABI | RO 0x00010000 |
| 0x08 | MOTORS | RO 2 |
| 0x0c | PERIOD_US | Shadow control period, default 1000 |
| 0x10 | COMMIT | WO transaction token; capture both shadow banks |
| 0x14 | APPLIED | RO most recently applied token |
| 0x18 | STATUS | RO bit0 pending, bit1 rejected, bit2 stopped |
| 0x1c | STOP | Any write cancels pending configuration and zeros both duties |
| 0x20 | KEEPALIVE | Any write refreshes command age |
| 0x24 | SNAPSHOT | Any write freezes both telemetry banks |
| 0x28 | SNAPSHOT_SEQUENCE | Control tick index of frozen snapshot |
| 0x100 + m*0x100 | Motor configuration | See generated parameter table |
| 0x1000 + m*0x100 | Motor telemetry | Frozen fields below |

Configuration writes stage in shadow registers. COMMIT validates both motors and
captures an immutable pending copy. Later shadow writes cannot affect it. A commit
while pending is rejected. At the next control tick both copies become active and
APPLIED becomes the submitted token. A valid commit clears REJECTED; STOP also
clears it. STOPPED means the global stop latch, not an assertion that motor shaft
speed is zero. A valid commit clears it even if both motors remain disabled.

Both motor configurations must be valid even when disabled. The boot shadow
contains unspecified motor geometry; configure both before the first commit.
Faulted operation requires a disabled commit to acknowledge the fault. STOP does
not erase position or fault diagnostics. Reset clears everything and starts a
new position/snapshot/token epoch. Software must reinitialize its history.

Telemetry offsets within each motor bank:

| Offset | Field |
|---|---|
| 0x00 | Signed accumulated position bits 31:0 |
| 0x04 | Position bits 63:32 |
| 0x08 | Signed measured speed, milli-RPM |
| 0x0c | Signed output duty Q16.16, after DRIVE_SIGN |
| 0x10 | Fault bits: watchdog=1, implausible feedback=2, invalid active config=4 |
| 0x14 | Encoder invalid-transition count, as supplied by capture hardware |
| 0x18 | Signed ramped speed target, milli-RPM |

Read telemetry only after SNAPSHOT; it stays frozen despite subsequent control
ticks. Splitting a 64-bit position into two reads is therefore coherent. There is
no bus-time automatic snapshot. Unknown reads return zero; unknown/unaligned
writes are ignored by the engine (a physical bus wrapper may instead flag them).

Validation bounds are authoritative in src/control.cpp::valid. PID/feedforward
gains are 0..16; duty/integral limits and filter weights are 0..1 (speed weight
must be >0); PWM gains are >0..2; PWM offsets are 0..1. Raw values are fixed point.
Position calibration is 0.1..10; maximum plausible delta 1..65535 counts/tick;
watchdog 100 us..10 s. Rate compatibility with FPGA clock and current-limit
hardware support must be additionally validated by the target shell.
