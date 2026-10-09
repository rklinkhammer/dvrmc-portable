# Validation record

Validated locally on 2026-10-09. No physical motor was driven.

- GCC 15.2 / CMake native build, C++14, address and undefined-behaviour sanitizers.
- Three CTest suites passed: controller/host tests (30 checks), common HLS top
  transaction tests, and two unequal toy motor models tracking 60 RPM within
  4 RPM after five simulated seconds. Model results are not physical tuning.
- Vitis HLS 2026.1 C simulation passed.
- Vitis HLS 2026.1 synthesis completed for xc7z020clg400-1 at a 100 MHz target.
  Estimated critical delay: 7.040 ns; synthesis latency range: 4..927 clocks
  (up to 9.27 us at 100 MHz). This is an estimate, not routed timing closure.
- Vitis C/RTL co-simulation passed 86 top-level transactions using the same
  top_tb.cpp, including staged configuration, applied token, control ticks and
  stop. Detailed fault/calibration stress coverage is in the native tests.
- SmartHLS 2026.1 software simulation, HLS RTL generation and Questa RTL
  co-simulation passed for representative PolarFireSoC MPFS250T at 100 MHz.
  Co-simulation exercised 86 calls: 13,999 cycles total, latency 11..699 cycles,
  average 161.77 cycles. This is simulation latency, not routed timing closure.
- Fresh Libero 2026.1.0.17 installation in an Ubuntu 24.04 Docker volume:
  installer exit zero, prerequisite status zero, batch Tcl startup passed.
  The host is Ubuntu 26.04.1; native Libero lacked libxml2.so.2.
- Libero/Synplify synthesis smoke test passed on the fresh installation with
  an eight-bit counter targeting PolarFire SoC. All three relocated host license
  daemons started successfully; tool license checkout and checkin succeeded.
- Full controller Libero synthesis is not yet validated. The initial run was
  stopped during TAO constraint mapping before Synplify; HLS RTL generation and
  RTL co-simulation above are separate, completed checks.
- Physical board implementation and motor validation: NOT RUN.

The software clear argument was renamed from `reset` to `clear` because it
collided with SmartHLS's generated hardware reset port in the co-simulation
harness. Native tests, Vitis C simulation and SmartHLS RTL co-simulation passed
after that change. Earlier Vitis synthesis/co-simulation results predate this
port-name-only change.

The Vitis build disables automatic loop pipelining in the register engine. The
initial automatic-pipeline schedule missed the target clock; the final serial
transaction schedule met the HLS estimate with enough budget for a 1 kHz loop.
There is no requirement for one register transaction per FPGA clock.

Tests explicitly cover startup defaults, transactional staging across two
motors, signed position differences, wrapping encoder counters, coherent frozen
snapshots, invalid configuration rejection, stop cancellation of pending enables,
forward/reverse PWM calibration, encoder scale, integral anti-windup, preservation
of integral state on target-only commands, command watchdog, latched-fault
acknowledgement and bounded arithmetic at the supported parameter extremes.
