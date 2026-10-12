# Software

The [flight-controller engineering log](../docs/flight-controller-engineering-log.md) records the current local C/CLion setup, PID implementation, tests and control-system explanations. It distinguishes local development from source published in this repository.

Software will eventually include:

```text
flight-controller/   Embedded flight-control software
simulation/           Dynamics / control simulation
ground-station/       Telemetry and operator interface
tools/                Analysis and engineering utilities
```

Develop incrementally: unit-test algorithms where possible, simulate before hardware tests, and keep hardware-dependent code separated from reusable control logic.
