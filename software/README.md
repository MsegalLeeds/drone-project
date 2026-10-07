# Software

Software will eventually include:

```text
flight-controller/   Embedded flight-control software
simulation/           Dynamics / control simulation
ground-station/       Telemetry and operator interface
tools/                Analysis and engineering utilities
```

Develop incrementally: unit-test algorithms where possible, simulate before hardware tests, and keep hardware-dependent code separated from reusable control logic.
