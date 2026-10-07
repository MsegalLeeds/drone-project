# System Architecture

Initial architecture to be refined after requirements and propulsion sizing.

```text
                     ┌─────────────────┐
                     │  Ground Station  │
                     └────────┬────────┘
                              │ Telemetry / commands
                              ▼
┌──────────┐          ┌─────────────────┐          ┌──────────────┐
│ Sensors  │ ───────► │ Flight          │ ───────► │ ESCs / Motors│
└──────────┘          │ Controller      │          └──────────────┘
                      │ State estimation│
                      │ Flight control  │
                      │ Motor mixing    │
                      └────────┬────────┘
                               │
                         ┌─────▼─────┐
                         │ Power     │
                         │ System    │
                         └───────────┘
```

## Subsystems

- Airframe
- Propulsion
- Battery / power distribution
- Flight controller
- IMU and other sensors
- Communications / telemetry
- Ground station
- Payload, if required
- Safety and failsafe systems
