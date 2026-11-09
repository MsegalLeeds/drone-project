# Custom UAV Engineering Project

A multidisciplinary university project to **design, manufacture, program, test and document a custom unmanned aerial vehicle (UAV)**.

The project combines mechanical engineering, electronics, embedded software, control systems, simulation and experimental validation.

## Project goals

- Define measurable aircraft requirements and success criteria.
- Design and manufacture a suitable airframe.
- Size and validate the propulsion and power systems.
- Develop the flight-control and embedded software.
- Build telemetry and ground-station tooling.
- Simulate and test the control system before flight.
- Conduct controlled bench and flight experiments.
- Record engineering decisions, calculations, failures and results.
- Produce a professional technical portfolio alongside the formal team report.

## Team

| Area | Lead |
|---|---|
| Software / embedded / flight control | Marc |
| Mechanical design / CAD / manufacturing | Robbie |
| Mechanical design / propulsion / testing | Naomi |

Ownership can change as the project develops; important decisions should be recorded in GitHub issues and decision records.

## Repository structure

```text
docs/          Requirements, architecture and engineering decisions
hardware/      CAD, electronics, manufacturing and drawings
software/      Flight controller, simulation and ground-station code
experiments/   Experimental plans, measurements and results
media/         Selected project photographs and videos
.github/       Issue and pull-request templates
```

## Working in CLion

Open this repository folder in CLion and let CMake load. Select **drone_sim** and click Run to start the simulator on your computer. Run **All CTest** for the flight-controller tests.

On macOS, select **stm32_blink** or **stm32_bench** and click Build to compile firmware with the board unplugged. The [STM32 C guide](software/stm32/blue-led/STM32%20C%20guide.md) covers editing and uploading the LED program. The [USB bench instructions](software/stm32/usb-bench/README.md) cover tilt replay and pressure readings. The [bench checklist](docs/bring-up/todo.md) records which hardware tests remain open.

## Engineering workflow

Every significant piece of work should follow:

```text
Issue
  ↓
Research / calculation
  ↓
Design
  ↓
Implementation / prototype
  ↓
Pull request
  ↓
Review
  ↓
Test / experiment
  ↓
Measured result
  ↓
Decision / iteration
```

Use GitHub for the **live engineering record**. The formal Word report can contain the polished final narrative, while this repository preserves the working process, evidence and history.

## Current development stages

| Area | Status |
|---|---|
| Requirements | Started |
| System architecture |  Started |
| Propulsion sizing | Not started |
| Frame design | Not started |
| Electronics |  Started |
| Flight controller |  Started |
| Simulation / HIL |  Started |
| Manufacturing | Not started |
| Bench testing | USB tilt replay passed. Barometer awaiting soldered headers. |
| Flight testing | Not started |
| Autonomous features | Not started |

## Safety

No powered propulsion or flight testing should take place until the relevant university safety procedures, risk assessments, supervision requirements and test conditions have been established.

## First tasks

1. Define system requirements — Issue #1
2. Survey available University of Leeds facilities — Issue #2
3. Define system architecture — Issue #3
4. Perform initial propulsion sizing — Issue #4
5. Define safety and test process — Issue #5
6. Set up team workflow and repository ownership — Issue #6
