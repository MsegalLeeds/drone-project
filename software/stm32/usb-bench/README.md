# STM32 tilt bench

[Show tilt results.command](<Show tilt results.command>) reads the latest run from the board. If the board has been reset, the launcher walks you through loading the saved RAM image. Press RST by itself to return to the original firmware. The LED stays blue when the replay and bench checks pass.

The same project sources run on both computers, including the simulated IMU, attitude estimator, three-axis flight controller, PID and Quad-X mixer. The input is generated in [replay.c](replay.c): 20 level samples, 20 rolling right at 50°/s, 40 holding 10°, 20 returning at 50°/s, then 60 level samples. The period is 10 ms, the target attitude is level, and the simulated throttle is 50%. The four motor values stay in memory. The program also configures PB8 and PB9 for the barometer check.

## Measured on 8 November 2026

| Checkpoint | Estimated roll | Estimated pitch | Front left | Front right | Rear right | Rear left |
|---|---:|---:|---:|---:|---:|---:|
| Level | 0° | 0° | 50% | 50% | 50% | 50% |
| Right side down | 10° | 0° | 25% | 75% | 75% | 25% |
| Level again | 0° | 0° | 50% | 50% | 50% | 50% |

The recorded Mac and STM32 values agreed to five decimal places at these checkpoints. The automated comparison allows 0.05° for angles and 0.002 for motor commands on the 0–1 scale. The STM32 measured a peak replay-step duration of 33 µs and maximum start lateness of 6 µs across 160 scheduled steps. These timings include generation of the artificial sample and the controller calculation. Real sensor I/O and motor signalling are future work. [Mac reference](../../../experiments/stm32-usb/mac.csv) · [USB report](../../../experiments/stm32-usb/board-report.txt)

The separate [bench interlock tests](faults.c) exercise high-throttle arming, missing IMU, stale data, an invalid reading, a late update, lost radio, disarm and restart. Each checks refusal or loss of simulated permission and deliberate rearming. All eight bits set in the report give 255. Aircraft failsafe actions and real motor inhibition still need implementation and physical testing.

## Rebuild after editing

Run `./build.sh` from this folder. The script compiles the Mac reference from the existing CLion project and builds the Arm image with the same C modules. Set `FLIGHT_CONTROLLER_DIR` to use a different checkout. The board build uses Arm GNU, CMSIS and TinyUSB. Their versions and sources are in the script. By default its downloads are cached in the Mac's temporary directory. Set `BENCH_DEPS` to retain them elsewhere.

To run a newly built image, put the board into BOOT mode before opening the launcher. Opening the launcher while the bench is already running reads the stored measurements from that boot.

## Read the barometer

In CLion build **stm32_bench**, then enter `software/stm32/usb-bench` in Terminal. With USB connected, hold BOOT, tap and release RST, then release BOOT. Run `./"Show tilt results.command"` to load the program and verify the tilt replay. Run `python3 read_board.py --barometer` for three sensor samples. The latest hardware attempt returned `BAROMETER,NO_ACK,weak_pulldown_B8=0,B9=0`. The next test requires soldered headers and verified continuity.

The [saved bench files](../../../experiments/stm32-usb/) include the USB report, Mac reference and the RAM images from the session. The report records a later replay with a 32 µs peak step and 5 µs maximum start lateness.
