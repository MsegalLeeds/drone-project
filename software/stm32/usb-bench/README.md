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

In CLion build **stm32_bench**, then enter `software/stm32/usb-bench` in Terminal. With USB connected, hold BOOT, tap and release RST, then release BOOT. Run `./"Show tilt results.command"` to load the program and verify the tilt replay. Run `python3 read_board.py --barometer` for three sensor samples. An early hardware attempt returned `BAROMETER,NO_ACK,weak_pulldown_B8=0,B9=0`. The BMP388 subsequently returned valid readings at 0x76, with intermittent no-ack errors in a later run.

The [saved bench files](../../../experiments/stm32-usb/) include the USB report, Mac reference and the RAM images from the session. The report records a later replay with a 32 µs peak step and 5 µs maximum start lateness.

## Identify the motion sensor over I²C

For the first bench test, use a four-pin Qwiic-to-female-jumper cable with the SparkFun LSM6DSO. With USB unplugged, connect GND to GND, 3V3 to 3V3, SDA to B9 and SCL to B8. Either side socket works. Use soldered headers on the STM32. The SPI pin plan is for later integration.

Build the current firmware with `./build.sh`. Put the board in programming mode and load it with `./"Show tilt results.command"`. Run `python3 read_board.py --imu-id`. A successful reply contains the I²C address (usually 0x6B) and the WHO_AM_I value 0x6C. This command checks identity only. [SparkFun wiring guide](https://learn.sparkfun.com/tutorials/qwiic-6dof-lsm6dso-breakout-hookup-guide/all) · [ST register definitions](https://github.com/STMicroelectronics/lsm6dso-pid/blob/master/lsm6dso_reg.h)

## Component tests in CLion

With the STM32 unplugged, **Build Project**, then run **All CTest** in the Debug profile. The nine controller groups exercise the PID, accelerometer/gyro simulation, attitude estimator, Quad-X mixer, motor dynamics and closed-loop control. Three bench groups also run on the Mac:

| CTest group | Pass condition |
|---|---|
| `bench_replay_and_faults` | All three tilt checkpoints match the expected angles and four motor values. All eight bench interlock cases pass. |
| `bench_sensor_registers` | The firmware reads a simulated register map through the real Bosch driver, converts to hPa and °C, recognises both motion-sensor addresses, and reports bus, identity, readiness and read failures. The first run needs GitHub access to cache the pinned Bosch source. |
| `bench_usb_reports` | The Mac reader accepts the recorded board report and valid sensor replies. Incomplete tilt reports, missed deadlines, incorrect IDs, intermittent no-ack replies and invalid measurements fail. |

For a **physical run**, build and load the current bench firmware first. In CLion’s CMake options set `-DSTM32_LIVE_COMPONENTS=board;barometer` for a board with the barometer attached. Choose `board;imu` for the Qwiic motion sensor. Reload CMake. Run the corresponding `live_board`, `live_barometer` or `live_imu` tests. Each selected device must reply. Clear the option to return to offline tests. The same checks can be run individually in Terminal with `python3 read_board.py`, `python3 read_board.py --barometer` or `python3 read_board.py --imu-id`.

The onboard LED has a visual test: load the [blink program](../blue-led/STM32%20C%20guide.md) and compare the observed pattern with the loop you built. The [hardware checklist](../../../docs/bring-up/todo.md) covers parts that require actual wiring or measurement.
