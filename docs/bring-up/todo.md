# Drone bench checklist

Tick a box after observing its result. The [build plan](plan.md) explains the connections and order.

## Mac and USB only

- [x] The STM32 ran the temporary blue-LED demo. Two flashes and a pause were observed.
- [x] All nine flight-controller tests passed in a fresh Mac build on 8 November 2026.
- [x] Save a short level → right tilt → level input sequence and its expected angles and four motor commands from the Mac.
- [x] Build the C flight controller for the STM32 and run the same sequence from RAM with motor pins inactive. Check values against the Mac within recorded tolerances.
- [x] Display pass/fail on PC13, then print angles, motor commands and loop timing over USB serial.
- [x] Exercise the bench-only arm/disarm, high-throttle, missing/stale/invalid IMU, late update, radio-loss and restart cases on the Mac and chip. All eight checks passed. [Measured tilt results](../../software/stm32/usb-bench/README.md)
- [ ] Define in-flight fault responses with the team and integrate the arming interlock with the real inputs and motor output path.
- [x] Record a [candidate pin table](pin-plan.md), MCU peripheral assignments, mixer motor corners and proposed storage interface.
- [ ] Verify breakout/header routing and shared nets on the actual board, mounted sensor axes, final radio choice and outstanding parts.

## Barometer

- [x] Exercised the USB barometer command on PB8 (SCL) and PB9 (SDA). The powered module gave no acknowledgement at either address. Both pins read low under the weak pull-down check. The side headers were pushed in without solder.
- [x] The BMP388 answered at 0x76 with three pressure/temperature readings of 1000.95–1000.99 hPa and 21.63 °C on 10 November 2026.
- [ ] Verify permanent header connections and repeat stationary sampling. The return-to-desk run included two no-ack replies with B8 high and B9 low during the weak pull-down check.

## USB and IMU

- [ ] For the Qwiic bench test, read WHO_AM_I over I²C using the [IMU ID command](../../software/stm32/usb-bench/README.md). Expect 0x6C.

- [ ] Check the SparkFun SPI jumper and six basic connections with power off. Read 0x6C from WHO_AM_I after power-up.
- [ ] Record still, tilted and rotating readings. Check gyro offsets, acceleration near 1 g at rest, units, signs and sample intervals.
- [ ] Feed real readings to the existing C estimator. Watch roll and pitch follow the board, and the blue LED respond to tilt.
- [ ] Check that calculated motor corrections oppose each disturbance and stale data triggers the intended response. Keep motor outputs disabled.

## Assembled aircraft, propellers off

- [ ] Inspect frame, mounts, battery restraint, propulsion wiring, polarity and insulation. Measure both supply rails before attaching dependent devices.
- [ ] Calibrate battery sensing. Save the original firmware and recovery instructions. Cold-start the flight build from battery power and verify it stays disarmed.
- [ ] Validate the radio channels, arm switch, range and link-loss response. Verify recovery requires deliberate pilot action.
- [ ] Read a bench log back from microSD. Test missing media, buzzer, barometer, GPS and compass individually.
- [ ] Confirm ESC protocol and waveforms. Run each motor separately and record its corner and spin direction.
- [ ] Repeat control-direction, timing, deliberate disarm, reset and signal-loss checks on the assembled aircraft.

### Individual hardware checks to perform when wired

These checks require the physical component and an appropriate reader, meter or output driver. Record the observed result alongside each test. The current USB firmware implements the barometer and motion-sensor identity commands.

| Component | Bench check and pass condition |
|---|---|
| Foxeer GPS | With a clear view of the sky, record a valid position fix and advancing timestamps. Compare the location with the known site. |
| Compass | Record all three magnetic axes while rotating the module away from motors and metal. The readings should change with rotation and return near their starting values in the original orientation. |
| Receiver and radio | With motor outputs disconnected, record neutral sticks, channel endpoints and arm switch. Switch off the transmitter and verify the receiver reports link loss and the controller disarms. |
| External microSD | Write a known payload to a new test file, close it, remount and compare every byte. Repeat with the card removed and check that logging reports the failure. |
| Power distribution, regulator and battery sensor | Compare board voltage and current telemetry with independent meter readings at known bench loads. Check the supply rails against their specified voltages before attaching the flight electronics. |
| Buzzer and its driver | With the driver wired, command off → on → off. Confirm silence, an audible tone and silence in that order. |
| Four ESCs and motors | Remove propellers. Check signal shape and disarmed output before energising the motors. With verified power and disarm control, test each corner separately and record its position, rotation and stop response. |

## University test facility and flight

- [ ] Record guarded propulsion measurements for the actual propeller and battery. Verify electrical limits, temperature and thrust margin at measured take-off mass.
- [ ] Validate controller response on the rig. Agree the pilot, site, hover target, operating limits and abort criteria.
- [ ] Complete a short manual hover, land, disarm and inspect. Review the log for oscillation, saturation, power sag and timing overruns before the next flight.
