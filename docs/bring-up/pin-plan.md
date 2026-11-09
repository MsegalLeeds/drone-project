# Candidate STM32 pin assignments

Paper design for FK743M3-VGT6 V1.1. The listed peripheral functions were checked against ST's STM32H743VGT package data. The side headers use labels such as A5 for PA5. This USB test exercised PA11/PA12 and PC13 only. Before soldering, check the exact board schematic or continuity for each remaining header and for shared connections to the camera, LCD, onboard flash and card socket. Check interface voltages and alternate-function settings when implementing each driver. [ST chip pin data](https://github.com/STMicroelectronics/STM32_open_pin_data/blob/master/mcu/STM32H743VGTx.xml)

| Connection | Proposed STM32 signals | Remaining checks |
|---|---|---|
| SparkFun LSM6DSO | PA5 clock, PA7 MOSI to SDI, PA6 MISO from SDO, PA4 GPIO chip select, PB0 input from INT1 | Confirm 3V3, GND and the breakout's address jumper for SPI. This is the first harness to check. |
| Waveshare BMP388 | PB8 I²C1 clock, PB9 data | Confirm address and pull-up voltage. |
| Foxeer GPS and compass | PD5 USART2 TX to GPS RX, PD6 RX from GPS TX. PB10 I²C2 clock, PB11 data for the IST8310 | Confirm connector order, voltage levels and pull-ups. |
| Proposed RP1 receiver | PD8 USART3 TX to receiver RX, PD9 RX from receiver TX | Confirm radio selection, use of CRSF and clearance from the LCD connector. |
| Listed external microSD breakout | PB13 SPI2 clock, PB15 MOSI to card DI, PB14 MISO from DO, PB12 GPIO chip select | Use this interface for the first design per the parts list. Check clearance from board peripherals. Leave the onboard slot unused. |
| Four ESC signals | PE9 TIM1 CH1 front left, PE11 CH2 front right, PE13 CH3 rear right, PE14 CH4 rear left | Proposed motor positions follow mixer.h. Front pair turn CCW/CW viewed from above; rear right/left turn CCW/CW. Verify each with props off before flight. |
| Battery sensing | PC4 ADC1 input 4 current, PC5 ADC1 input 8 voltage | Verify scaling and maximum signal voltage before connection. |
| VIFLY buzzer control | PE3 GPIO to the transistor driver in the wiring diagram | Confirm package orientation and buzzer input. |
| Reserved | PA11/PA12 USB, PA13/PA14 SWD debugging, PC13 blue LED, BOOT and RST | Keep available for loading, debugging and recovery. |

Mark the camera-connector end or another fixed frame edge as aircraft front when choosing mounts; document the installed sensor-axis rotation. Radio selection, battery strap, microSD card, transmitter cells and current limiter still need a physical inventory. The full power and bench wiring is in the [existing diagram](../../hardware/wiring/drone-wiring-diagram.pdf).
