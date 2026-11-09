#!/bin/bash
set -eu
cd "$(dirname "$0")"
project=${FLIGHT_CONTROLLER_DIR:-"../../flight-controller"}
deps=${BENCH_DEPS:-/private/tmp/drone-usb-step1}
mkdir -p "$deps" build
fetch() {
    local name=$1 repo=$2 tag=$3 sha=$4
    if [[ ! -d "$deps/$name" ]]; then
        git clone --depth 1 --branch "$tag" "https://github.com/$repo.git" "$deps/$name"
    fi
    [[ $(git -C "$deps/$name" rev-parse HEAD) == "$sha" ]] || {
        echo "Unexpected revision of $name in $deps" >&2; exit 1;
    }
}
fetch tinyusb hathach/tinyusb 0.18.0 86ad6e56c1700e85f1c5678607a762cfe3aa2f47
fetch cmsis_device_h7 STMicroelectronics/cmsis-device-h7 v1.10.5 e4d57d6edc39f35aced298732829fc2a5e6a8a71
fetch CMSIS_5 ARM-software/CMSIS_5 5.9.0 2b7495b8535bdcb306dac29b9ded4cfb679d7e5c
fetch BMP3_SensorAPI boschsensortec/BMP3_SensorAPI bmp3_v2.0.6 db4cf8e4140c593b8c3d85f8c6c07335c7ffa9dc
if ! command -v arm-none-eabi-gcc >/dev/null; then
    if [[ ! -x "$deps/arm-pkg/Payload/bin/arm-none-eabi-gcc" ]]; then
        pkg=$(/opt/homebrew/bin/brew --cache --cask gcc-arm-embedded)
        [[ -f "$pkg" ]] || /opt/homebrew/bin/brew fetch --cask gcc-arm-embedded
        pkgutil --expand-full "$pkg" "$deps/arm-pkg"
    fi
    export PATH="$deps/arm-pkg/Payload/bin:$PATH"
fi
flight=("$project/src/control/flight_controller.c" "$project/src/control/pid.c"
       "$project/src/estimation/attitude.c" "$project/src/mixer/mixer.c" "$project/src/imu.c")
shared=(-std=c11 -O2 -ffp-contract=off "-I$project/include")
xcrun clang "${shared[@]}" reference.c replay.c faults.c "${flight[@]}" -lm -o build/mac-reference
build/mac-reference > build/mac.csv
usb="$deps/tinyusb/src"
dwc="$usb/portable/synopsys/dwc2"
arm-none-eabi-gcc "${shared[@]}" -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
    -DNDEBUG -DSTM32H743xx -ffunction-sections -fdata-sections -I. -I"$usb" \
    -I"$deps/BMP3_SensorAPI" "$deps/BMP3_SensorAPI/bmp3.c" -I"$deps/CMSIS_5/CMSIS/Core/Include" -I"$deps/cmsis_device_h7/Include" \
    -Os -nostartfiles --specs=nano.specs --specs=nosys.specs \
    -Wl,-T,ram.ld,--gc-sections -u _printf_float board.c barometer.c usb.c replay.c faults.c "${flight[@]}" \
    "$usb/tusb.c" "$usb/common/tusb_fifo.c" "$usb/device/usbd.c" "$usb/device/usbd_control.c" \
    "$usb/class/cdc/cdc_device.c" "$dwc/dcd_dwc2.c" "$dwc/dwc2_common.c" \
    -lm -o build/tilt.elf
arm-none-eabi-objcopy -O ihex build/tilt.elf build/tilt.hex
arm-none-eabi-size build/tilt.elf
