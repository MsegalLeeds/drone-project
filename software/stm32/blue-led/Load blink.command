#!/bin/bash
set -eu

cd "$(dirname "$0")"
export PATH="/opt/homebrew/bin:$PATH"
programmer="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT

echo "Building your saved double-blink.c..."
xcrun clang --target=thumbv7em-none-eabi -mcpu=cortex-m7 -mthumb -Os \
    -ffreestanding -fno-unwind-tables -Wall -Wextra \
    -c double-blink.c -o "$build_dir/blink.o"
arm-none-eabi-ld -T blink-ram.ld "$build_dir/blink.o" -o "$build_dir/blink.elf"
arm-none-eabi-objcopy -O ihex -j .image "$build_dir/blink.elf" "$build_dir/blink.hex"
cp "$build_dir/blink.hex" double-blink-ram.hex
if [[ ${1:-} == --build-only ]]; then
    echo "Build complete."
    exit 0
fi

echo "Keep USB plugged in. Hold BOOT, tap and release RST, then release BOOT."
echo "If STM32CubeProgrammer is connected, click Disconnect in its window."
read -r -p "Press Return here to load the program. "
if "$programmer" -c port=USB1 --skipErase -d "$build_dir/blink.hex" -v -g 0x24010000; then
    echo "Started. Watch the blue LED. Press RST to return to the original program."
else
    echo "Loading failed. Try holding BOOT while reconnecting USB and run this loader again." >&2
    exit 1
fi
