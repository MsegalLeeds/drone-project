#!/bin/bash
set -eu
cd "$(dirname "$0")"
export PATH="/opt/homebrew/bin:$PATH"
if [[ ! -f build/tilt.hex || ! -f build/mac.csv ]]; then
    ./build.sh
fi
# Read the existing result if the bench firmware is running. Otherwise start
# the saved, verified RAM image after the user puts the board in DFU mode.
if ! ioreg -r -n 'STM32 Tilt Bench' -l -w0 | grep -q '"USB Serial Number"'; then
    echo "Unplug USB, hold BOOT while reconnecting, then release BOOT."
    read -r -p "Press Return once the board is in programming mode. "
    programmer="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"
    "$programmer" -c port=USB1 --skipErase -d build/tilt.hex -v -g 0x24010000
    echo "Click Allow if macOS asks about the STM32 Tilt Bench."
    for ((i=0; i<30; ++i)); do
        if ioreg -r -n 'STM32 Tilt Bench' -l -w0 | grep -q '"IOCalloutDevice"'; then break; fi
        sleep 1
    done
fi
echo "Columns: position, roll (deg), pitch (deg), front-left, front-right, rear-right, rear-left"
/usr/bin/python3 read_board.py
