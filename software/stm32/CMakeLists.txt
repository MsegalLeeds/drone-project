# These targets use the board's Arm toolchain alongside the native simulator.
add_custom_target(stm32_blink
    COMMAND bash "Load blink.command" --build-only
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/blue-led"
    SOURCES blue-led/double-blink.c blue-led/blink-ram.ld
    VERBATIM)

add_custom_target(stm32_bench
    COMMAND bash build.sh
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/usb-bench"
    SOURCES usb-bench/board.c usb-bench/barometer.c usb-bench/usb.c
            usb-bench/replay.c usb-bench/reference.c usb-bench/faults.c
            usb-bench/replay.h usb-bench/tusb_config.h usb-bench/ram.ld
    VERBATIM)
