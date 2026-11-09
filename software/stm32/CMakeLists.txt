# These targets use the board's Arm toolchain alongside the native simulator.
add_custom_target(stm32_blink
    COMMAND bash "Load blink.command" --build-only
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/blue-led"
    SOURCES blue-led/double-blink.c blue-led/blink-ram.ld
    VERBATIM)

add_custom_target(stm32_bench
    COMMAND bash build.sh
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/usb-bench"
    SOURCES usb-bench/ram.ld
    VERBATIM)

# Custom targets do not provide a compiler model to CLion. Parse the firmware
# as Cortex-M7 C while the simulator continues to use the Mac toolchain.
execute_process(
    COMMAND bash build.sh --clion-includes
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/usb-bench"
    OUTPUT_VARIABLE firmware_includes
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
string(REPLACE "\n" ";" firmware_includes "${firmware_includes}")

add_library(stm32_bench_sources OBJECT EXCLUDE_FROM_ALL
    usb-bench/board.c usb-bench/barometer.c usb-bench/usb.c
    usb-bench/replay.c usb-bench/faults.c usb-bench/reference.c)
target_include_directories(stm32_bench_sources PRIVATE ${firmware_includes})
target_compile_definitions(stm32_bench_sources PRIVATE STM32H743xx NDEBUG)
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(stm32_bench_sources PRIVATE
        --target=arm-none-eabi -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16
        -mfloat-abi=hard -ffreestanding -nostdlibinc)
endif()
