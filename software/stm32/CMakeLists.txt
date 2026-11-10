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
    usb-bench/board.c usb-bench/bench_i2c.c usb-bench/barometer.c usb-bench/usb.c
    usb-bench/replay.c usb-bench/faults.c usb-bench/reference.c)
target_include_directories(stm32_bench_sources PRIVATE ${firmware_includes})
target_compile_definitions(stm32_bench_sources PRIVATE STM32H743xx NDEBUG)
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(stm32_bench_sources PRIVATE
        --target=arm-none-eabi -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16
        -mfloat-abi=hard -ffreestanding -nostdlibinc)
endif()

# Bench checks that run on the Mac. The same replay and sensor C code runs on the board.
set(bench_dir "${CMAKE_CURRENT_SOURCE_DIR}/usb-bench")
set(controller_dir "${CMAKE_CURRENT_SOURCE_DIR}/../flight-controller")
add_executable(test_bench_replay
    usb-bench/reference.c usb-bench/replay.c usb-bench/faults.c
    "${controller_dir}/src/control/flight_controller.c" "${controller_dir}/src/control/pid.c"
    "${controller_dir}/src/estimation/attitude.c" "${controller_dir}/src/mixer/mixer.c"
    "${controller_dir}/src/imu.c")
target_include_directories(test_bench_replay PRIVATE "${controller_dir}/include")
target_link_libraries(test_bench_replay PRIVATE m)
add_test(NAME bench_replay_and_faults COMMAND test_bench_replay)
add_test(NAME bench_sensor_registers COMMAND bash "${bench_dir}/build.sh" --test-sensors)
add_test(NAME bench_usb_reports COMMAND python3 -B "${bench_dir}/tests/test_usb_reports.py")

# Explicit opt-in keeps unattended and simulation-only runs independent of USB.
set(STM32_LIVE_COMPONENTS "" CACHE STRING "Connected bench components to test: board;barometer;imu")
foreach(component IN LISTS STM32_LIVE_COMPONENTS)
    if(component STREQUAL "board")
        set(args "")
    elseif(component STREQUAL "barometer")
        set(args --barometer)
    elseif(component STREQUAL "imu")
        set(args --imu-id)
    else()
        message(FATAL_ERROR "Unknown STM32_LIVE_COMPONENTS entry: ${component}")
    endif()
    add_test(NAME live_${component} COMMAND python3 "${bench_dir}/read_board.py" ${args})
    set_tests_properties(live_${component} PROPERTIES LABELS hardware RESOURCE_LOCK stm32_usb TIMEOUT 30)
endforeach()
