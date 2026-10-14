cmake_minimum_required(VERSION 3.20)

project(drone_flight_controller C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(drone_sim
        src/main.c
        src/control/pid.c
        src/control/flight_controller.c
        src/estimation/attitude.c
        src/simulation/simulated_imu.c
        src/mixer/mixer.c
        include/imu.h
        src/imu.c
)

target_include_directories(drone_sim PRIVATE include)

target_link_libraries(drone_sim m)

enable_testing()

add_executable(test_pid
        tests/test_pid.c
        src/control/pid.c
)

target_include_directories(test_pid PRIVATE include)
target_link_libraries(test_pid m)

add_test(NAME pid_tests COMMAND test_pid)


add_executable(test_simulation
        tests/test_simulation.c
        src/simulation/simulated_imu.c
        src/imu.c
        src/mixer/mixer.c
)

target_include_directories(test_simulation PRIVATE include)
target_link_libraries(test_simulation m)

add_test(NAME simulation_tests COMMAND test_simulation)

add_executable(test_attitude
        tests/test_attitude.c
        src/estimation/attitude.c
)

target_include_directories(test_attitude PRIVATE include)
target_link_libraries(test_attitude m)

add_test(NAME attitude_tests COMMAND test_attitude)

add_executable(test_imu
        tests/test_imu.c
        src/imu.c
)

target_include_directories(test_imu PRIVATE include)
target_link_libraries(test_imu m)

add_test(NAME imu_tests COMMAND test_imu)

add_executable(test_closed_loop
        tests/test_closed_loop.c
        src/control/pid.c
        src/estimation/attitude.c
        src/imu.c
        src/simulation/simulated_imu.c
        src/mixer/mixer.c
)

target_include_directories(test_closed_loop PRIVATE include)
target_link_libraries(test_closed_loop m)

add_test(NAME closed_loop_tests COMMAND test_closed_loop)

add_executable(test_mixer
        tests/test_mixer.c
        src/mixer/mixer.c
)

target_include_directories(test_mixer PRIVATE include)
target_link_libraries(test_mixer m)

add_test(NAME mixer_tests COMMAND test_mixer)

add_executable(test_flight_controller
        tests/test_flight_controller.c
        src/control/flight_controller.c
        src/control/pid.c
        src/estimation/attitude.c
        src/imu.c
        src/mixer/mixer.c
)

target_include_directories(
        test_flight_controller
        PRIVATE
        include
)

target_link_libraries(
        test_flight_controller
        m
)

add_test(
        NAME flight_controller_tests
        COMMAND test_flight_controller
)