cmake_minimum_required(VERSION 3.20)

project(motor_control
    VERSION 0.1.0
    LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(MC_BUILD_TESTS "Build unit tests" ON)

add_library(motor_control STATIC
    src/api/mc_api.c
    src/api/mc_identify.c
    src/api/mc_cfg.c
    src/api/mc_adc_calibrate.c
    src/api/mc_port_adc_map.c
    src/api/mc_port_adc_ref.c
    src/api/mc_diag.c
    src/math/mc_math.c
    src/control/mc_transform.c
    src/control/mc_control_foc.c
    src/control/mc_control_pi.c
    src/control/mc_control_svpwm.c
    src/reconstruct/mc_reconstruct_1shunt.c
    src/reconstruct/mc_reconstruct_2shunt.c
    src/reconstruct/mc_reconstruct_3shunt.c
    src/estimator/mc_sensor_encoder.c
    src/estimator/mc_sensor_hall.c
    src/estimator/mc_sensor_resolver.c
    src/estimator/mc_sensor_sensorless.c
    src/drive/mc_drive_bldc.c
    src/drive/mc_drive_bldc_sensorless.c
    src/drive/mc_drive_pmsm.c)

target_include_directories(motor_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_definitions(motor_control
    PUBLIC
        MC_CFG_BUILD_M1=1)

if(MSVC)
    target_compile_options(motor_control PRIVATE /W4)
else()
    target_compile_options(motor_control PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(motor_control PRIVATE m)
endif()

if(MC_BUILD_TESTS)
    include(CTest)
    enable_testing()

    add_executable(test_mc_api
        tests/unit/test_mc_api.c
        tests/unit/test_mc_math.c
        tests/unit/test_mc_transform.c
        tests/unit/test_mc_control_foc.c
        tests/unit/test_mc_control_pi.c
        tests/unit/test_mc_control_svpwm.c
        tests/unit/test_mc_port_adc_map.c
        tests/unit/test_mc_port_adc_ref.c
        tests/unit/test_mc_reconstruct_1shunt.c
        tests/unit/test_mc_reconstruct_2shunt.c
        tests/unit/test_mc_reconstruct_3shunt.c
        tests/unit/test_mc_sensor_encoder.c
        tests/unit/test_mc_sensor_hall.c
        tests/unit/test_mc_sensor_resolver.c
        tests/unit/test_mc_sensor_sensorless.c
        tests/unit/test_mc_drive_bldc.c
        tests/unit/test_mc_drive_bldc_sensorless.c
        tests/unit/test_mc_drive_pmsm.c
        tests/unit/test_mc_cfg.c
        tests/unit/test_mc_diag.c
        tests/unit/test_mc_adc_calibrate.c
        tests/unit/test_mc_identify.c
        tests/support/unity/unity.c)

    target_include_directories(test_mc_api
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/unity)

    target_link_libraries(test_mc_api PRIVATE motor_control)

    add_test(NAME test_mc_api COMMAND test_mc_api)
endif()
