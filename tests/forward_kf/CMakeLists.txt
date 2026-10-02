cmake_minimum_required(VERSION 3.22)
project(forward_kf_checks LANGUAGES C CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(DSP_ROOT "${REPO_ROOT}/third_party/CMSIS-DSP")

set(KF_SOURCES
    ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_forward_kf.cpp
    ${REPO_ROOT}/PYRo/Algorithm/KF/kalman_filter.cpp
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_init_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_add_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_sub_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_trans_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_inverse_f32.c
)
add_executable(forward_kf_test forward_kf_test.cpp ${KF_SOURCES}
    ${REPO_ROOT}/PYRo/Algorithm/EKF/QuaternionEKF.cpp)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(CONTROL_TEST_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/control-generated")
add_custom_command(
    OUTPUT ${CONTROL_TEST_GENERATED}/chassis_types.inc ${CONTROL_TEST_GENERATED}/chassis_functions.inc
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/generate_chassis_control_test.py ${CONTROL_TEST_GENERATED}
    DEPENDS generate_chassis_control_test.py
        ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_chassis.h
        ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_chassis.cpp
        ${REPO_ROOT}/Robot/infantry1/Chassis/fsm/pyro_wl_balance_state.cpp
        ${REPO_ROOT}/Robot/infantry1/Chassis/fsm/pyro_wl_spin_state.cpp
    VERBATIM
)
add_executable(chassis_control_test chassis_control_test.cpp ${KF_SOURCES}
    ${REPO_ROOT}/PYRo/Algorithm/Common/pyro_algo_common.cpp
    ${CONTROL_TEST_GENERATED}/chassis_types.inc ${CONTROL_TEST_GENERATED}/chassis_functions.inc)
target_include_directories(chassis_control_test PRIVATE ${CONTROL_TEST_GENERATED})

foreach(TEST_TARGET forward_kf_test chassis_control_test)
    target_include_directories(${TEST_TARGET} PRIVATE
        ${REPO_ROOT}/Robot/infantry1/Chassis
        ${REPO_ROOT}/PYRo/Algorithm/Common
        ${REPO_ROOT}/PYRo/Algorithm/KF
        ${REPO_ROOT}/PYRo/Algorithm/EKF
        ${DSP_ROOT}/Include
        ${DSP_ROOT}/PrivateInclude
    )
    # CMSIS's portable host configuration; keep dimension checks enabled in tests.
    target_compile_definitions(${TEST_TARGET} PRIVATE __GNUC_PYTHON__ ARM_MATH_MATRIX_CHECK)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${TEST_TARGET} PRIVATE -ffp-contract=off -fno-strict-aliasing)
    endif()
endforeach()
add_test(NAME forward_kf_checks COMMAND forward_kf_test)
add_test(NAME chassis_control_chain COMMAND chassis_control_test)
