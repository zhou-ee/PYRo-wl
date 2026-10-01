cmake_minimum_required(VERSION 3.22)
project(forward_kf_checks LANGUAGES C CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(DSP_ROOT "${REPO_ROOT}/third_party/CMSIS-DSP")

add_executable(forward_kf_test
    forward_kf_test.cpp
    ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_forward_kf.cpp
    ${REPO_ROOT}/PYRo/Algorithm/KF/kalman_filter.cpp
    ${REPO_ROOT}/PYRo/Algorithm/EKF/QuaternionEKF.cpp
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_init_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_add_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_sub_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_trans_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_inverse_f32.c
)
target_include_directories(forward_kf_test PRIVATE
    ${REPO_ROOT}/Robot/infantry1/Chassis
    ${REPO_ROOT}/PYRo/Algorithm/Common
    ${REPO_ROOT}/PYRo/Algorithm/KF
    ${REPO_ROOT}/PYRo/Algorithm/EKF
    ${DSP_ROOT}/Include
    ${DSP_ROOT}/PrivateInclude
)
# CMSIS's portable host configuration; keep dimension checks enabled in tests.
target_compile_definitions(forward_kf_test PRIVATE __GNUC_PYTHON__ ARM_MATH_MATRIX_CHECK)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(forward_kf_test PRIVATE -ffp-contract=off -fno-strict-aliasing)
endif()
add_test(NAME forward_kf_checks COMMAND forward_kf_test)
