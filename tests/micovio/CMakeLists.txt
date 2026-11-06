cmake_minimum_required(VERSION 3.22)
project(micovio_checks LANGUAGES C CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(VIO_ROOT "${REPO_ROOT}/PYRo/Component/VIO")
add_executable(micovio_test
    test_micovio.cpp
    ${VIO_ROOT}/Protocol/pyro_micovio_protocol.cpp
    ${VIO_ROOT}/pyro_micovio_drv.cpp
    ${REPO_ROOT}/PYRo/Algorithm/CRC/pyro_crc.cpp
    ${REPO_ROOT}/PYRo/Component/Referee/fifo.c)
target_include_directories(micovio_test PRIVATE
    stubs ${VIO_ROOT} ${REPO_ROOT}/PYRo/Core/Def
    ${REPO_ROOT}/PYRo/Algorithm/CRC
    ${REPO_ROOT}/PYRo/Component/Referee)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(micovio_test PRIVATE -Wall -Wextra -funsafe-math-optimizations)
endif()
add_test(NAME micovio_checks COMMAND micovio_test)
