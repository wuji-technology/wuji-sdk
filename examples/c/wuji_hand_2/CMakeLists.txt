cmake_minimum_required(VERSION 3.16)
project(wuji_sdk_c_hand2_example LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Point CMake at the extracted SDK tarball:
#   cmake -S . -B build \
#     -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
#     -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so   # macOS: .dylib
set(WUJI_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include" CACHE PATH
    "Directory containing wuji_sdk.h")
if(APPLE)
    set(_wuji_sdk_lib_name "libwuji_sdk_c.dylib")
else()
    set(_wuji_sdk_lib_name "libwuji_sdk_c.so")
endif()
set(WUJI_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/${_wuji_sdk_lib_name}" CACHE FILEPATH
    "Path to ${_wuji_sdk_lib_name}")

if(NOT EXISTS "${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
    message(FATAL_ERROR "wuji_sdk.h not found: ${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
endif()
if(NOT EXISTS "${WUJI_SDK_LIB}")
    message(FATAL_ERROR "${_wuji_sdk_lib_name} not found: ${WUJI_SDK_LIB}")
endif()

# RPATH so built binaries find the SDK library without LD_LIBRARY_PATH at
# build time, and without it or DYLD_LIBRARY_PATH after install.
# - BUILD_RPATH: absolute directory of WUJI_SDK_LIB (works for both the
#   extracted-tarball and the in-repo layout).
# - INSTALL_RPATH: loader-relative fallback for an installed binary; the loader
#   resolves $ORIGIN (ELF) / @loader_path (Mach-O) to the binary's own dir.
get_filename_component(_wuji_sdk_lib_dir "${WUJI_SDK_LIB}" DIRECTORY)
if(APPLE)
    set(_wuji_sdk_install_rpath "@loader_path/../../../../lib")
else()
    set(_wuji_sdk_install_rpath "\$ORIGIN/../../../../lib")
endif()
find_package(Threads REQUIRED)

foreach(demo
        0_subscribe_joint_states
        1_control_motion
        2_hand_info
        3_fingertip
        4_set_stream_rate
        5_export_flash_logs
        7_mit_sweep)
    add_executable(${demo} ${demo}.c)
    target_include_directories(${demo} PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
    target_link_libraries(${demo} PRIVATE "${WUJI_SDK_LIB}")
    target_compile_options(${demo} PRIVATE -Wall -Wextra -O2)
    set_target_properties(${demo} PROPERTIES
        BUILD_RPATH   "${_wuji_sdk_lib_dir}"
        INSTALL_RPATH "${_wuji_sdk_install_rpath}"
    )
endforeach()

add_subdirectory(6_opposition)

target_link_libraries(1_control_motion PRIVATE m Threads::Threads)
target_link_libraries(7_mit_sweep PRIVATE m Threads::Threads)
target_link_libraries(3_fingertip PRIVATE m Threads::Threads)  # sin()/cos()/sqrt(), pthread

message(STATUS "WUJI_SDK_INCLUDE_DIR = ${WUJI_SDK_INCLUDE_DIR}")
message(STATUS "WUJI_SDK_LIB         = ${WUJI_SDK_LIB}")
