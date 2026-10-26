cmake_minimum_required(VERSION 3.16)
project(wuji_sdk_c_hand_example LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Point CMake at the extracted SDK tarball:
#   cmake -S . -B build \
#     -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
#     -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so   # macOS: .dylib
#
# NOTE: Wuji Hand support is validated on the Linux x86_64 / aarch64 (gnu)
# tarballs. The driver is built into libwuji_sdk_c.so — nothing else to deploy.
# The examples exit cleanly when no hand is connected.
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

foreach(demo 0_subscribe 1_publish 2_grasp_loop 3_tactile_status)
    add_executable(${demo} ${demo}.c)
    target_include_directories(${demo} PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
    target_link_libraries(${demo} PRIVATE "${WUJI_SDK_LIB}" m)
    target_compile_options(${demo} PRIVATE -Wall -Wextra -O2)
    set_target_properties(${demo} PROPERTIES
        BUILD_RPATH   "${_wuji_sdk_lib_dir}"
        INSTALL_RPATH "${_wuji_sdk_install_rpath}"
    )
endforeach()

message(STATUS "WUJI_SDK_INCLUDE_DIR = ${WUJI_SDK_INCLUDE_DIR}")
message(STATUS "WUJI_SDK_LIB         = ${WUJI_SDK_LIB}")
