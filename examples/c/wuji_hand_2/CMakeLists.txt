cmake_minimum_required(VERSION 3.16)
project(wuji_sdk_c_hand2_example LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Point CMake at the extracted SDK tarball:
#   cmake -S . -B build \
#     -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
#     -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
set(WUJI_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include" CACHE PATH
    "Directory containing wuji_sdk.h")
set(WUJI_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/libwuji_sdk_c.so" CACHE FILEPATH
    "Path to libwuji_sdk_c.so")

if(NOT EXISTS "${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
    message(FATAL_ERROR "wuji_sdk.h not found: ${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
endif()
if(NOT EXISTS "${WUJI_SDK_LIB}")
    message(FATAL_ERROR "libwuji_sdk_c.so not found: ${WUJI_SDK_LIB}")
endif()

# RPATH so built binaries find libwuji_sdk_c.so without LD_LIBRARY_PATH.
get_filename_component(_wuji_sdk_lib_dir "${WUJI_SDK_LIB}" DIRECTORY)

foreach(demo 0_subscribe_joint_states 1_control_motion 2_hand_info)
    add_executable(${demo} ${demo}.c)
    target_include_directories(${demo} PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
    target_link_libraries(${demo} PRIVATE "${WUJI_SDK_LIB}")
    target_compile_options(${demo} PRIVATE -Wall -Wextra -O2)
    set_target_properties(${demo} PROPERTIES
        BUILD_RPATH   "${_wuji_sdk_lib_dir}"
        INSTALL_RPATH "\$ORIGIN/../../../../lib"
    )
endforeach()
target_link_libraries(1_control_motion PRIVATE m)  # sinf()/cosf()

message(STATUS "WUJI_SDK_INCLUDE_DIR = ${WUJI_SDK_INCLUDE_DIR}")
message(STATUS "WUJI_SDK_LIB         = ${WUJI_SDK_LIB}")
