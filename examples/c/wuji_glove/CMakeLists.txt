cmake_minimum_required(VERSION 3.16)
project(wuji_sdk_c_example LANGUAGES C)

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
# - BUILD_RPATH: when run from the build tree, use the absolute directory of
#   WUJI_SDK_LIB (works for both extracted-tarball and in-repo layouts).
# - INSTALL_RPATH: $ORIGIN-relative fallback for an installed binary; ld.so
#   resolves $ORIGIN to the executable's own directory.
get_filename_component(_wuji_sdk_lib_dir "${WUJI_SDK_LIB}" DIRECTORY)

foreach(demo 0_subscribe_callback 1_emf_poses_rate_divider 2_set_hand_model_path 3_user_management 4_user_calibration 5_tactile_calibration 6_tactile_contact_view 7_tactile_residual_view)
    add_executable(${demo} ${demo}.c)
    target_include_directories(${demo} PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
    target_link_libraries(${demo} PRIVATE "${WUJI_SDK_LIB}")
    target_compile_options(${demo} PRIVATE -Wall -Wextra -O2)
    set_target_properties(${demo} PROPERTIES
        BUILD_RPATH   "${_wuji_sdk_lib_dir}"
        INSTALL_RPATH "\$ORIGIN/../../../../lib"
    )
endforeach()

message(STATUS "WUJI_SDK_INCLUDE_DIR = ${WUJI_SDK_INCLUDE_DIR}")
message(STATUS "WUJI_SDK_LIB         = ${WUJI_SDK_LIB}")
