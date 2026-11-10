cmake_minimum_required(VERSION 3.16)
project(wuji_sdk_c_recording_example LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Point CMake at the extracted SDK package:
#   cmake -S . -B build \
#     -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
#     -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so   # macOS: .dylib
# Windows: -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.dll.a \
#          -DWUJI_SDK_RUNTIME_DIR=/path/to/sdk/bin
set(WUJI_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include" CACHE PATH
    "Directory containing wuji_sdk.h")

if(WIN32)
    set(_wuji_sdk_lib_name "libwuji_sdk_c.dll.a")
elseif(APPLE)
    set(_wuji_sdk_lib_name "libwuji_sdk_c.dylib")
else()
    set(_wuji_sdk_lib_name "libwuji_sdk_c.so")
endif()

if(WIN32)
    set(WUJI_SDK_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../bin" CACHE PATH
        "Directory holding wuji_sdk_c.dll and its runtime DLLs")
endif()
set(WUJI_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/${_wuji_sdk_lib_name}" CACHE FILEPATH
    "Path to ${_wuji_sdk_lib_name} (Windows: the import library)")

if(NOT EXISTS "${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
    message(FATAL_ERROR "wuji_sdk.h not found: ${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
endif()
if(NOT EXISTS "${WUJI_SDK_LIB}")
    message(FATAL_ERROR "${_wuji_sdk_lib_name} not found: ${WUJI_SDK_LIB}")
endif()
if(WIN32 AND NOT EXISTS "${WUJI_SDK_RUNTIME_DIR}/wuji_sdk_c.dll")
    message(FATAL_ERROR "wuji_sdk_c.dll not found: ${WUJI_SDK_RUNTIME_DIR}")
endif()

if(WIN32)
    add_library(wuji_sdk_c SHARED IMPORTED)
    set_target_properties(wuji_sdk_c PROPERTIES
        IMPORTED_IMPLIB   "${WUJI_SDK_LIB}"
        IMPORTED_LOCATION "${WUJI_SDK_RUNTIME_DIR}/wuji_sdk_c.dll"
    )
else()
    get_filename_component(_wuji_sdk_lib_dir "${WUJI_SDK_LIB}" DIRECTORY)
    if(APPLE)
        set(_wuji_sdk_install_rpath "@loader_path/../../../../lib")
    else()
        set(_wuji_sdk_install_rpath "\$ORIGIN/../../../../lib")
    endif()
endif()

foreach(demo 0_record_topics)
    add_executable(${demo} ${demo}.c)
    target_include_directories(${demo} PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
    if(MSVC)
        target_compile_options(${demo} PRIVATE /W4 /utf-8)
    else()
        target_compile_options(${demo} PRIVATE -Wall -Wextra -O2)
    endif()
    if(WIN32)
        target_link_libraries(${demo} PRIVATE wuji_sdk_c shell32)
        add_custom_command(TARGET ${demo} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
                    "${WUJI_SDK_RUNTIME_DIR}"
                    "$<TARGET_FILE_DIR:${demo}>")
    else()
        target_link_libraries(${demo} PRIVATE "${WUJI_SDK_LIB}" pthread)
        set_target_properties(${demo} PROPERTIES
            BUILD_RPATH   "${_wuji_sdk_lib_dir}"
            INSTALL_RPATH "${_wuji_sdk_install_rpath}")
    endif()
endforeach()

message(STATUS "WUJI_SDK_INCLUDE_DIR = ${WUJI_SDK_INCLUDE_DIR}")
message(STATUS "WUJI_SDK_LIB         = ${WUJI_SDK_LIB}")
if(WIN32)
    message(STATUS "WUJI_SDK_RUNTIME_DIR = ${WUJI_SDK_RUNTIME_DIR}")
endif()
