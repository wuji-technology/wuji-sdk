cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(wuji_sdk_c_hand2_opposition LANGUAGES C)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(WUJI_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../include" CACHE PATH
        "Directory containing wuji_sdk.h")
    if(APPLE)
        set(_wuji_sdk_lib_name "libwuji_sdk_c.dylib")
    else()
        set(_wuji_sdk_lib_name "libwuji_sdk_c.so")
    endif()
    set(WUJI_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/${_wuji_sdk_lib_name}" CACHE FILEPATH
        "Path to ${_wuji_sdk_lib_name}")
    find_package(Threads REQUIRED)
endif()

if(NOT EXISTS "${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
    message(FATAL_ERROR "wuji_sdk.h not found: ${WUJI_SDK_INCLUDE_DIR}")
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
add_executable(6_opposition opposition.c)
target_include_directories(6_opposition PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
target_link_libraries(6_opposition PRIVATE "${WUJI_SDK_LIB}" m Threads::Threads)
target_compile_options(6_opposition PRIVATE -Wall -Wextra -O2)
set_target_properties(6_opposition PROPERTIES
    BUILD_RPATH "${_wuji_sdk_lib_dir}"
    INSTALL_RPATH "${_wuji_sdk_install_rpath}"
)

add_custom_command(TARGET 6_opposition POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/data"
            "$<TARGET_FILE_DIR:6_opposition>/data"
)

install(TARGETS 6_opposition RUNTIME DESTINATION bin)
install(DIRECTORY data/ DESTINATION bin/data)
