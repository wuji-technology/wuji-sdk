cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(wuji_sdk_c_hand2_opposition LANGUAGES C)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(WUJI_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../include" CACHE PATH
        "Directory containing wuji_sdk.h")
    set(WUJI_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/libwuji_sdk_c.so" CACHE FILEPATH
        "Path to libwuji_sdk_c.so")
    find_package(Threads REQUIRED)
endif()

if(NOT EXISTS "${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h")
    message(FATAL_ERROR "wuji_sdk.h not found: ${WUJI_SDK_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${WUJI_SDK_LIB}")
    message(FATAL_ERROR "libwuji_sdk_c.so not found: ${WUJI_SDK_LIB}")
endif()

get_filename_component(_wuji_sdk_lib_dir "${WUJI_SDK_LIB}" DIRECTORY)
add_executable(6_opposition opposition.c)
target_include_directories(6_opposition PRIVATE "${WUJI_SDK_INCLUDE_DIR}")
target_link_libraries(6_opposition PRIVATE "${WUJI_SDK_LIB}" m Threads::Threads)
target_compile_options(6_opposition PRIVATE -Wall -Wextra -O2)
set_target_properties(6_opposition PROPERTIES
    BUILD_RPATH "${_wuji_sdk_lib_dir}"
    INSTALL_RPATH "\$ORIGIN/../../../../lib"
)

add_custom_command(TARGET 6_opposition POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/data"
            "$<TARGET_FILE_DIR:6_opposition>/data"
)

install(TARGETS 6_opposition RUNTIME DESTINATION bin)
install(DIRECTORY data/ DESTINATION bin/data)
