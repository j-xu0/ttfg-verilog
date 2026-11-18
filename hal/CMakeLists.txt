cmake_minimum_required(VERSION 3.16)
project(tinystep C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# build/compile_commands.json, symlinked to hal/compile_commands.json below.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Platform file to link: platform/tinystep_platform_<name>.c
set(TINYSTEP_PLATFORM "host" CACHE STRING "Platform target")

add_library(tinystep_warnings INTERFACE)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tinystep_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

# Core: platform-free apart from the seam. The assert header is listed so
# IDEs and the target graph see it; it produces no compile_commands entry.
add_library(tinystep STATIC
  src/tinystep.c
  src/tinystep_assert.h
)
target_include_directories(tinystep PUBLIC inc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(tinystep PRIVATE tinystep_warnings)

# Exactly one implementation, so an incomplete port fails to link.
set(TINYSTEP_PLATFORM_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/platform/tinystep_platform_${TINYSTEP_PLATFORM}.c)
if(NOT EXISTS ${TINYSTEP_PLATFORM_SRC})
  message(FATAL_ERROR
    "Unknown TINYSTEP_PLATFORM '${TINYSTEP_PLATFORM}': no ${TINYSTEP_PLATFORM_SRC}")
endif()

add_library(tinystep_platform STATIC ${TINYSTEP_PLATFORM_SRC})
target_include_directories(tinystep_platform PUBLIC inc)
target_link_libraries(tinystep_platform PRIVATE tinystep_warnings)

enable_testing()

add_executable(test_config test/test_config.c)
target_link_libraries(test_config PRIVATE tinystep tinystep_platform tinystep_warnings)
target_include_directories(test_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/platform)
add_test(NAME config COMMAND test_config)

add_executable(test_move_cmd test/test_move_cmd.c)
target_link_libraries(test_move_cmd PRIVATE tinystep tinystep_platform tinystep_warnings)
target_include_directories(test_move_cmd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/platform)
add_test(NAME move_cmd COMMAND test_move_cmd)

add_executable(bringup_example examples/bringup.c)
target_link_libraries(bringup_example PRIVATE tinystep tinystep_platform tinystep_warnings)
target_include_directories(bringup_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/platform)

add_custom_target(compile_commands ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          ${CMAKE_BINARY_DIR}/compile_commands.json
          ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
  VERBATIM
)
