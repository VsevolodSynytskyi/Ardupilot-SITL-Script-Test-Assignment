cmake_minimum_required(VERSION 3.20)
project(altitude_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# MAVSDK is built project-locally by scripts/build_mavsdk.sh
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavsdk-install")
find_package(MAVSDK REQUIRED)

# Controller logic without MAVSDK: unit-testable in isolation
add_library(altctl_core STATIC
  src/config.cpp
)
target_include_directories(altctl_core PUBLIC include)
target_compile_options(altctl_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(altitude_control
  src/main.cpp
  src/drone_interface.cpp
  src/diagnostics.cpp
)
target_link_libraries(altitude_control PRIVATE altctl_core MAVSDK::mavsdk)
target_compile_options(altitude_control PRIVATE -Wall -Wextra)
# Run-time lookup of the project-local libmavsdk.dylib
set_target_properties(altitude_control PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavsdk-install/lib")

# Unit tests (GoogleTest via FetchContent) are added in Stage 7 together with the PID code.
