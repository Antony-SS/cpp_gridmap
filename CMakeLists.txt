cmake_minimum_required(VERSION 3.20)
project(cplusplus_robotics_refresh VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build a static library from everything in src/
file(GLOB LIB_SOURCES src/*.cpp)
add_library(robotics_lib STATIC ${LIB_SOURCES})
target_include_directories(robotics_lib PUBLIC include)

# Add the scripts (executables)
add_subdirectory(scripts)