cmake_minimum_required(VERSION 3.13)  # CMake version check
project(wastar_demo)
set(CMAKE_CXX_STANDARD 17)            # Enable c++17 standard

# set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS "-O3")

set(SOURCE_FILES main.cpp)

add_executable(wastar_demo ${SOURCE_FILES})