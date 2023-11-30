cmake_minimum_required(VERSION 3.12)
project(unit)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Subdirectories
add_subdirectory(src)

