# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and C++ standard
project(NUSTARS_ADS)
set(CMAKE_CXX_STANDARD 14)

# Add include directory for Eigen
include_directories(${CMAKE_SOURCE_DIR}/eigen)


# Define the executable and specify source files
add_executable(my_program main.cpp q.cpp)