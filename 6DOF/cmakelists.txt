# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and C++ standard
project(NUSTARS_ADS)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directory for Eigen (relative path)
include_directories(${CMAKE_SOURCE_DIR}/eigen)

# Define the main program executable and specify source files
add_executable(my_program 
  AeroData.cpp 
  calcDynamics.cpp 
  constants.cpp  
  controls.cpp 
  sensing.cpp 
  simulator.cpp 
  main.cpp 
  q.cpp
)

# Add the test directory if it exists
message(STATUS "Source directory: ${CMAKE_SOURCE_DIR}") 
# message("hi")
# if(EXISTS “${CMAKE_SOURCE_DIR}/test/CMakeLists.txt”)
add_subdirectory(test)
enable_testing()
# endif()
