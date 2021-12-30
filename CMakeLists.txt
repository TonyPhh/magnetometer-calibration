cmake_minimum_required(VERSION 3.20)
project(code)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

#set the EIGEN3_INCLUDE_DIR to your path where you install your Eigen
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(magnetometer_calibrate magnetometer_calibrate.cpp)