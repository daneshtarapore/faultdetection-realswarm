# Project file for tracking server.
cmake_minimum_required(VERSION 2.8)
project(machine_vision C CXX ) #C for finding pthread lib (http://stackoverflow.com/a/15193877)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Make CMake aware of our FindCVB.cmake module
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "$ENV{CVB}/cmake")
find_package(CVB REQUIRED)

# check for pthread lib
find_package(Threads REQUIRED)

add_executable(machine_vision
        machine_vision.cpp)

target_link_libraries(machine_vision
        CVB::CVCDriver
        CVB::CVCUtilities
        CVB::CVGenApi
	pthread)

include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)
include_directories(${OPENCV_INCLUDE_DIRS})
target_link_libraries(machine_vision ${OPENCV_LDFLAGS})
