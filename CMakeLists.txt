cmake_minimum_required(VERSION 3.17.5)
project(kalman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(INCLUDE_DIR /Users/administrator/Documents/Projects/kalman_filter_cpp/include/)
include_directories(${INCLUDE_DIR})

# The below commands build sandbox
# set(SOURCE_FILES sandbox.cpp Kalman.cpp)
# add_executable(sandbox ${SOURCE_FILES})

# The below commands build main
set(SOURCE_FILES main.cpp Kalman.cpp BasicKalman.cpp ExtendedKalman.cpp)
add_executable(main ${SOURCE_FILES})