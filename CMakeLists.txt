cmake_minimum_required(VERSION 3.17.5)
project(kalman-filter)

list(APPEND CMAKE_PREFIX_PATH "/Users/administrator/Documents/Projects/kalman_filter_cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

set(INCLUDE_DIR /Users/administrator/Documents/Projects/kalman_filter_cpp/include/)
set(LIBRARY_DIR /Users/administrator/Documents/Projects/kalman_filter_cpp/lib/)

#find_package(Eigen REQUIRED PATHS ${INCLUDE_DIR})

file(GLOB SOURCES src/main.cpp src/Kalman.cpp src/BasicKalman.cpp)

add_executable(kalman-filter ${SOURCES})

target_include_directories(kalman-filter PUBLIC ${INCLUDE_DIR})
target_link_libraries(kalman-filter PUBLIC ${LIBRARY_DIR})