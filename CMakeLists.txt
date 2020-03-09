cmake_minimum_required(VERSION 3.0)
project(rscapture)

#set(LIBREALSENSE_INCLUDE "librealsense/include")
#set(LIBREALSENSE_BUILD "./librealsense/build")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2 -lstdc++")

add_executable(rscapture src/main.cpp)

include_directories(rscapture ./ ./odometry/recorder ./odometry/lib/json/include ./librealsense/include)

find_library(RECORDER_LIB recorder HINTS ./odometry/target/recorder)
find_library(LOGURU_LIB loguru HINTS ./odometry/target)
find_library(REALSENSE_LIB realsense2 HINTS ./librealsense/build)
find_library(SDL2_LIB SDL2)
find_library(M_LIB m)
target_link_libraries(rscapture PUBLIC ${REALSENSE_LIB} ${SDL2_LIB} ${M_LIB} ${RECORDER_LIB} ${LOGURU_LIB})
