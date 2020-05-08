cmake_minimum_required(VERSION 3.0)
project(rscapture)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2 -lstdc++")

add_executable(rscapture src/main.cpp)

option (JSON_BuildTests "Build nlohmann_json unit tests" ON)
set(JSON_BuildTests OFF)
add_subdirectory("json")
include_directories(rscapture ./ ./jsonl-recorder ./librealsense/include)
add_subdirectory("jsonl-recorder")
find_library(REALSENSE_LIB realsense2 HINTS ./librealsense/build)
find_library(SDL2_LIB SDL2)
find_library(M_LIB m)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
target_link_libraries(rscapture PRIVATE
  ${REALSENSE_LIB}
  ${SDL2_LIB}
  ${M_LIB}
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
  recorder
  Threads::Threads)
