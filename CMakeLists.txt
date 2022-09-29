cmake_minimum_required(VERSION 3.23)
project(camera-calibration)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(camera-calibration main.cpp)
target_link_libraries(camera-calibration ${OpenCV_LIBS})