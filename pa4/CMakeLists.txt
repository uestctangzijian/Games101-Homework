cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

#find_package(OpenCV REQUIRED)
include_directories(./../)
include_directories(./../opencv/include)
set(OpenCV_LIBRARIES ${PROJECT_SOURCE_DIR}/../opencv/lib/opencv_world450d.lib)
file(COPY ${PROJECT_SOURCE_DIR}/../opencv/bin/opencv_world450d.dll DESTINATION ${PROJECT_BINARY_DIR}/)

set(CMAKE_CXX_STANDARD 14)

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})
