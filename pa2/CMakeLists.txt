cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

#find_package(OpenCV REQUIRED)
include_directories(./../)
include_directories(./../opencv/include)
set(OpenCV_LIBRARIES ${PROJECT_SOURCE_DIR}/../opencv/lib/opencv_world450d.lib)
file(COPY ${PROJECT_SOURCE_DIR}/../opencv/bin/opencv_world450d.dll DESTINATION ${PROJECT_BINARY_DIR}/)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(./../Eigen3/include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
