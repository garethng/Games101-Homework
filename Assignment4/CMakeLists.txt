cmake_minimum_required(VERSION 3.10)
project(BezierCurve)


list(APPEND CMAKE_PREFIX_PATH 
    "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/arm64-osx"
)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})
