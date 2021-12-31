cmake_minimum_required(VERSION 2.6)
project(OpenCV_Test)

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++17" )

find_package( OpenCV 3.4.16 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(OpenCV_Test main.cpp)
target_link_libraries(OpenCV_Test ${OpenCV_LIBS})