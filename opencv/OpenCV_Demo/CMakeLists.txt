cmake_minimum_required(VERSION 2.6)
project(OpenCV_Demo)

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++17" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(OpenCV_Test main.cpp)
target_link_libraries(OpenCV_Test ${OpenCV_LIBS})
