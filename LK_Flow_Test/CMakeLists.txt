cmake_minimum_required(VERSION 3.20)
project(LK_Flow_Test)

set(CMAKE_CXX_STANDARD 20)

find_package( OpenCV 3.4.16 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(LK_Flow_Test main.cpp)
target_link_libraries(LK_Flow_Test ${OpenCV_LIBS} )
