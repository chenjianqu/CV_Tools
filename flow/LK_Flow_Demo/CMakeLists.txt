cmake_minimum_required(VERSION 3.20)
project(LK_Flow_Demo)

set(CMAKE_CXX_STANDARD 20)

find_package( OpenCV 3.4.16 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(LK_Flow_Demo main.cpp)
target_link_libraries(LK_Flow_Demo ${OpenCV_LIBS} )
