cmake_minimum_required(VERSION 3.20)
project(ArucoDemo)

set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV 3.4.16 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )



add_executable(ArucoDemo main.cpp)
target_link_libraries(ArucoDemo ${OpenCV_LIBS})


