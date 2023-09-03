cmake_minimum_required(VERSION 3.20)
project(OpenCV_Video_Demo)

set(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(OpenCV_Video_Demo main.cpp)
target_link_libraries(OpenCV_Video_Demo ${OpenCV_LIBS})