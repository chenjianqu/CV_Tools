cmake_minimum_required(VERSION 3.25)
project(OpencvVis)

set(CMAKE_CXX_STANDARD 17)


set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )



add_executable(OpencvVis src/main.cpp)
target_link_libraries(OpencvVis ${OpenCV_LIBS})

