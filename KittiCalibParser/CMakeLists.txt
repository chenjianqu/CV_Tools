cmake_minimum_required(VERSION 3.20)
project(KittiCalibParser)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories( ${Eigen3_INCLUDE_DIRS} )


find_package( OpenCV 3.4.16 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("${PCL_LIBRARIES}")


add_executable(KittiCalibParser main.cpp)
target_link_libraries (KittiCalibParser ${PCL_LIBRARIES}  ${OpenCV_LIBS})