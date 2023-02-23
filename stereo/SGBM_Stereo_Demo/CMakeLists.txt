cmake_minimum_required(VERSION 3.20)
project(SGBM_Stereo_Demo)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(SGBM_Stereo_Demo main.cpp)
target_link_libraries(SGBM_Stereo_Demo ${OpenCV_LIBS} ${PCL_LIBRARIES})



