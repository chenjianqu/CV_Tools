cmake_minimum_required(VERSION 3.20)
project(LineFeatureTest)

set(CMAKE_CXX_STANDARD 17)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV 3.2 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

SET(LineDescriptorDIR  /home/chen/CLionProjects/CV_Tools/LineFeatureTest/line_descriptor)
include_directories(${LineDescriptorDIR}/include)


add_executable(LineFeatureTest main.cpp linefeature_tracker.cpp)
target_link_libraries(LineFeatureTest ${OpenCV_LIBS} ${LineDescriptorDIR}/lib/liblinedesc.so)

