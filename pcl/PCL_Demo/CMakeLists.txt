cmake_minimum_required(VERSION 2.6)
project(PCL_Demo)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_definitions(-DSPDLOG_FMT_EXTERNAL)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


#CUDA
find_package(CUDA 11.4 REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})


#set(Eigen3_DIR /usr/local/include/eigen3)
#set(Eigen3_ROOT /home/chen/app/eigen-3.1.2/build)
#find_package(Eigen3 3.1 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#message("${EIGEN3_INCLUDE_DIR}")

#message("${PCL_DIR}")

#set(VTK_DIR /usr/local/lib)
#set(PCL_DIR /home/chen/app/pcl-pcl-1.11.0/build)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("${PCL_LIBRARIES}")

add_executable(PCL_Test src/main.cpp)
target_link_libraries (PCL_Test ${PCL_LIBRARIES} ${CUDA_LIBRARIES})

add_executable(icp_test src/icp_test.cpp)
target_link_libraries (icp_test ${PCL_LIBRARIES} ${CUDA_LIBRARIES})


add_executable(surface_reconstruct_test src/surface_reconstruct_test.cpp)
target_link_libraries (surface_reconstruct_test ${PCL_LIBRARIES} ${CUDA_LIBRARIES})






