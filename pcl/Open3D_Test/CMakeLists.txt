cmake_minimum_required(VERSION 3.25)
project(Open3D_Test)

set(CMAKE_CXX_STANDARD 17)


find_package( Open3D  REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(Open3D_Test src/main.cpp)
target_link_libraries(Open3D_Test ${Open3D_LIBRARIES})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



add_executable(pcl_o3d_test src/pcl_o3d_test.cpp)
target_link_libraries(pcl_o3d_test ${Open3D_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mesh_reconstruction src/mesh_reconstruction.cpp)
target_link_libraries(mesh_reconstruction ${Open3D_LIBRARIES} ${OpenCV_LIBRARIES})
