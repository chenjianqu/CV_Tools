cmake_minimum_required(VERSION 3.0.2)
project(stereo_pointcloud)



add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
		std_msgs
        )

catkin_package(
        #${catkin_INCLUDE_DIRS}
)

include_directories(
		# include
		${catkin_INCLUDE_DIRS}
)




find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(stereo_to_pointcloud src/stereo_to_pointcloud.cpp src/io_utils.cpp)
target_link_libraries(stereo_to_pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${spdlog_LIBRARIES})

add_executable(build_pointcloud src/build_pointcloud.cpp src/io_utils.cpp)
target_link_libraries(build_pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${spdlog_LIBRARIES})




