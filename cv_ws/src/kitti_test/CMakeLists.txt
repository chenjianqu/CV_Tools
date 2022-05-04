cmake_minimum_required(VERSION 3.0.2)
project(kitti_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  stereo_msgs
  tf
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES elas_test
        #  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport libelas pcl_conversions pcl_ros roscpp sensor_msgs stereo_msgs tf
        #  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)



include_directories(src ${libelas_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


add_executable(kitti_test src/kitti_test.cpp)
target_link_libraries(kitti_test ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
