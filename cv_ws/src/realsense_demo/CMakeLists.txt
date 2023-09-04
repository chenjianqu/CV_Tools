cmake_minimum_required(VERSION 3.0.2)
project(realsense_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS:  ${OpenCV_INCLUDE_DIRS}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


find_package(OpenCV 3.4 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_test
#  CATKIN_DEPENDS cv_bridge roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable( generate_cubic src/generate_cubic.cpp)
target_link_libraries(generate_cubic ${catkin_LIBRARIES} ${OpenCV_LIBS})



