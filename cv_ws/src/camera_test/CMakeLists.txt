cmake_minimum_required(VERSION 3.0.2)
project(camera_test)


find_package(catkin REQUIRED COMPONENTS
  camera_model
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  std_msgs
)

catkin_package(
)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR})

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")


find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(camera_test src/main.cpp src/camera_model.cpp src/log_utils.cpp)
target_link_libraries(camera_test ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${spdlog_LIBRARIES})


