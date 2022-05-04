cmake_minimum_required(VERSION 3.0.2)
project(FCOS_3DBox)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        image_transport
        )

find_package(Eigen3 REQUIRED)
include_directories( ${Eigen3_INCLUDE_DIRS} )

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS:  ${OpenCV_INCLUDE_DIRS}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


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


aux_source_directory(src src_SOURCES)
message("${src_SOURCES}")


add_executable(fcos_parser src/fcos_parser.cpp src/visualization.cpp)
target_link_libraries(fcos_parser ${catkin_LIBRARIES} ${OpenCV_LIBS} ${spdlog_LIBRARIES})


