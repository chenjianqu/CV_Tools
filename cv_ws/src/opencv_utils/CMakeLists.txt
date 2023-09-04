cmake_minimum_required(VERSION 3.0.2)
project(opencv_utils)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)

#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -no-pie -fno-pie -fPIC")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -no-pie -fno-pie -fPIC")

#set(CMAKE_CXX_FLAGS   "-fPIC")
#set(CMAKE_C_FLAGS   "-fPIC")

#手动申请堆栈大小 6000,000kB大小
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=6000000")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge
        image_transport)

include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS:  ${OpenCV_INCLUDE_DIRS}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


find_package(OpenCV 3.4 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_utils
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


add_executable(write_video src/write_video.cpp)
target_link_libraries(write_video ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(read_video src/read_video.cpp)
target_link_libraries(read_video ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sub_image src/sub_image.cpp)
target_link_libraries(sub_image ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sub_write_images src/sub_write_images.cpp)
target_link_libraries(sub_write_images ${catkin_LIBRARIES} ${OpenCV_LIBS})
