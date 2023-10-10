cmake_minimum_required(VERSION 3.25)
project(CvBasicTools)


set(CMAKE_CXX_STANDARD 17)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS:  ${OpenCV_INCLUDE_DIRS}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")


find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

set(THIRTY_LIBS
        ${GFLAGS_LIBRARIES}
        -lglog
        ${spdlog_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )


add_executable(concatenate_two_image src/concatenate_two_image.cpp)
target_link_libraries(concatenate_two_image  ${THIRTY_LIBS} )

add_executable(concate_two_dir_to_video src/concate_two_dir_to_video.cpp)
target_link_libraries(concate_two_dir_to_video  ${THIRTY_LIBS} )

add_executable(main src/main.cpp)
target_link_libraries(main  ${THIRTY_LIBS} )


add_executable(read_dir_to_video src/read_dir_to_video.cpp)
target_link_libraries(read_dir_to_video  ${THIRTY_LIBS} )

message(STATUS "XXXXXXXX"${CUDA_TOOLKIT_ROOT_DIR})
