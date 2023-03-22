cmake_minimum_required(VERSION 3.20)
project(ORBextractor)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


set(CMAKE_CXX_STANDARD 17)


find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")

#set(OpenCV_DIR /home/chen/app/opencv-3.4.16/build)
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


add_executable(ORBextractor src/main.cpp src/ORBextractor.cc)
target_link_libraries(ORBextractor ${OpenCV_LIBS} ${spdlog_LIBRARIES})

