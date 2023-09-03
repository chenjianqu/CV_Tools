cmake_minimum_required(VERSION 3.20)
project(KittiTest)

set(CMAKE_CXX_STANDARD 17)


find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


add_executable(KittiTest main.cpp)
target_link_libraries(KittiTest ${OpenCV_LIBRARIES})

