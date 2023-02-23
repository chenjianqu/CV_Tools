cmake_minimum_required(VERSION 3.20)
project(Realsense2_Align)

set(CMAKE_CXX_STANDARD 14)


find_package(realsense2 2.42.0)
find_package(OpenCV 3.4 REQUIRED)

include_directories(
        include
        ${realsense2_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(Realsense2_Align main.cpp)
target_link_libraries(Realsense2_Align ${realsense2_LIBRARY} ${OpenCV_LIBS})

