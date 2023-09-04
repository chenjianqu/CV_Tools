cmake_minimum_required(VERSION 3.17)
project(TriangulationPointDemo)

set(CMAKE_CXX_STANDARD 14)




find_package(OpenCV 3 REQUIRED)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)


add_executable(TriangulationPointDemo main.cpp)
target_link_libraries(TriangulationPointDemo ${OpenCV_LIBS})