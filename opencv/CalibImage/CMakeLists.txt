cmake_minimum_required(VERSION 3.20)
project(CalibImage)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(CalibImage main.cpp io_utils.cpp)
target_link_libraries(CalibImage ${OpenCV_LIBS})

