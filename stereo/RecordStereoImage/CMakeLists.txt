cmake_minimum_required(VERSION 3.20)
project(RecordStereoImage)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(RecordStereoImage RecordStereoImage.cpp)
target_link_libraries(RecordStereoImage ${OpenCV_LIBS})

add_executable(split_stereo split_stereo.cpp)
target_link_libraries(split_stereo ${OpenCV_LIBS})

add_executable(calib calib.cpp)
target_link_libraries(calib ${OpenCV_LIBS})

add_executable(RecordZED RecordZED.cpp)
target_link_libraries(RecordZED ${OpenCV_LIBS})

