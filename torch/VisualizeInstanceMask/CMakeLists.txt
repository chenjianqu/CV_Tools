cmake_minimum_required(VERSION 3.20)
project(VisualizeInstanceMask)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")


find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


#Libtorch
set(CMAKE_PREFIX_PATH "/usr/local/lib;/home/chen/app/libtorch")

find_package(TorchVision REQUIRED)
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})


add_executable(VisualizeInstanceMask main.cpp dataloader.cpp file_utils.cpp image_viewer.cpp)
target_link_libraries(VisualizeInstanceMask ${OpenCV_LIBS} ${spdlog_LIBRARIES}  ${TORCH_LIBRARIES} TorchVision::TorchVision)



