cmake_minimum_required(VERSION 3.20)
project(Flow_Visualize)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_PREFIX_PATH "/usr/local/lib;/home/chen/app/libtorch")
find_package(TorchVision REQUIRED)
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

add_executable(Flow_Visualize main.cpp Visualization.cpp)
target_link_libraries(Flow_Visualize  ${OpenCV_LIBRARIES} ${TORCH_LIBRARIES}
        TorchVision::TorchVision)


add_executable(vis_feature vis_feature.cpp Visualization.cpp)
target_link_libraries(vis_feature  ${OpenCV_LIBRARIES} ${TORCH_LIBRARIES}
        TorchVision::TorchVision)
