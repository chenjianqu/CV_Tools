cmake_minimum_required(VERSION 3.20)
project(OpenCV_GPU)

set(CMAKE_CXX_STANDARD 20)



#OpenCV
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


#Libtorch
set(CMAKE_PREFIX_PATH "/usr/local/lib;/home/chen/app/libtorch")
find_package(TorchVision REQUIRED)
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

#CUDA
find_package(CUDA 10.2 REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})




add_executable(OpenCV_GPU main.cpp)
target_link_libraries(OpenCV_GPU ${CUDA_LIBRARIES} ${OpenCV_LIBRARIES}
        ${TORCH_LIBRARIES} TorchVision::TorchVision)

