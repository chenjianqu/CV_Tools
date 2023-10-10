cmake_minimum_required(VERSION 3.25)
project(MaskUtils)

set(CMAKE_CXX_STANDARD 17)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(OpencvTools src/main.cpp )

add_executable(read_image_diagonal_pixels src/read_image_diagonal_pixels.cpp )
target_link_libraries(read_image_diagonal_pixels ${OpenCV_LIBS})

add_executable(convert_seg_to_mask src/convert_seg_to_mask.cpp src/utils/file_utils.cpp)
target_link_libraries(convert_seg_to_mask ${OpenCV_LIBS})

add_executable(visualize_mask_class src/visualize_mask_class.cpp )
target_link_libraries(visualize_mask_class ${OpenCV_LIBS})

add_executable(add_manual_mask src/add_manual_mask.cpp src/utils/file_utils.cpp)
target_link_libraries(add_manual_mask ${OpenCV_LIBS})

add_executable(erode_mask src/erode_mask.cpp src/utils/file_utils.cpp)
target_link_libraries(erode_mask ${OpenCV_LIBS})

add_executable(mask_operation src/mask_operation.cpp src/utils/file_utils.cpp)
target_link_libraries(mask_operation ${OpenCV_LIBS})

add_executable(mask_rgb_vis src/mask_rgb_vis.cpp src/utils/file_utils.cpp src/utils/dataloader.cpp src/utils/image_viewer.cpp)
target_link_libraries(mask_rgb_vis ${OpenCV_LIBS})

add_executable(image_dir_viewer src/image_dir_viewer.cpp src/utils/file_utils.cpp src/utils/dataloader.cpp src/utils/image_viewer.cpp)
target_link_libraries(image_dir_viewer ${OpenCV_LIBS})


