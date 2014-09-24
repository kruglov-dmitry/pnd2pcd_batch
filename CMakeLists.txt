cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project ( Png2Png_Batch )
#edit line below to give cmake a hint where to find a PCLConfig.cmake in case of manually built and not installed PCL
#set(PCL_DIR "/path/to/PCLConfig.cmake")
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
#edit line below to give cmake a hint where to find a OpenCVConfig.cmake in case of manually built and not installed OpenCV
#set(OpenCV_DIR "/path/to/OpenCVConfig.cmake")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( png2pcd_batch png2pcd_batch.cpp)
target_link_libraries( png2pcd_batch ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS})
