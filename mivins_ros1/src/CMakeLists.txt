project(svo_ros)
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
    pcl_ros)
    
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Ceres_INCLUDE_DIR}
  ./mivins_core/include/
)
catkin_package()

set(SOURCES
  mivins_ros1/svo_process.cpp
  mivins_ros1/svo_node_base.cpp
  mivins_ros1/visualizer.cpp
  #mivins_core/include/mivins_api/mivins_api.cpp
)


add_library(${PROJECT_NAME} ${SOURCES})
 
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  # ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/libs/libmivins_core.so
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/libs/libmivins_core.a
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/libs/libmivins_frontend.a
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/libs/libmivins_interface.a
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/libs/libvins_backend.a
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)



add_executable(svo_node mivins_ros1/svo_node.cpp)
target_link_libraries(svo_node ${PROJECT_NAME})



#############################################################################################################################
# cs_add_library(svo_nodelet src/svo_nodelet.cpp)
# target_link_libraries(svo_nodelet ${PROJECT_NAME})
# cs_add_executable(svo_visual_mapping src/visual_mapping.cpp)
# target_link_libraries(svo_visual_mapping ${PROJECT_NAME})# ${TORCH_LIBRARIES})
# cs_add_executable(svo_visual_relocalization src/visual_relocalization.cpp)
# target_link_libraries(svo_visual_relocalization ${PROJECT_NAME})# ${TORCH_LIBRARIES})
# cs_add_executable(svo_benchmark src/benchmark_node.cpp)
# target_link_libraries(svo_benchmark ${PROJECT_NAME})
#############################################################################################################################

