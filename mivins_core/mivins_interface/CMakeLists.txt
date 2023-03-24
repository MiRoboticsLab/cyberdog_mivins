##############################################################################
#    Created by XIAOMI MIVINS on 2021/12/22.       #
#   Copyright © 2021 XIAOMI MIVINS. All rights reserved.     #
##############################################################################

project(mivins_interface)
cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake_modules")
set(THIRDPARTY_PATH "${PROJECT_SOURCE_DIR}/../../thirdparty")
set(BACKEND_MODULE_PATH "${PROJECT_SOURCE_DIR}/../vins_backend")
set(FRONTEND_MODULE_PATH "${PROJECT_SOURCE_DIR}/../mivins_frontend")

find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/data_align.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/loose_couple.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/interpolation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pose_update.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/svo_factory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vins_backend_factory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/svo_interface.cpp
)

add_library(${PROJECT_NAME} STATIC
  ${SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${GFLAGS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${YAMLCPP_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}

  ${PROJECT_SOURCE_DIR}/include
  
  ${FRONTEND_PATH}/mivins_base/include
  ${FRONTEND_PATH}/mivins_common/include
  ${FRONTEND_PATH}/mivins_direct/include
  ${FRONTEND_PATH}/mivins_tracker/include
  ${FRONTEND_PATH}/mivins_fast_corner/include
  ${FRONTEND_PATH}/mivins_img_align/include
  ${FRONTEND_PATH}/mivins_rpg_common/include
  ${FRONTEND_PATH}/mivins_minkindr/minkindr/include
  ${FRONTEND_PATH}/mivins_utils/include
  ${FRONTEND_PATH}/mivins_solver_base/include
  ${FRONTEND_PATH}/mivins_camera_models/include
  ${FRONTEND_PATH}/mivins_ekf_3dof/include
  ${BACKEND_PATH}/include
  ${THIRDPARTY_PATH}/DBoW2/include
)

target_link_libraries(${PROJECT_NAME}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  
  ${LIBRARY_OUTPUT_PATH}/libDBoW2.so
  ${LIBRARY_OUTPUT_PATH}/libvins_backend.so
  ${LIBRARY_OUTPUT_PATH}/libmivins_frontend.so
)

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${LIBRARY_INSTALL_PATH}
#   ARCHIVE DESTINATION ${LIBRARY_INSTALL_PATH}
# )
