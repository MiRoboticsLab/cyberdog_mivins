##############################################################################
#      Created by XIAOMI MIVINS on 2021/12/22.             #
#     Copyright © 2021 XIAOMI MIVINS. All rights reserved.         #
##############################################################################

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
project(mivins_frontend) #LANGUAGES CXX CUDA)
cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake_modules")

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

# CUDA 10.2
find_package(CUDA ${CUDA_VERSION} QUIET)
if(NOT CUDA_FOUND)
    message(WARNING "CUDA not found!")
else()
    enable_language(CUDA)
    message(STATUS "    version     : ${CUDA_VERSION}")
    message(STATUS "    root path   : ${CUDA_TOOLKIT_ROOT_DIR}")
    message(STATUS "    include path: ${CUDA_INCLUDE_DIRS}")
    message(STATUS "    libs path   : ${CUDA_LIBRARIES}")
    set(CUDA_VERBOSE_BUILD ON)
    set(CMAKE_CUDA_ARCHITECTURES 70)
    # Specify the cuda host compiler to use the same compiler as cmake.
    set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3")
    ADD_DEFINITIONS(-D__CUDA__)
endif()

IF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
  ADD_DEFINITIONS(-D__SSE2__)
  MESSAGE("X86_64")
ELSEIF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
  ADD_DEFINITIONS(-D__ARM_NEON__)
  MESSAGE("AARCH64")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon") # -march=armv7-a
  ADD_DEFINITIONS(-D__ARM_NEON__)
  MESSAGE("ARMV7 | ARMV8")
ENDIF()


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/src/fast_10.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/src/fast_9.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/src/nonmax_3x3.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/src/fast_10_score.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_common/src/frame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_common/src/point.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/mivins_common/src/test/synthetic_dataset.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/mivins_common/src/empty.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/test/test_frame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_frame_stereo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_frame_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_imu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_odom.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/pose_optimizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/initialization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/stereo_triangulation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_frame_mono.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/reprojector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/frontend_local_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_frame_rgbdfisheye.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/channel_frame_array.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_img_align/src/sparse_img_align_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/src/transformation_optimizer.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/mivins_img_align/src/sparse_img_align_gpu.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/mivins_img_align/src/frame_gpu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_img_align/src/sparse_img_align.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/src/path_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/src/performance_monitor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/src/math_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/src/geometry_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/src/cv_utils.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/src/user_input_thread.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}mivins_utils/src/sample.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/omni_geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/camera_yaml_serialization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/ncamera.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/equidistant_fisheye_geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/camera_geometry_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/ncamera_yaml_serialization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/camera_factory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/equidistant_fisheye_projection.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/src/omni_projection.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/patch_matcher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/elder_zucker_utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/feature_patch_alignment.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/patch_affine_warp.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/feature_detector_utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/feature_detector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/depth_estimation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/depth_optimization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_tracker/src/feature_tracker_obs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_tracker/src/feature_tracker_tools.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_tracker/src/feature_tracker_viz.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_tracker/src/feature_tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_solver_base/src/loss_function.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/head_tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/matrix_3x3.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/matrix_4x4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/matrixutils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/rotation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/vectorutils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/gyroscope_bias_estimator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/lowpass_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/mean_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/median_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/neck_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/pose_prediction.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/src/sensor_fusion_ekf.cpp
)
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
IF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  LIST(APPEND SRCS ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/src/faster_corner_10_sse.cpp)
ELSE()
  LIST(APPEND SRCS ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/src/faster_corner_9_neon.cpp)
ENDIF()
IF(CUDA_FOUND)
    LIST(APPEND SRCS ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/fast_detection_gpu.cu)
    LIST(APPEND SRCS ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/src/edgelet_detection_v2_gpu.cu)
ENDIF()

add_library(${PROJECT_NAME} STATIC
  ${SRCS}
)
set(INCS)
IF(CUDA_FOUND)
    LIST(APPEND INCS ${CUDA_INCLUDE_DIRS})
ENDIF()
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PUBLIC
  ${GFLAGS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${YAMLCPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_minkindr/minkindr/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_fast_corner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_base/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_img_align/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_rpg_common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_utils/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_solver_base/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_camera_models/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_direct/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_tracker/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ekf_3dof/include
  ${INCS}
)

target_link_libraries(${PROJECT_NAME}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${BOOST_SERIALIZATION}
  yaml-cpp
)

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${LIBRARY_INSTALL_PATH}
#   ARCHIVE DESTINATION ${LIBRARY_INSTALL_PATH}
# )

