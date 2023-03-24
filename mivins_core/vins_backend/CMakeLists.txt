project(vins_backend)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake_modules")

set(THIRDPARTY_PATH "${PROJECT_SOURCE_DIR}/../../thirdparty")
set(FRONTEND_PATH "${PROJECT_SOURCE_DIR}/../mivins_frontend")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)

find_package(OpenMP QUIET)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found")
  add_definitions(-DUSE_OPENMP)
endif ()

set(CUDA OFF)

if (CUDA)
  find_package(CUDA QUIET)
  if (CUDA_FOUND)
    message("-- Found CUDA version ${CUDA_VERSION}: "
        "${CUDA_LIBRARIES};"
        "${CUDA_cusolver_LIBRARY};"
        "${CUDA_cusparse_LIBRARY}")

    include_directories(${CUDA_INCLUDE_DIRS})

    message("system platform: ${CMAKE_SYSTEM_PROCESSOR}")

    if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      message("-- x86_64 platform")
      ADD_DEFINITIONS(-D__GPU__)
      link_directories(/usr/local/cuda/targets/x86_64-linux/lib/)
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      message("-- aarch64 platform")
      ADD_DEFINITIONS(-D__GPU__)
      include_directories(/usr/local/cuda/targets/aarch64-linux/include)
      link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
    else ()
      message("-- ERROR: unknow platform")
      ADD_DEFINITIONS(-D__CPU__)
    endif()

  else (not CUDA_FOUND)
    message("-- Did not find CUDA library, disabling CUDA support.")
    update_cache_variable(CUDA OFF)
    ADD_DEFINITIONS(-D__CPU__)
    list(APPEND CERES_COMPILE_OPTIONS CERES_NO_CUDA)
  endif (CUDA_FOUND)
else (not CUDA)
  message("-- Building without CUDA.")
  ADD_DEFINITIONS(-D__CPU__)
  list(APPEND CERES_COMPILE_OPTIONS CERES_NO_CUDA)
endif (CUDA)

include_directories(
  include

  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  
  ${FRONTEND_PATH}/mivins_base/include
  ${FRONTEND_PATH}/mivins_direct/include
  ${FRONTEND_PATH}/mivins_tracker/include
  ${FRONTEND_PATH}/mivins_common/include

  ${FRONTEND_PATH}/mivins_utils/include
  ${FRONTEND_PATH}/mivins_solver_base/include
  ${FRONTEND_PATH}/mivins_camera_models/include
  ${FRONTEND_PATH}/mivins_rpg_common/include
  ${FRONTEND_PATH}/mivins_minkindr/minkindr/include
  
  ${THIRDPARTY_PATH}/DBoW2/include
)

ADD_DEFINITIONS(-D__MIVINS__)

#############
# LIBRARIES #
#############
set(HEADERS
  include/vins_backend_interface.h

  include/initial/solve_5pts.h
  include/initial/initializer.h
  include/initial/initial_sfm.h
  include/initial/initial_alignment.h
  include/initial/initial_ex_rotation.h

  include/factor/imu_factor.h
  include/factor/odom_factor.h
  include/factor/odom_factor_s1d.h
  include/factor/odom_factor_s3d.h
  include/factor/integration_base.h
  include/factor/integration_odom.h
  include/factor/visual_base_factor.h
  include/factor/marginalization_factor.h
  include/factor/pose_local_parameterization.h
  include/factor/diff_bundle_diff_cam_factor.h
  include/factor/diff_bundle_same_cam_factor.h
  include/factor/same_bundle_diff_cam_factor.h

  include/estimator/estimator.h
  include/estimator/frame_manager.h
  # include/estimator/optimize_pose.h
  include/estimator/motion_detector.h
  include/estimator/outlier_rejection.h

  include/common/global.h
  include/common/common_lib.h
  include/common/parameters.h
  include/common/color_print.h
)

set(SOURCES
  src/vins_backend_interface.cpp
  
  src/initial/solve_5pts.cpp
  src/initial/initial_sfm.cpp
  src/initial/initializer.cpp
  src/initial/initial_alignment.cpp
  src/initial/initial_ex_rotation.cpp

  src/factor/imu_factor.cpp
  src/factor/odom_factor.cpp
  src/factor/odom_factor_s1d.cpp
  src/factor/odom_factor_s3d.cpp
  src/factor/integration_base.cpp
  src/factor/integration_odom.cpp
  src/factor/visual_base_factor.cpp
  src/factor/marginalization_factor.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/diff_bundle_diff_cam_factor.cpp
  src/factor/diff_bundle_same_cam_factor.cpp
  src/factor/same_bundle_diff_cam_factor.cpp

  src/estimator/estimator.cpp
  src/estimator/frame_manager.cpp
  # src/estimator/optimize_pose.cpp
  src/estimator/motion_detector.cpp
  src/estimator/outlier_rejection.cpp

  src/common/parameters.cpp
)

LIST(APPEND SOURCES src/common/global_mivins.cpp)
LIST(APPEND HEADERS include/common/global_mivins.h)

LIST(APPEND SOURCES src/estimator/optimize_pose_mivins.cpp)
LIST(APPEND HEADERS include/estimator/optimize_pose_mivins.h)

add_library(${PROJECT_NAME} STATIC 
  ${SOURCES} 
  ${HEADERS}
)

target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES} 
  ${CERES_LIBRARIES}
  ${GLOG_LIBRARIES}
)

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${LIBRARY_INSTALL_PATH}
#   ARCHIVE DESTINATION ${LIBRARY_INSTALL_PATH}
# )
