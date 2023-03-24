##############################################################################
#    Created by XIAOMI MIVINS on 2021/12/22.       #
#   Copyright © 2021 XIAOMI MIVINS. All rights reserved.     #
##############################################################################

project(mivins_core)
cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_api.cpp
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

  ${PROJECT_SOURCE_DIR}
  ${INTERFACE_PATH}/include
  ${FRONTEND_PATH}/mivins_minkindr/minkindr/include
  ${FRONTEND_PATH}/mivins_ekf_3dof/include
)

target_link_libraries(${PROJECT_NAME}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  ${LIBRARY_OUTPUT_PATH}/libmivins_interface.so
  
)

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${LIBRARY_INSTALL_PATH}
#   ARCHIVE DESTINATION ${LIBRARY_INSTALL_PATH}
# )

install(FILES  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_api.h
  DESTINATION ${INCLUDE_INSTALL_PATH}
)

