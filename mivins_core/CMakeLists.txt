project(mivins_core)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_REQUIRED_FLAGS "-Wno-error")
option(LOG_DEBUG "option for debug" OFF)
if (LOG_DEBUG) 
	add_definitions(-DLOG_DEBUG)
endif(LOG_DEBUG)

message("Build order: ")
message("if xr: cmake .. -DAPP_TYPE=xr")    
message("if raw: cmake .. -DAPP_TYPE=raw")    
message("if ros1: cmake .. -DAPP_TYPE=ros1")
message("if ros2: cmake .. -DAPP_TYPE=ros2")    

if(NOT APP_TYPE)
  set(APP_TYPE "ros1")
  message("Use the default build type: ros1")
else()
  if( (${APP_TYPE} STREQUAL "ros1") OR (${APP_TYPE} STREQUAL "ros2")
       OR (${APP_TYPE} STREQUAL "raw") OR (${APP_TYPE} STREQUAL "xr") )
    set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/../mivins_${APP_TYPE}/src/mivins_core/libs")
    set(INCLUDE_INSTALL_PATH "${PROJECT_SOURCE_DIR}/../mivins_${APP_TYPE}/src/mivins_core/include")
  else()
    message("Error APP_TYPE")
  endif()
endif()

message("The shared libraries would be install to ${LIBRARY_OUTPUT_PATH}")
message("The include files would be install to ${INCLUDE_INSTALL_PATH}")

# set(CMAKE_INSTALL_PREFIX ${LIBRARY_INSTALL_PATH} CACHE PATH "Installation Directory")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

set(THIRDPARTY_PATH "${PROJECT_SOURCE_DIR}/../thirdparty")

set(API_PATH "${PROJECT_SOURCE_DIR}/mivins_api")
set(BACKEND_PATH "${PROJECT_SOURCE_DIR}/vins_backend")
set(FRONTEND_PATH "${PROJECT_SOURCE_DIR}/mivins_frontend")
set(INTERFACE_PATH "${PROJECT_SOURCE_DIR}/mivins_interface")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED) 
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

add_subdirectory(${PROJECT_SOURCE_DIR}/mivins_frontend)
add_subdirectory(${PROJECT_SOURCE_DIR}/vins_backend)
add_subdirectory(${PROJECT_SOURCE_DIR}/mivins_interface)
add_subdirectory(${PROJECT_SOURCE_DIR}/mivins_api)


