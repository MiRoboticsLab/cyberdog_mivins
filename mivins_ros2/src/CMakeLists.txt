cmake_minimum_required(VERSION 3.5)
project(mivins)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  message("CMAKE_CXX_STANDARD 14")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,'$ORIGIN/'" )
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath,'$ORIGIN/'" )

set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#mivins_core
# set(MIVINS_CORE_LIBRARY_URL https://cnbj2m-fds.api.xiaomi.net/bsp-internal/ROS/carpo-camera/mivins/mivins_core.tgz)
# execute_process(
#   COMMAND rm -rf ${CMAKE_SOURCE_DIR}/mivins_core
# )
# execute_process(
#   COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/mivins_core
#   COMMAND wget -q ${MIVINS_CORE_LIBRARY_URL} -P ${CMAKE_SOURCE_DIR}
# )

# execute_process(
#   COMMAND tar xzvf ${CMAKE_SOURCE_DIR}/mivins_core.tgz -C ${CMAKE_SOURCE_DIR}
# )

# execute_process(COMMAND rm ${CMAKE_SOURCE_DIR}/mivins_core.tgz)

find_package(cyberdog_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_visions_interfaces REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CAMERA_MODELS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ros2/
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/include/
)

link_directories(
  ${catkin_LIB_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/mivins_core/libs/
)

set (SOURCES
    #${CMAKE_CURRENT_SOURCE_DIR}/mivins_ros2/mivins_node_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ros2/mivins_process.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ros2/visualizer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ros2/mivins_lifecycle.cpp
)


find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}/unsupported
)

add_library(vins_lib SHARED ${SOURCES})

target_link_libraries(vins_lib 
  mivins_core
  mivins_interface
  mivins_frontend
  vins_backend
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  yaml-cpp
  ${PCL_LBRARIES}
#   cyberdog_visions_interfaces
)

ament_target_dependencies(vins_lib
    OpenCV
    rclcpp
    tf2_ros
    tf2_eigen
    cv_bridge
    std_msgs
    ament_index_cpp
    pcl_conversions
    geometry_msgs
    image_transport
    visualization_msgs
    nav_msgs
    Boost
    cyberdog_visions_interfaces
    std_srvs
    nav2_util
    protocol
    cyberdog_common
)


target_include_directories(vins_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_executable(mivins_node   ${CMAKE_CURRENT_SOURCE_DIR}/mivins_ros2/mivins_lifecycle_node.cpp)
target_link_libraries(mivins_node vins_lib mivins_core mivins_interface mivins_frontend vins_backend yaml-cpp)
set_target_properties(mivins_node
PROPERTIES
CMAKE_INSTALL_RPATH lib/${PROJECT_NAME}
)
install(TARGETS vins_lib mivins_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(vins_lib mivins_core mivins_interface mivins_frontend vins_backend)

install(PROGRAMS
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mivins_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/param
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(include)
ament_package()
