cmake_minimum_required(VERSION 3.5)
project(svo_cmake)

find_package(catkin_simple REQUIRED)
catkin_simple()

# install all included cmake files
install(DIRECTORY cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

cs_install()
cs_export(CFG_EXTRAS svo_cmake-extras.cmake)
