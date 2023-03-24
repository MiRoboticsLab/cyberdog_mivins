#!/bin/bash
clear && clear
source devel/setup.bash
roslaunch svo_ros airsim_vio_triple_with_depth.launch type:=vo
