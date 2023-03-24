#!/bin/bash
clear && clear
source devel/setup.bash
roslaunch svo_ros dog_d435i_vio_stereo.launch type:=vio
