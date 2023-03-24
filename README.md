# MIVINS

NOTE: 编译mivins_core过程若使用make -j*编译可能出现问题(so之间存在依赖关系)

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 18.04/20.04
ROS    noetic/foxy


### 1.2. **Ceres Solver**
version: 1.14
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **Opencv4**

```
cd <source dir>
git clone -b 4.2.0 https://github.com/opencv/opencv
git clone -b 4.2.0 https://github.com/opencv/opencv_contrib
cd opencv
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules
sudo make -Cbuild install -j4
```

### 1.4. **yaml-cpp**

```
git clone https://github.com/jbeder/yaml-cpp.git
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
```

## 2. Build Mivins

### 2.1. **build thirdparty**
```
NOTE: the APP_TYPE should be ros1(default), ros2, raw, xr;
compile mivins_core:
cd ~/mivins/thirdparty/
mkdir build && cd build
cmake .. -DAPP_TYPE=ros1 && make
```

### 2.2. **build mivins_core**
```
NOTE: the APP_TYPE should be ros1(default), ros2, raw, xr;
compile mivins_core:
cd ~/mivins/mivins_core/
mkdir build
cd build
cmake .. -DAPP_TYPE=ros1
make
make install
```
### 2.3. **build mivins_ros1**

```
cd ~/mivins/mivins_ros1/
catkin_make
```
### 2.4. **build mivins_ros2**

```
cd ~/mivins/mivins_ros2/
colcon build --merge-install
```

## 3. RUN

### 3.1. **run mivins_ros1**

```cpp
cd ~/mivins/mivins_ros1/
source devel/setup.bash    
roslaunch svo_ros airsim_vio_triple_with_depth.launch type:=vo //airsim--triple
roslaunch svo_ros tum_vio_stereo.launch type:=vio //tum--stereo
roslaunch svo_ros mi_fisheye_stereo_frontend.launch
    
rosbag play /home/mi/dataset/tum/dataset-room1_512_16.bag
rosbag play /home/mi/dataset/dog/ros1_dog_01_11_01.bag
     
```

### 3.1. **run mivins_ros2**

```cpp
cd ~/mivins/mivins_ros2/
source install/setup.bash  
ros2 launch vins mi_rgbd_fisheye_frontend.py


source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 bag play -s rosbag_v2 /home/gc/me/dataset/airsim/indoor_0.bag     
```

```cpp
realsense d430:
cd ~/mivins/mivins_ros2/
source install/setup.bash 
ros2 launch vins dog_d430i_stereo_odometry.py
ros2 launch vins dog_d430i_stereo.py
```
```cpp
mapping lifecycle命令:
ros2 lifecycle set /vins configure
ros2 lifecycle set /vins activate
ros2 lifecycle set /vins deactivate  

ros2 lifecycle set /mivinsmapping configure
ros2 lifecycle set /mivinsmapping activate
ros2 lifecycle set /mivinsmapping deactivate 
ros2 lifecycle set /mivinsmapping cleanup 

ros2 lifecycle set /mivinslocalization configure
ros2 lifecycle set /mivinslocalization activate
ros2 lifecycle set /mivinslocalization deactivate 
ros2 lifecycle set /mivinslocalization cleanup 

ros2 lifecycle set /mivinsfollowing configure
ros2 lifecycle set /mivinsfollowing activate
ros2 lifecycle set /mivinsfollowing deactivate 
ros2 lifecycle set /mivinsfollowing cleanup 
```
