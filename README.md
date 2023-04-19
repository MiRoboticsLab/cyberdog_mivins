# MIVINS

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu:18.04/20.04
ROS2:galactic


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

### 2.1. **build mivins_core**
```
NOTE: the APP_TYPE should be ros1(default), ros2, raw, xr;
compile mivins_core:
cd ~/mivins/mivins_core/
mkdir build
cd build
cmake .. -DAPP_TYPE=ros2
make
make install
```

### 2.2. **build mivins_ros2**

```
cd ~/mivins/mivins_ros2/
colcon build --merge-install
```

## 3. RUN

### 3.1. **run mivins_ros2**
miloc service is required in both mapping mode and localization mode.

mapping mode:
```cpp
cd ~/mivins/mivins_ros2/
source install/setup.bash  
ros2 launch mivins dog_d430i_stereo_odometry_mapping.py    
```

localization mode:
```cpp
cd ~/mivins/mivins_ros2/
source install/setup.bash 
ros2 launch mivins dog_d430i_stereo_odometry_localization.py
```

following mode:
```cpp
cd ~/mivins/mivins_ros2/
source install/setup.bash 
ros2 launch mivins dog_d430i_stereo_odometry_following.py
```

lifecycle command:
```cpp
mapping lifecycle command:
ros2 lifecycle set /mivinsmapping configure
ros2 lifecycle set /mivinsmapping activate
ros2 service call /namespace/start_vins_mapping std_srvs/srv/SetBool "data: true"
ros2 service call /namespace/stop_vins_mapping cyberdog_visions_interfaces/srv/FinishMap "{finish: true, map_name: "map"}"  
ros2 lifecycle set /mivinsmapping deactivate 
ros2 lifecycle set /mivinsmapping cleanup 

mapping lifecycle command:
ros2 lifecycle set /mivinslocalization configure
ros2 lifecycle set /mivinslocalization activate
ros2 lifecycle set /mivinslocalization deactivate 
ros2 lifecycle set /mivinslocalization cleanup 

following mode command:
ros2 lifecycle set /mivinsfollowing configure
ros2 lifecycle set /mivinsfollowing activate
ros2 lifecycle set /mivinsfollowing deactivate 
ros2 lifecycle set /mivinsfollowing cleanup 
```
