# 运行MIVINS
[参考运行MIVINS](https://xiaomi.f.mioffice.cn/docs/dock4eDC14cEdkUzE4YMC7K5wEe#J64mAc)
## 1. 更新yaml-cpp  ceres  egien
[下载deb](https://xiaomi.f.mioffice.cn/docs/dock4eDC14cEdkUzE4YMC7K5wEe#)
```
cd mivins_deps_updata
sudo dpkg -i  *.deb
```

## 2. 编译cyberdog_mivins
```
git clone git@git.n.xiaomi.com:MiRoboticsLab/rop/visions/cyberdog_mivins.git
cd cyberdog_mivins
colcon build --merge-install
```

## 3. 启动launch文件
```
cd cyberdog_mivins
source install/setup.bash 
export LD_LIBRARY_PATH="/home/mi/cyberdog_mivins/src/mivins_core/libs/:$LD_LIBRARY_PATH"
#融合轮速计的版本
ros2 launch vins dog_d430i_stereo_odometry.py 
#纯视觉版本
ros2 launch vins dog_d430i_stereo.py 

#建图模式
mivins_mapping.sh
建图模式lifecycle状态切换:
ros2 lifecycle set /mivinsmapping configure
ros2 lifecycle set /mivinsmapping activate
ros2 lifecycle set /mivinsmapping deactivate 
ros2 lifecycle set /mivinsmapping cleanup 

#导航模式
mivins_localization.sh
导航模式lifecycle状态切换:
ros2 lifecycle set /mivinslocalization configure
ros2 lifecycle set /mivinslocalization activate
ros2 lifecycle set /mivinslocalization deactivate 
ros2 lifecycle set /mivinslocalization cleanup

#跟随模式
mivins_following.sh
跟随模式lifecycle状态切换:
ros2 lifecycle set /mivinsfollowing configure
ros2 lifecycle set /mivinsfollowing activate
ros2 lifecycle set /mivinsfollowing deactivate 
ros2 lifecycle set /mivinsfollowing cleanup 

```


