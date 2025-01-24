
# 编译步骤

安装依赖

```
sudo apt-get install libpcap-dev
sudo apt-get install liborocos-bfl-dev
```

编译相关功能包（注意有先后顺序）

```
catkin_make -DCATKIN_WHITELIST_PACKAGES=lslidar_msgs
catkin_make -DCATKIN_WHITELIST_PACKAGES=lslidar_driver
catkin_make -DCATKIN_WHITELIST_PACKAGES=fdilink_ahrs
catkin_make -DCATKIN_WHITELIST_PACKAGES=robot_pose_ekf
catkin_make -DCATKIN_WHITELIST_PACKAGES=turn_on_wheeltec_robot
catkin_make -DCATKIN_WHITELIST_PACKAGES=wheeltec_robot_rc
```
