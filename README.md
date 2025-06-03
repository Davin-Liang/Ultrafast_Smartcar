# Ultrafast Smartcar

## 环境需要

* Noetic
* Ubuntu20.04

## 硬件组成

* 地平线智能车
* IMU
* 雷达

## 部署步骤

1. 安装基础依赖

```
sudo apt-get install libpcap-dev
sudo apt-get install liborocos-bfl-dev
sudo apt install libgoogle-glog-dev
```

2. 安装OSQP求解库

```
cd MPC_car/osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
```

3. 编译

```
catkin_make -DCATKIN_WHITELIST_PACKAGES=lslidar_msgs
catkin_make -DCATKIN_WHITELIST_PACKAGES="lslidar_driver;fdilink_ahrs;robot_pose_ekf;turn_on_wheeltec_robot;wheeltec_robot_rc;common;bspline_opt;jie_ware;mpc_car"
sudo cp devel/lib/libBSPLINE_OPT_LIB.so devel/lib/libCOMMON_LIB.so /usr/lib/
catkin_make -DCATKIN_WHITELIST_PACKAGES=hybrid_astar_planner
```

## Action Procedure

### 在虚拟环境上使用

#### 建图

1. 启动 gazebo 仿真（同时也会启动键盘控制）

```
roslaunch racebot_gazebo tianracer.launch
```

2. 启动 robot_pose_tf 定位算法

```
roslaunch turn_on_wheeltec_robot robot_pose_ekf.launch
```

3. 启动 SLAM 建图

```
roslaunch racebot_gazebo slam_gmapping.launch
```

4. 建完图后，保存地图

```
rosrun map_server map_saver -f /home/stitch/test/test2_ws/src/Ultrafast_Smartcar/hybrid_astar_planner/test_the_plugin/maps/<地图名称>
```

#### 导航

1. 启动 gazebo 仿真

```
roslaunch racebot_gazebo tianracer.launch
```

2. 启动自定义导航框架

```
roslaunch ufs_state_machine ufs_simulation.launch
```

### 在实际环境上使用

#### 建图

1. 启动小车
   需要提前为IMU模块、雷达模块上电。

```
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

2. 启动键盘控制

```
roslaunch wheeltec_robot_rc keyboard_teleop.launch
```

3. 启动 SLAM 建图

```
roslaunch turn_on_wheeltec_robot slam_gmapping.launch
```

4. 在 PC 端启动 rviz，添加相应插件
   注意点：

* 小车需和 PC 做好主从机连接

5. 建完图后，保存地图

```
rosrun map_server map_saver -f /home/stitch/test/test2_ws/src/Ultrafast_Smartcar/hybrid_astar_planner/test_the_plugin/maps/<地图名称>
```

#### 导航

1. 启动小车

```
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

2. 启动自定义导航框架

```
roslaunch ufs_state_machine ufs_realization.launch
```

# 开源参考

[https://github.com/Lord-Z/ackermann_gazebo](https://github.com/Lord-Z/ackermann_gazebo)

[https://github.com/6-robot/jie_ware](https://github.com/6-robot/jie_ware)

[https://github.com/dengpw/hybrid_astar_planner](https://github.com/dengpw/hybrid_astar_planner)

[https://github.com/QingZhuanya/corridor_Bspline_optimization](https://github.com/QingZhuanya/corridor_Bspline_optimization)

[https://github.com/Raiden49/planner](https://github.com/Raiden49/planner)

[https://github.com/qimao7213/Hybrid_A_Star-and-mpc_controller](https://github.com/qimao7213/Hybrid_A_Star-and-mpc_controller)

[https://github.com/jwk1rose/MPC-Application-in-Trajectory-Tracking](https://github.com/jwk1rose/MPC-Application-in-Trajectory-Tracking)
