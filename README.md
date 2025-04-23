# Ultrafast Smartcar

## Enviroment Requirement

* Noetic


## Deployment Procedure

1. Install dependancy.

```
sudo apt-get install libpcap-dev
sudo apt-get install liborocos-bfl-dev
sudo apt install libgoogle-glog-dev
```

2. Install Osqp.

```
cd MPC_car/osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
```

3. Conpile related feature packages.

```
catkin_make -DCATKIN_WHITELIST_PACKAGES=lslidar_msgs
catkin_make -DCATKIN_WHITELIST_PACKAGES="lslidar_driver;fdilink_ahrs;robot_pose_ekf;turn_on_wheeltec_robot;wheeltec_robot_rc;common;bspline_opt;jie_ware;mpc_car"
sudo cp devel/lib/libBSPLINE_OPT_LIB.so devel/lib/libCOMMON_LIB.so /usr/lib/
catkin_make -DCATKIN_WHITELIST_PACKAGES=hybrid_astar_planner
```

## Action Procedure

### 在虚拟机器人上使用

1. 启动 gazebo 仿真

```
roslaunch racebot_gazebo tianracer.launch
```

2. 启动 robot_pose_tf 定位算法

```
roslaunch turn_on_wheeltec_robot robot_pose_ekf.launch
```

3. 启动自定义全局规划器

```
roslaunch hybrid_astar_planner test.launch
```

4. 启动MPC轨迹跟踪

```
roslaunch mpc_car simulation.launch
```

5. 启动导航状态机

```

```

6. 启动SLAM建图

```
roslaunch racebot_gazebo slam_gmapping.launch
```

### 在实际机器人上使用

1. 启动阿克曼小车

```
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

2. 启动键盘控制

```
roslaunch wheeltec_robot_rc keyboard_teleop.launch
```

3. 启动SLAM建图

```
roslaunch turn_on_wheeltec_robot slam_gmapping.launch
```

4. 启动自定义全局规划器

```
roslaunch hybrid_astar_planner test.launch
```

5. 启动MPC轨迹跟踪

```
roslaunch mpc_car simulation.launch
```

6. 启动导航状态机

```
rosrun map_server map_saver -f /home/stitch/test/test2_ws/src/Ultrafast_Smartcar/hybrid_astar_planner/test_the_plugin/maps/test1
```

# 开源参考

[https://github.com/Lord-Z/ackermann_gazebo](https://github.com/Lord-Z/ackermann_gazebo)

[https://github.com/6-robot/jie_ware](https://github.com/6-robot/jie_ware)

[https://github.com/dengpw/hybrid_astar_planner](https://github.com/dengpw/hybrid_astar_planner)

[https://github.com/QingZhuanya/corridor_Bspline_optimization](https://github.com/QingZhuanya/corridor_Bspline_optimization)

[https://github.com/Raiden49/planner](https://github.com/Raiden49/planner)

[https://github.com/qimao7213/Hybrid_A_Star-and-mpc_controller](https://github.com/qimao7213/Hybrid_A_Star-and-mpc_controller)

[https://github.com/jwk1rose/MPC-Application-in-Trajectory-Tracking](https://github.com/jwk1rose/MPC-Application-in-Trajectory-Tracking)
