#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <mpc_car/mpc_car.hpp>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

/* 将输入的任意弧度值归一化到区间[-pi, pi] */
double Mod2Pi(const double &x) 
{
  double v = fmod(x, 2 * M_PI);
  if (v < -M_PI) 
    v += 2.0 * M_PI;
  else if (v > M_PI) 
    v -= 2.0 * M_PI;
  
  return v;
}

namespace mpc_car {
class Nodelet : public nodelet::Nodelet 
{
 private:
  std::shared_ptr<MpcCar> mpcPtr_;
  ros::Timer plan_timer_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;
  ros::Subscriber path_sub_;
  VectorX state_;
  Eigen::Vector2d v;
  bool init_odom = false;
  bool init_path_seg = false;//原程序
  bool init_path_all = false;
  bool arrive_goal = false;
  double delay_ = 0.0;
  int path_seg_index = 0;//当前跟踪的seg的编号

  std::vector<std::vector<Eigen::Vector2d>> path_segs;
  std::vector<int> path_direction;

  void plan_timer_callback(const ros::TimerEvent& event) 
  {
    /* 处理话题接收的路径 */
    if (init_path_all && !init_path_seg)
    {
      if (path_segs.empty()) 
        return;
      if (path_segs[path_seg_index].size() > 2)
        mpcPtr_->setPath(path_segs[path_seg_index], path_direction[path_seg_index]);
      
      else if (path_segs[path_seg_index].size() == 2) // 这里对于路径点size为2的，就用插值的方式来插成5个
      {
        std::vector<Eigen::Vector2d> interpPoint(5);
        interpPoint[0] = path_segs[path_seg_index][0]; // 插入路径的两个路径点的第一个点
        interpPoint[4] = path_segs[path_seg_index][1]; // 插入路径的两个路径点的第二个点
        interpPoint[1] = 0.25 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
        interpPoint[2] =  0.5 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
        interpPoint[3] = 0.75 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
        mpcPtr_->setPath(interpPoint, path_direction[path_seg_index]);
        ROS_WARN("Get an interpolated traj!!");
      }
      init_path_seg = true; // 设置当前跟踪的路径已被处理，可以开始跟踪
    }

    if (init_odom && init_path_seg && !arrive_goal)
    {
      ros::Time t1 = ros::Time::now();
      std::cout << "x0: " << state_.transpose() << std::endl;

    double vel_angle = atan2(v.y(), v.x());
    double angle_diff = vel_angle - state_.z();
    int forward_dir = path_direction[path_seg_index];

    angle_diff = Mod2Pi(angle_diff);
    // 如果此时的速度方向是倒车，但是路径是正向，则速度取负数
    if(forward_dir == 1 && abs(angle_diff) > 0.75 * M_PI)  state_(3) *= -1;
    // //如果此时是倒车路径，那么需要取负数
    if (forward_dir == 0)                                  state_(3) *= -1;
    // // 如果此时的速度方向是正车，但是路径是倒车，则速度取负数
    if(forward_dir == 0 && abs(angle_diff) < 0.25 * M_PI)  state_(3) *= -1;

      if (mpcPtr_->solveQP(state_) == 11) // 11表示到达终点了
      {
        if (path_seg_index < (path_segs.size()-1))
        {
          path_seg_index += 1; // 跳到并跟踪下一条分轨迹
          if (path_segs[path_seg_index].size() > 2)
            mpcPtr_->setPath(path_segs[path_seg_index], path_direction[path_seg_index]);
          
          else if (path_segs[path_seg_index].size() == 2) // 这里对于路径点size为2的，就用插值的方式来插成5个
          {
            std::vector<Eigen::Vector2d> interpPoint(5);
            interpPoint[0] = path_segs[path_seg_index][0]; // 插入路径的两个路径点的第一个点
            interpPoint[4] = path_segs[path_seg_index][1]; // 插入路径的两个路径点的第二个点
            interpPoint[1] = 0.25 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
            interpPoint[2] =  0.5 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
            interpPoint[3] = 0.75 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
            mpcPtr_->setPath(interpPoint, path_direction[path_seg_index]);
            ROS_WARN("Get an interpolated traj!!");
          }
          
          /* 到达每一段轨迹的终点时把车停下 */
          geometry_msgs::Twist msg;
          msg.linear.x = 0;
          msg.angular.z = 0;
          cmd_pub_.publish(msg);
        }
        else
        {
          geometry_msgs::Twist msg;
          msg.linear.x = 0;
          msg.angular.z = 0;
          cmd_pub_.publish(msg);
          arrive_goal = true;
          std::cout << "-------------------------------------------------------" << std::endl;
          std::cout << "--------------------已经到达终点了！--------------------" << std::endl;
          std::cout << "-------------------------------------------------------" << std::endl;
          return;
        }
      }
      ros::Time t2 = ros::Time::now();
      double solve_time = (t2 - t1).toSec();
      std::cout << "solve qp costs: " << 1e3 * solve_time << "ms" << std::endl;

      VectorX x;
      VectorU u;
      mpcPtr_->getPredictXU(0, x, u);
      std::cout << "u: " << u.transpose() << std::endl;
      std::cout << "x: " << x.transpose() << std::endl;
      std::cout << "---------------------" << std::endl;

      geometry_msgs::Twist msg;
      msg.linear.x = state_(3) + u(0) * 0.1;
      msg.angular.z = u(1);
      cmd_pub_.publish(msg);     
      mpcPtr_->visualization();
    }
    return;
  }

  void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg) 
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.y, 
                     msg->pose.pose.orientation.z, 
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    v.x() = msg->twist.twist.linear.x;
    v.y() = msg->twist.twist.linear.y;

    state_.x() = x;
    state_.y() = y;
    state_.z() = yaw;
    state_.w() = v.norm();
    init_odom = true; // 设置标志为已经成功接收到 odom 数据
  }

  void path_callback(const nav_msgs::Path::ConstPtr& pathMsg)
  {
    path_direction.resize(0);
    path_segs.resize(0);

    std::vector<Eigen::Vector2d> path_seg; // 用于临时存储当前路径段的点
    int initDirection = pathMsg->poses[0].pose.position.z; // 从路径的第一个点提取初始方向（z 坐标）
    path_direction.emplace_back(initDirection);
    // path_direction.emplace_back(1);
    for (int i = 0; i < pathMsg->poses.size(); ++i) // 遍历每一个路径点
    {
      if (pathMsg->poses[i].pose.position.z == path_direction.back()) // 如果当前点的方向（z 坐标）与 path_direction.back()（当前方向）相同
        path_seg.emplace_back(Eigen::Vector2d(pathMsg->poses[i].pose.position.x, 
                                              pathMsg->poses[i].pose.position.y)); // 将点的二维坐标添加到当前路径段 path_seg
      else // 如果方向发生变化
      {
        path_direction.emplace_back(pathMsg->poses[i].pose.position.z); // 存储新的路径方向
        path_segs.emplace_back(path_seg); // 将 path_seg 存入 path_segs（完成一段路径的存储）
        path_seg.resize(0); // 清空 path_seg，并将当前点作为新路径段的第一个点
        path_seg.emplace_back(Eigen::Vector2d(pathMsg->poses[i].pose.position.x, 
                                              pathMsg->poses[i].pose.position.y));
      }

      if (i == (pathMsg->poses.size()-1)) // 如果当前点是路径的最后一个点
        path_segs.emplace_back(path_seg); // 将 path_seg 存入 path_segs，以确保最后一段路径被保存
    }
    //-----打印信息--------------
    std::cout << "一共生成 " << path_segs.size() << " 段轨迹, " << path_direction.size() << " 个方向" <<  std::endl;
    // for(int i = 0; i < path_segs.size(); ++i)
    // {
    //   std::cout << "第" << i << "段：" <<  std::endl;
    //   for(int j = 0; j < path_segs[i].size(); ++j)
    //   {
    //     /* 0 代表后退，1 代表前进 */
    //     std::cout << path_segs[i][j].transpose() << ", " << path_direction[i] << std::endl;
    //   }
    // }

    // mpcPtr_->setPath(pathMsg);

    init_path_all = true;
    init_path_seg = false;
    arrive_goal = false; // 每次有新的路径进来，说明就没有到达终点
    path_seg_index = 0;
    return;
  }

 public:
  void onInit(void) 
  {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    mpcPtr_ = std::make_shared<MpcCar>(nh);
    double dt = 0;
    nh.getParam("dt", dt);
    nh.getParam("delay", delay_);

    plan_timer_ = nh.createTimer(ros::Duration(dt), 
                                 &Nodelet::plan_timer_callback, 
                                 this);
    /* 订阅 odom */
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", // /car_simulator/odom_car
                                                  1, 
                                                  &Nodelet::odom_call_back, 
                                                  this);
    /* 发布小车控制话题 */
    // cmd_pub_ = nh.advertise<car_msgs::CarCmd>("car_cmd", 1);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/my_cmd_vel", 1);
    /* 订阅全局路径 */
    // ros::TransportHints().tcpNoDelay() 的作用是通过禁用 TCP 的 Nagle 算法来减少网络通信的延迟，从而提高实时性能
    path_sub_ = nh.subscribe<nav_msgs::Path>("/move_base/HybridAStarPlanner/plan_bspline", 
                                              1, 
                                              &Nodelet::path_callback, 
                                              this, 
                                              ros::TransportHints().tcpNoDelay());
  }
};
}  // namespace mpc_car

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpc_car::Nodelet, nodelet::Nodelet);