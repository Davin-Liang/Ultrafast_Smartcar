/*********************************************************************
 *
 *  BSD 3-Clause License
 *
 *  Copyright (c) 2021, dengpw
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1 Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2 Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   3 Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Author:  dengpw
 *********************************************************************/
#include <iostream>
#include "planner_core.h"
#include <tf/transform_datatypes.h>
#include <ros/node_handle.h>
#include "astar.h"
#include "hybrid_astar.h"
#include <math/vec2d.h>
#include <math/math_utils.h>

PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)//注册为类插件的声明

namespace hybrid_astar_planner {

HybridAStarPlanner::HybridAStarPlanner():
    initialized_(false),costmap(NULL),resolution(1.0) {
  std::cout << "creating the hybrid Astar planner" << std::endl;
}


void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}


void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *_costmap, std::string frame_id) {
  if(!initialized_) {
    ROS_INFO("initializing the hybrid Astar planner");
    /* 订阅global_costmap的内容，以便获取参数 */
    ros::NodeHandle nh("~/global_costmap");
    ros::NodeHandle nh2("~/");
    ros::NodeHandle private_nh("~/" + name);

    bspline_optimizer_rebound_.reset(new opt_planner::BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(private_nh);

    nh2.param("use_hybrid_astar", use_hybrid_astar, true);
    nh2.getParam("UfsPlaner/max_inflate_iter", max_inflate_iter_);
    nh2.getParam("UfsPlaner/expansion_step_length", expansion_step_length_);
    nh2.getParam("UfsPlaner/control_point_distance", control_point_distance_);
    nh2.getParam("UfsPlaner/max_vel", max_vel_);


    if(use_hybrid_astar) {
      ROS_INFO("Using hybrid_astar mode!");
    } else {
      ROS_INFO("Using Astar mode!");
    }
    costmap = _costmap;
    frame_id_ = frame_id;
    std::cout << frame_id << std::endl;
    /* 初始化发布路径的主题 */
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    path_vehicles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
    make_plan_srv_ = private_nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);
    corridor_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);
    path_pub = private_nh.advertise<nav_msgs::Path>("control_points_path", 1);
    plan_pub_bspline = private_nh.advertise<nav_msgs::Path>("plan_bspline", 1);
  }
  initialized_ = true;
}//end of constructor function HybridAStarPlanner

HybridAStarPlanner::~HybridAStarPlanner() {

}//end of deconstructor function HybridAStarPlanner


bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, 
                                         nav_msgs::GetPlan::Response& resp) 
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;
  return true;
}


bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                  const geometry_msgs::PoseStamped &goal, 
                                  std::vector<geometry_msgs::PoseStamped>& plan) 
{
  plan.clear();
  std::vector<geometry_msgs::PoseStamped> plan_;
  // std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
  // std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
  Vec3d start_state(start.pose.position.x,
                    start.pose.position.y,
                    tf::getYaw(start.pose.orientation));

  Vec3d goal_state(goal.pose.position.x,
                   goal.pose.position.y,
                   tf::getYaw(goal.pose.orientation));

  Expander* _planner;

  // ROS_INFO("the resolution of cost map: %f ",costmap->getResolution());
  if (use_hybrid_astar)
    _planner = new hybridAstar(frame_id_,costmap);
  else
    _planner = new astar(frame_id_,costmap);

  /* 检查设定的目标点参数是否合规 */
  if ( !(checkStartPose(start) && checkgoalPose(goal)) ) 
  {
    ROS_WARN("Failed to create a global plan!");
    return false; // 代表规划失败
  }

  /* ----------------------------------------------------------------------------------------------- */
  /* ------------------------------------生成Hybird A*前端全局路径------------------------------------ */
  /* ----------------------------------------------------------------------------------------------- */
  visualization_msgs::MarkerArray AstarpathNodes;
  if( !_planner->calculatePath(start, 
                              goal, 
                              costmap->getSizeInCellsX(), 
                              costmap->getSizeInCellsY(), 
                              plan_, 
                              path_vehicles_pub_, 
                              AstarpathNodes) ) 
  {
    if (_planner != nullptr)
      delete _planner;

    return false; // 代表规划失败
  }

  if (_planner != nullptr)
    delete _planner;

  std::cout << "规划出来的初始全局路径一共有 " << plan_.size() << " 个轨迹点！" << std::endl;

  /* ----------------------------------------------------------------------------------------------- */
  /* --------------------------------------------分割路径-------------------------------------------- */
  /* ----------------------------------------------------------------------------------------------- */
  // 分割路径，为每个分段轨迹生成速度、加速度、转向角等动态信息
  HybridAStartResult result;
  DataTransform(plan_, &result);
  if (!TrajectoryPartition(result, &partition_trajectories)) 
  {
      std::cout << "分割路径失败！" << std::endl;
      return false;
  }
  std::cout << "完成路径分割！" << std::endl;

  /* ----------------------------------------------------------------------------------------------- */
  /* ------------------------------------------构建安全走廊------------------------------------------ */
  /* ----------------------------------------------------------------------------------------------- */
  std::vector<VecCube> corridors;
  VecCube connected_corridors;
  Timer time_bef_corridor; // 用于计算耗时
  int segment = 0; // 用于标记现在处理到哪段轨迹

  /* 为每段轨迹生成安全走廊 */
  for (const auto & trajectory : partition_trajectories) 
  {
    segment++;
    VectorVec3d post_path;

    for (int i = 0; i < int(trajectory.x.size()); i++)
    {
      post_path.push_back({trajectory.x[i], trajectory.y[i], 0}); // 只需取出每个轨迹点的位置坐标
    }
    VecCube temp = corridorGeneration(post_path, segment);
    std::cout << "成功为第 " << segment << " 段轨迹生成安全走廊！"<< std::endl;
    // timeAllocation(temp.second, start_state, goal_state); // 为安全走廊分配时间信息
    // timeAllocation(temp, start_state, goal_state); // 为安全走廊分配时间信息
    // std::cout << "成功为第 " << segment << " 处安全走廊分配时间信息！"<< std::endl;
    corridors.push_back(temp);
  }
  std::cout << "全局路径一共有" << segment << "段轨迹！" << std::endl;
  ConnectCorridors(corridors, connected_corridors);
  std::cout << "成功合并走廊！" << std::endl;
  std::cout << "生成安全走廊所花费的时间为: " << time_bef_corridor.End() <<" ms."<< std::endl;

  PublishCorridor(connected_corridors); // 发布安全走廊

  /* ----------------------------------------------------------------------------------------------- */
  /* -------------------------------------------B 样条优化------------------------------------------- */
  /* ----------------------------------------------------------------------------------------------- */
  // double total_opt_time = 0.0;
  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> BSplineSmooth_set;
  for (int i = 0; i < int(partition_trajectories.size()); ++i)
  {
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> point_set;
    double start_angle, end_angle, distance;
    int sample_count = 2; // 隔 sample_count-1 个原始点进行采样

    /* 得到样本路径 */
    getSample(partition_trajectories[i], &point_set, sample_count, start_angle, end_angle, distance);
    std::cout << "得到第 " << i << " 样本路径" << std::endl;

    if (point_set.size() < 4) 
    {
      std::cout << "本次优化的抽样路径的路径点小于 4 个, 不进行优化, 跳过该路径!!!" << std::endl;
      continue; // 防止参考点数量太少
    }

    /* 确定时间区间的大小 */
    double ts = distance > 0.1 ? (control_point_distance_ / max_vel_ * 1.5) 
                               : (control_point_distance_ / max_vel_ * 5);

    /* 初始化起点和终点的速度和加速度约束 */
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> start_end_derivatives;
    start_end_derivatives.push_back({cos(start_angle), sin(start_angle), 0}); // 起点的一阶导数（速度）
    start_end_derivatives.push_back({cos(end_angle), sin(end_angle), 0}); // 终点的一阶导数（速度）
    start_end_derivatives.push_back(Eigen::Vector3d::Zero()); // 起点的二阶导数（加速度）
    start_end_derivatives.push_back(Eigen::Vector3d::Zero()); // 终点的二阶导数（加速度）
    
    Eigen::MatrixXd ctrl_pts;
    /* B样条曲线拟合 */
    opt_planner::UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, &ctrl_pts);
    publishPathFromCtrlPts(ctrl_pts);
    bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    Timer time_bef_optimization;
    /* B样条曲线优化 */
    bool optimization_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts, corridors[i]);
    // publishPathFromCtrlPts(ctrl_pts);
    std::cout << "Time consume in optimization is: " << time_bef_optimization.End() <<" ms"<< std::endl;

    if (!optimization_success)
    {
      ROS_ERROR("OPTIMIZE failed!");
      std::cout << "OPTIMIZE failed in iteration: " << i << std::endl;

      return false;
    }

    opt_planner::UniformBspline pos = opt_planner::UniformBspline(ctrl_pts, 3, ts);
    double tn = pos.getTimeSum();
    for (double t = 0; t <= tn; t += 0.01) 
    {
      Eigen::Vector3d pt = pos.evaluateDeBoor(t);
      BSplineSmooth_set.emplace_back(pt); // 得到优化后的路径
    }
  }
  std::cout << "成功优化所有路径样本路径" << std::endl;
  
  double b_spline_length = 0;
  std::cout << " B 样条优化后的路径点一共有 " << BSplineSmooth_set.size() << " 个！" << std::endl;
  for(size_t i = 0; i < BSplineSmooth_set.size(); ++i)
  {
    if (i < BSplineSmooth_set.size() - 1)
      b_spline_length += (BSplineSmooth_set[i+1].head(2) - BSplineSmooth_set[i].head(2)).norm(); // 计算优化后的路径的长度
  }
  std::cout << "length in optimization is: " << b_spline_length <<" m"<< std::endl;

  /* 参数后期处理，发布到RViz上进行可视化 */
  publishPlan(plan_); // 发布初始的全局规划路径
  publishPathNodes(plan_);
  publishPlan_bspline(BSplineSmooth_set); // 发布 B 样条优化后的路径

  plan.swap(plan_);
  return true; // 代表规划成功
} /* end of makeplan */

bool HybridAStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start) 
{
  unsigned int startx,starty;
  if ( costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty) ) 
  {
    return true;
  }
  ROS_WARN("The Start pose is out of the map!");
  return false;
} /* end of checkStartPose */

bool HybridAStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal) 
{
  unsigned int goalx,goaly;
  if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly)) 
  {
    if (costmap->getCost(goalx, goaly) > 252) 
    {
      // std::cout << costmap->getCost(goalx, goaly) << std::endl;
      ROS_WARN("The Goal pose is out of the map! %d",costmap->getCost(goalx, goaly));
      ROS_WARN("The Goal pose is occupied , please reset the goal!");
      return false;
    }
    return true;
  }
  return false;
} /* end of checkgoalPose */

void HybridAStarPlanner::DataTransform(std::vector<geometry_msgs::PoseStamped>& plan, HybridAStartResult* result)
{
  for (unsigned int i = 0; i < int(plan.size()); i++)
  {
    result->x.emplace_back(plan[i].pose.position.x);
    result->y.emplace_back(plan[i].pose.position.y);//todo
    result->phi.emplace_back(tf::getYaw(plan[i].pose.orientation));
  }
} /* end of DataTransform */

bool HybridAStarPlanner::TrajectoryPartition(const HybridAStartResult& result, std::vector<HybridAStartResult>* partitioned_result) 
{
  /* 此时的result只有路径静态信息，x,y,phi */
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) 
  {
    std::cout << "states sizes are not equal when do trajectory partitioning of "
              "Hybrid A Star result" << std::endl;
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back(); // 在输出容器中添加一个新的轨迹段
  auto* current_traj = &(partitioned_result->back()); // 指向当前轨迹段的指针，便于修改当前段的数据
  double heading_angle = phi.front(); // 车头的角度
  const common::math::Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]); // 从 x[0], y[0] 到 x[1], y[1] 的向量
  double tracking_angle = init_tracking_vector.Angle(); // 计算从 x[0], y[0] 到 x[1], y[1] 的向量角度，表示起始段的运动方
  bool current_gear = // 判断当前的行驶方向，true 表示车辆沿正方向行驶，false 表示反方向
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);

  /* 将Hybrid A*计算的轨迹结果，按照行驶的正反方向切换，分割为数段 */
  for (size_t i = 0; i < horizon - 1; ++i) 
  {
    heading_angle = phi[i];
    const common::math::Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear = std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) < (M_PI_2);
    if (gear != current_gear) 
    { // 判断方向是否发生了变化
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) 
  {
    /* 为每个分段轨迹生成速度、加速度、转向角等动态信息 */
    if (!GenerateSpeedAcceleration(&result)) // 根据result中的静态信息x,y,phi，利用相邻点、逐点求动态信息v,a,steer
    {
        ROS_ERROR("GenerateSpeedAcceleration fail.");
        return false;
    }
  }
  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp; 
  std::cout << "speed profile total time: " << diff.count() * 1000.0 <<" ms."<< std::endl; // 记录时间，输出生成速度信息所需的总时间
  return true;
} /* end of TrajectoryPartition */

bool HybridAStarPlanner::GenerateSpeedAcceleration(HybridAStartResult* result) 
{
  // Sanity Check
  delta_t_ = 0.5;
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) 
  {
    ROS_ERROR("result size check when generating speed and acceleration fail"); 
    return false;
  }
  const size_t x_size = result->x.size();

  /* 根据位置求解速度 */
  result->v.push_back(0.0); // initial and end speed are set to be zeros
  for (size_t i = 1; i + 1 < x_size; ++i) // 从第二个点开始计算，因为第一个点默认速度为 0
  {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) * // delta_t_ = 0.5
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +  //上面是x方向上，利用连续3点的坐标求中间点的速度，平均速度
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;  //上面是y方向上，利用连续3点的坐标求中间点的速度，平均速度
    result->v.push_back(discrete_v); // 将计算得到的速度存储好
  }
  result->v.push_back(0.0); // 设置最后一个点速度为 0

  /* 根据速度求解加速度 */
  for (size_t i = 0; i + 1 < x_size; ++i) 
  {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  /* load steering from phi */
  for (size_t i = 0; i + 1 < x_size; ++i) 
  {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            Constants::wheel_base / Constants::move_step_size; 
    if (result->v[i] > 0.0) 
      discrete_steer = std::atan(discrete_steer);
    else 
      discrete_steer = std::atan(-discrete_steer);
    result->steer.push_back(discrete_steer);
  }
  return true;
} /* end of GenerateSpeedAcceleration */

VecCube HybridAStarPlanner::corridorGeneration(const VectorVec3d &path_coord, int segment)
{
  VecCube SmoothPathcubeList;
  VecCube bsplinecubeList;
  Cube lstcube;
  for (size_t i = 0; i < path_coord.size(); ++i)
  {
    Cube cube = generateCube(path_coord[i]); // 根据一个点生成一个平面长方形，其实该长方形是一个点
    auto result = inflateCube(cube, lstcube); // 检查该次循环生成的长方形是否在上次生成的长方形内
    if ((0 == i) || (path_coord.size() == i + 1) || (1 == i % 2)) 
      bsplinecubeList.push_back(result.first);

    if (result.second == false) // 当前路径点膨胀一次之后的cube被上一个cube完全包含,对该点进行剪枝
      continue;
    cube = result.first;
    lstcube = cube;
    SmoothPathcubeList.push_back(cube);
  }

  return bsplinecubeList;
} /* end of corridorGeneration */

Cube HybridAStarPlanner::generateCube(Vec3d pt)
{
/*
      P4-----------P3 
      /           /           /y 
    /           /           /    
  P1-----------P2          /      
                          /--------> x                        
*/       
  Cube cube;
  cube.index = 0;
  /* 限制点在地图范围内 */ // TODO:
  // pt(0) = std::max(std::min(pt(0), map_x_upper_), map_x_lower_); //_pt_min_x < pt < _pt_max_x
  // pt(1) = std::max(std::min(pt(1), map_y_upper_), map_y_lower_);

  cube.center = pt;

  double x_u = pt(0);
  double x_l = pt(0);
  
  double y_u = pt(1);
  double y_l = pt(1);

  /* 将cube初始化为一个点 */ // 四个顶点为同一坐标，所以说将cube初始化为一个点
  cube.vertex.row(0) = Vec3d(x_l, y_l, 0); // 设置第一个顶点，坐标是 (x_l, y_l, 0)，位于矩形的左下角，即 P1
  cube.vertex.row(1) = Vec3d(x_u, y_l, 0); // 设置第二个顶点，坐标是 (x_u, y_l, 0)，位于矩形的右下角，即 P2
  cube.vertex.row(2) = Vec3d(x_u, y_u, 0); // 设置第三个顶点，坐标是 (x_u, y_u, 0)，位于矩形的右上角，即 P3
  cube.vertex.row(3) = Vec3d(x_l, y_u, 0); // 设置第四个顶点，坐标是 (x_l, y_u, 0)，位于矩形的左上角，即 P4

  return cube;
} /* end of generateCube */

std::pair<Cube, bool> HybridAStarPlanner::inflateCube(const Cube &cube, const Cube &lstcube)
{
  Cube cubeMax = cube;

  // Inflate sequence: right, left, front, back, above, below                                                                              
  Eigen::MatrixXi vertex_idx = Eigen::MatrixXi::Zero(8, 3);
  
  // 判断当前的路径点是否触碰障碍,因为传入的cube是一个点
  for (int i = 0; i < 4; i++)
  { 
    // double coord_x = std::max(std::min(cube.vertex(i, 0), map_x_upper_), map_x_lower_);
    // double coord_y = std::max(std::min(cube.vertex(i, 1), map_y_upper_), map_y_lower_);
    double coord_x = cube.vertex(i, 0);
    double coord_y = cube.vertex(i, 1);
  
    Vec2i map_pt_index(0, 0);
    unsigned int temp1, temp2;
    if (!costmap->worldToMap(coord_x, coord_y, temp1, temp2))
      std::cout << "世界坐标超出的地图边界1" << std::endl;
    map_pt_index[0] = temp1;
    map_pt_index[1] = temp2;
    
    if (HasObstacle(map_pt_index[0], map_pt_index[1]))
    {       
      std::cout << "[Planning Node] 规划出来的路径有轨迹点在障碍物上！" << std::endl;
      
      return std::make_pair(cubeMax, false);
    }
    
    vertex_idx.row(i).head(2) = map_pt_index;  // 若未触碰障碍，将该点的 x, y 坐标赋值给 vertex_idx 的对应行,等待膨胀
  }

  /*
              P4------------P3 
              /|           /|              ^
             / |          / |              | z
           P1--|---------P2 |              |
            |  P8--------|--p7             |
            | /          | /               /--------> y
            |/           |/               /  
           P5------------P6              / x
  */           
  /* 开始膨胀 */
  int id_x, id_y;
  bool collide;
  Eigen::MatrixXi vertex_idx_lst = vertex_idx; // 存储未膨胀前 cube 的 idx

  // 依次将cube的某个面(例如P1-P4-P8-P5)向对应的坐标轴方向扩展_step_length, 并检查这个面是否触碰障碍物
  int iter = 0;
  while (iter < max_inflate_iter_) // 迭代次数也就是最大扩展距离
  {   
    /* --------------------------------------------Y Axis-------------------------------------------- */
    int y_lo = std::max(0, vertex_idx(1, 1) - expansion_step_length_);
    int y_up = std::min(static_cast<int>(costmap->getSizeInCellsY()), vertex_idx(2, 1) + expansion_step_length_);

    // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
    // ############################################################################################################
    collide = false;
    Eigen::Vector2d tem_pos;
    for(id_y = vertex_idx(2, 1); id_y <= y_up; id_y ++)
    {   
      if (collide == true) 
        break; // 已触碰障碍物，停止膨胀
      
      for (id_x = vertex_idx(2, 0); id_x >= vertex_idx(3, 0); id_x--) // P2P3
      {
        unsigned int temp1, temp2;
        if (!costmap->worldToMap(id_x, id_y, temp1, temp2))
          ;
          // std::cout << "世界坐标超出的地图边界2" << std::endl;
        tem_pos[0] = temp1;
        tem_pos[1] = temp2;
        
        if (HasObstacle((int64_t)id_x, (int64_t)id_y)) // the voxel is occupied
        {   
          collide = true;
          break;
        }
      }
    }

    if (collide)
    {
      vertex_idx(2, 1) = std::max(id_y-2, vertex_idx(2, 1));   // _step_length = 1, 若有障碍说明之前cube就已经到达边界 //cause when collide = true, id_y++ run one more time
      vertex_idx(3, 1) = std::max(id_y-2, vertex_idx(3, 1));   // 此时id_y = y_up+1
    }
    else
        vertex_idx(2, 1) = vertex_idx(3, 1)  = id_y - 1;  // for循环后id_y = y_up+1

    // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face 
    // ############################################################################################################
    collide  = false;
    for (id_y = vertex_idx(1, 1); id_y >= y_lo; id_y--) 
    {   
      if (collide == true)   // 退出多层for循环
        break;
      for (id_x = vertex_idx(1, 0); id_x >= vertex_idx(0, 0); id_x--) // P1P4
      {    
        if (collide == true) 
          break;

        unsigned int temp1, temp2;
        if (!costmap->worldToMap(id_x, id_y, temp1, temp2))
          ;
          // std::cout << "世界坐标超出的地图边界3" << std::endl;
        tem_pos[0] = temp1;
        tem_pos[1] = temp2;

        if (HasObstacle( (int64_t)id_x, (int64_t)id_y)) // the voxel is occupied
        {   
            collide = true;
            break;
        }
      }
    }

    if (collide)
    {
      vertex_idx(1, 1) = std::min(id_y+2, vertex_idx(1, 1));
      vertex_idx(0, 1) = std::min(id_y+2, vertex_idx(0, 1));
    }
    else
      vertex_idx(1, 1) = vertex_idx(0, 1)  = id_y + 1;    
    
    /* --------------------------------------------X Axis-------------------------------------------- */
    int x_lo = std::max(0, vertex_idx(0, 0) - expansion_step_length_);
    int x_up = std::min(static_cast<int>(costmap->getSizeInCellsX()), vertex_idx(1, 0) + expansion_step_length_);
    // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
    // ############################################################################################################

    collide = false;
    for (id_x = vertex_idx(1, 0); id_x <= x_up; id_x++)
    {   
      if (collide == true) 
        break;
      for (id_y = vertex_idx(1, 1); id_y <= vertex_idx(2, 1); id_y++) // P1P2
      {
        if (collide == true) 
          break;
        unsigned int temp1, temp2;
        if (!costmap->worldToMap(id_x, id_y, temp1, temp2))
          ;
          // std::cout << "世界坐标超出的地图边界4" << std::endl;
        tem_pos[0] = temp1;
        tem_pos[1] = temp2; 
        if (HasObstacle((int64_t)id_x, (int64_t)id_y)) // the voxel is occupied
        {   
          collide = true;
          break;
        }
      }
    }

    if (collide)
    {
      vertex_idx(1, 0) = std::max(id_x-2, vertex_idx(1, 0)); 
      vertex_idx(2, 0) = std::max(id_x-2, vertex_idx(2, 0)); 
    }
    else
      vertex_idx(1, 0) = vertex_idx(2, 0) = id_x - 1;    

    // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
    // ############################################################################################################
    collide = false;
    for (id_x = vertex_idx(0, 0); id_x >= x_lo; id_x--)
    {   
      if (collide == true) 
        break;
      
      for (id_y = vertex_idx(0, 1); id_y <= vertex_idx(3, 1); id_y++) // P4P3
      {
        if (collide == true) 
          break;
        unsigned int temp1, temp2;
        if (!costmap->worldToMap(id_x, id_y, temp1, temp2))
          ;
          // std::cout << "世界坐标超出的地图边界5" << std::endl;
        tem_pos[0] = temp1;
        tem_pos[1] = temp2;
        if (HasObstacle( (int64_t)id_x, (int64_t)id_y)) // the voxel is occupied
        {   
          collide = true;
          break;
        }
      }
    }

    if (collide)
    {
      vertex_idx(0, 0) = std::min(id_x+2, vertex_idx(0, 0)); 
      vertex_idx(3, 0) = std::min(id_x+2, vertex_idx(3, 0)); 
    }
    else
      vertex_idx(0, 0) = vertex_idx(3, 0) = id_x + 1;

    if (vertex_idx_lst == vertex_idx) // 膨胀one step,前后无变化,达到最大状态,跳出循环
      break;

    vertex_idx_lst = vertex_idx;

    /* 将长方形的四个顶点从栅格坐标转成世界坐标 */
    Eigen::MatrixXd vertex_coord = Eigen::MatrixXd::Zero(8, 3);
    for (int i = 0; i < 4; i++)
    {   
      int index_x = std::max(std::min(vertex_idx(i, 0), static_cast<int>(costmap->getSizeInCellsX()) - 1), 0);  // 这里为什么是_max_x_id-1和0? --control point size
      int index_y = std::max(std::min(vertex_idx(i, 1), static_cast<int>(costmap->getSizeInCellsY()) - 1), 0);

      Eigen::Vector2d pos;
      costmap->mapToWorld(index_x, index_y, pos[0], pos[1]);
      vertex_coord.row(i).head(2) = pos;
    }

    // 使用vertex_idx继续迭代, 这里vertex_coord只是为了计算cubeMax(每次迭代后的cube)
    cubeMax.setVertex(vertex_coord, costmap->getResolution()); // 将从collision_map->GridIndexToLocation(index)获得的顶点pos放入栅格中心???????????
    if (isContains(lstcube, cubeMax)) // 剪枝
      return std::make_pair(lstcube, false);

    iter ++;
  }

  return std::make_pair(cubeMax, true);   // 膨胀前后无变化,则达到最大状态    
}

bool HybridAStarPlanner::HasObstacle(const int grid_index_x, const int grid_index_y) const 
{
  /* 检查索引是否在地图范围内 */
  if (grid_index_x < 0 || grid_index_x >= static_cast<int>(costmap->getSizeInCellsX()) ||
      grid_index_y < 0 || grid_index_y >= static_cast<int>(costmap->getSizeInCellsY())) 
  {
    return false; // 索引超出范围
  }
  /* 获取网格的代价值 */
  unsigned char cost = costmap->getCost(static_cast<unsigned int>(grid_index_x), static_cast<unsigned int>(grid_index_y));

  return (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE); // 判断是否是障碍物
  // costmap_2d::INSCRIBED_INFLATED_OBSTACLE (253)：表示膨胀区域
  // costmap_2d::LETHAL_OBSTACLE (254)：表示致命障碍物
} /* end of HasObstacle */

bool HybridAStarPlanner::isContains(const Cube &cube1, const Cube &cube2)
{
  if( (cube1.vertex(1, 0) >= cube2.vertex(1, 0)) && (cube1.vertex(1, 1) <= cube2.vertex(1, 1)) &&
      (cube1.vertex(3, 0) <= cube2.vertex(3, 0)) && (cube1.vertex(3, 1) >= cube2.vertex(3, 1)) )
    return true;
  else
    return false; 
} /* end of isContains */

void HybridAStarPlanner::timeAllocation(std::vector<Cube>& corridor, const Vec3d& start_pt_, const Vec3d& end_pt_)
{
  /* points 收集路径的所有关键点：
      起点 start_pt_。
      每个 corridor 的中心点（除第一个 cube，因为它表示起点）。
      终点 end_pt_。 */
  std::vector<Vec3d> points;
  points.push_back(start_pt_);
  // 计算出的corridor的特点: 第一个cube的边界点为第二个cube的center
  for (int i = 1; i < (int)corridor.size(); i++)
    points.push_back(corridor[i].center); // 每个 corridor 的中心点（除第一个 cube，因为它表示起点）
  points.push_back(end_pt_);

  double _Vel = Constants::MAX_Vel * 0.6; // 设置额定速度
  double _Acc = Constants::MAX_Acc * 0.6; // 设置额定加速度

  for (int k = 0; k < (int)points.size(); k++)
  {
    double dtxyz;
    Vec3d p0 = points[k]; // 当前点
    Vec3d p1 = points[k + 1]; // 下一个点
    Vec3d d = p1 - p0; // 从 当前点指向下一个点的向量
    Vec3d v0(0.0, 0.0, 0.0);

    // if (k == 0)
    //     v0 = _start_vel;    // _start_vel从odom中获得,为起始点的速度

    double D = d.norm();    // 相邻两点的距离（标量）
    double V0 = v0.dot(d / D);  // V0的含义???  V0 > 0:速度方向与目标点方向相同, V0 < 0:速度方向与目标点方向相反
    double aV0 = fabs(V0);

    double acct = (_Vel - V0) / _Acc * ((_Vel > V0) ? 1 : -1);  // 加速时间（从现速度加速到额定速度）
    double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0) ? 1 : -1);  // 加速位移 v*t+0.5*a*t^2(公式)
    double dcct = _Vel / _Acc;  // 减速时间（从额定速度减速到0）
    double dccd = _Acc * dcct * dcct / 2;  // 减速位移

    if (D < aV0 * aV0 / (2 * _Acc))    // 两点之间距离小于加速距离, 这行写错了吧???, 测试结果:一直不执行
    {
        double t1 = (V0 < 0) ? 2.0 * aV0 / _Acc : 0.0;
        double t2 = aV0 / _Acc;
        dtxyz = t1 + t2;
    }
    else if (D < accd + dccd)    // 两点之间距离小于加速距离+减速距离
    {
        double t1 = (V0 < 0) ? 2.0 * aV0 / _Acc : 0.0;
        double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
        double t3 = (aV0 + _Acc * t2) / _Acc;
        dtxyz = t1 + t2 + t3;  
    }
    else    // 正常情况,两点之间距离=加速距离+匀速距离+减速距离
    {
        double t1 = acct;
        double t2 = (D - accd - dccd) / _Vel;
        double t3 = dcct;
        dtxyz = t1 + t2 + t3;
    }
    corridor[k].t = dtxyz;  // 一共points.size()-1条轨迹
  }
}

// void HybridAStarPlanner::ConnectCorridors(std::vector<std::pair<VecCube, VecCube>>& cs, VecCube& connected_cs)
void HybridAStarPlanner::ConnectCorridors(std::vector<VecCube>& cs, VecCube& connected_cs)
{
  connected_cs.clear();

  for (const auto& corridor : cs)
  {
    // const auto& second_part = corridor.second;
    const auto& second_part = corridor;
    connected_cs.insert(connected_cs.end(), second_part.begin(), second_part.end()); // todo
  }
}

void HybridAStarPlanner::PublishCorridor(const std::vector<Cube> &corridor)
{
  for (auto & mk: corridor_array.markers) 
      mk.action = visualization_msgs::Marker::DELETE;  // 删除上一次的cube  // 删除上一次的cube
  corridor_pub_.publish(corridor_array); // todo

  corridor_array.markers.clear();  // 和DELETE操作重复

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.ns = "corridor";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.a = 0.2; // 0.08
  mk.color.r = 0.0;
  mk.color.g = 1.0;
  mk.color.b = 1.0;

  int idx = 0;
  for(int i = 0; i < int(corridor.size()); i++)
  // for(int i = 0; i < 1; i++)
  {   
    mk.id = idx;

    mk.pose.position.x = (corridor[i].vertex(1, 0) + corridor[i].vertex(0, 0) ) / 2.0; 
    mk.pose.position.y = (corridor[i].vertex(1, 1) + corridor[i].vertex(2, 1) ) / 2.0; 
    mk.pose.position.z = 0.0;   // 二维

    mk.scale.x = abs((corridor[i].vertex(1, 0) - corridor[i].vertex(0, 0) ));
    mk.scale.y = abs((corridor[i].vertex(2, 1) - corridor[i].vertex(1, 1) ));

    mk.scale.z = 0.1; 

    idx ++;
    corridor_array.markers.emplace_back(mk);
  }

  corridor_pub_.publish(corridor_array);
} /* end of PublishCorridor */

void HybridAStarPlanner::getSample(const HybridAStartResult& trajectory, 
                                  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>* point_set, 
                                  int sample_count, 
                                  double& start_angle, 
                                  double& end_angle, 
                                  double& total_distance)
{
  if (trajectory.x.size() == 4) 
    sample_count = 1;

  /* 得到抽样路径 */
  size_t j = 0;
  while ((j + 1) < trajectory.x.size()) 
  {
    point_set->emplace_back(trajectory.x[j], trajectory.y[j], trajectory.phi[j]);
    j += sample_count;
  }
  if ((j + 1) != trajectory.x.size()) 
    point_set->emplace_back(trajectory.x.back(),trajectory.y.back(), trajectory.phi.back()); // 插入轨迹尾

  /* 计算路径总长度 */
  // double sum = 0;
  // for (size_t j = 1; j < point_set->size(); ++j) // 从第二个轨迹点开始遍历
  // {
  //   sum += (point_set->at(j).head(2) - point_set->at(j - 1).head(2)).norm();
  // }
  //一段路径出入射角度保持一致
  //CheckGear()函数在PathSmoothAlgorithm()中调用，如果没有调用PathSmoothAlgorithm()则gear为初始值！！！！
  /* 得到一段路径的起始角度和终点角度 */
  gear_ = CheckGear(trajectory);
  start_angle = gear_.first ? point_set->front()(2) : common::math::NormalizeAngle(point_set->front()(2) + M_PI);
  end_angle = gear_.second ? point_set->back()(2) : common::math::NormalizeAngle(point_set->back()(2) + M_PI);
  /* 得到一段路径的从起点到终点的直线位移 */
  total_distance = (point_set->back().head(2) - point_set->front().head(2)).norm();
  std::cout << "一段路径的从起点到终点的直线位移(s): "<< total_distance << std::endl;
}

std::pair<bool,bool> HybridAStarPlanner::CheckGear(const struct HybridAStartResult &trajectory) 
{
  if(trajectory.x.size() < 2) 
  {
    return {false, false};
  }
  double init_heading_angle = trajectory.phi[0];
  const common::math::Vec2d init_tracking_vector(trajectory.x[1] - trajectory.x[0],
                                                 trajectory.y[1] - trajectory.y[0]);
  double init_tracking_angle = init_tracking_vector.Angle();

  int n  = trajectory.phi.size();
  double end_heading_angle = trajectory.phi[n-1];
  const common::math::Vec2d end_tracking_vector(trajectory.x[n-1] - trajectory.x[n-2],
                                                trajectory.y[n-1] - trajectory.y[n-2]);
  double end_tracking_angle = end_tracking_vector.Angle();
  bool ini_gear = std::abs(common::math::NormalizeAngle(init_tracking_angle - init_heading_angle)) <
          M_PI_2;
  bool end_gear = std::abs(common::math::NormalizeAngle(end_tracking_angle - end_heading_angle)) <
          M_PI_2;

  return {ini_gear, end_gear};
}

void HybridAStarPlanner::publishPlan_bspline(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path) 
{
  nav_msgs::Path nav_path;
  geometry_msgs::PoseStamped pose_stamped;

  for (size_t index = 0; index < path.size(); ++index) {
    const auto &pose= path[index];
    double head_angle = path[index].z();
    double tracking_angle;
    if (index + 1 < path.size()) {
      double diff_x = path[index+1].x() - path[index].x();
      double diff_y = path[index+1].y() - path[index].y();
      tracking_angle = std::atan2(diff_y, diff_x);
    } else {
      if (nav_path.poses.empty()) {
        tracking_angle = head_angle;
      } else {
        tracking_angle = nav_path.poses.back().pose.position.z;
      }
    }
    int gear = 
      std::abs(common::math::NormalizeAngle(tracking_angle - head_angle)) <
      (M_PI_2)? 1 : 0;
    
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = pose.x();
    pose_stamped.pose.position.y = pose.y();
    pose_stamped.pose.position.z = gear;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());//创建四元数

    nav_path.poses.emplace_back(pose_stamped);
  }

  nav_path.header.frame_id = "map";
  nav_path.header.stamp = ros::Time::now();

  plan_pub_bspline.publish(nav_path);
}

void HybridAStarPlanner::publishPathFromCtrlPts(const Eigen::MatrixXd& ctrl_pts) 
{
    // 初始化 Path 消息
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    // 提取控制点并添加到 Path 消息中
    for (int i = 0; i < ctrl_pts.cols(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        // 从控制点矩阵提取坐标
        pose.pose.position.x = ctrl_pts(0, i);
        pose.pose.position.y = ctrl_pts(1, i);
        if (ctrl_pts.rows() > 2) {
            pose.pose.position.z = ctrl_pts(2, i); // 如果有 z 坐标，添加 z 值
        } else {
            pose.pose.position.z = 0.0; // 默认为 0
        }

        pose.pose.orientation.w = 1.0; // 默认姿态为单位四元数

        path_msg.poses.push_back(pose);
    }

    // 发布 Path 消息
    path_pub.publish(path_msg);
}

} /* namespace hybrid_astar_planner */

