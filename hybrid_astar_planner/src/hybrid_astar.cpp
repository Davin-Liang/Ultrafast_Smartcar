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
 *  Email: dengpw@stu.xmu.edu.cn
 *********************************************************************/
#include "node3d.h"
#include <math.h>
#include "constants.h"
#include "hybrid_astar.h"
#include <tf/transform_datatypes.h>
#include "visualize.h"
// #define SearchingVisulize
// #define debug_mode
namespace hybrid_astar_planner {

// hybridAstar::hybridAstar(std::string frame_id, costmap_2d::Costmap2D* _costmap)
// :Expander(frame_id, _costmap) 
// {
//   int a = 1;
// }

bool hybridAstar::calculatePath(
    const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal,
    int cellsX, int cellsY, std::vector<geometry_msgs::PoseStamped>& plan,
    ros::Publisher& pub, visualization_msgs::MarkerArray& AstarpathNodes) 
{
  #ifdef debug_mode
  ros::Time t0 = ros::Time::now();
  #endif

  /* 初始化优先队列，未来改善混合A*算法性能，这里使用的是二项堆优先队列 */
  boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>> openSet;
  /* 初始化并创建一些参数 */
  const unsigned char* charMap = costmap->getCharMap(); // charMap中存储的是地图的障碍物信息
  unsigned int originalX, originalY, goalX, goalY;

  float g;

  unsigned int dx, dy;
  costmap->worldToMap(0, 0, dx, dy);
  dx = dx / 2.5;
  dy = dy / 2.5;
  
  float t;
  t = tf::getYaw(start.pose.orientation); // t为起始点朝向
  Node3D* startPose = new Node3D(start.pose.position.x, start.pose.position.y, t, 0, 0, false, nullptr);

  unsigned int goal_x, goal_y, start_x, start_y;
  costmap->worldToMap(goal.pose.position.x, 
                      goal.pose.position.y, goal_x, goal_y);
  costmap->worldToMap(start.pose.position.x, 
                      start.pose.position.y, start_x, start_y);
  //************************************************
  // 建立启发势场取决于地图大小,231(nodes/ms)
  std::unordered_map<int, std::shared_ptr<Node2D>> dp_map = 
      grid_a_star_heuristic_generator_->GenerateDpMap(goal_x, goal_y, costmap); //todo    

  #ifdef debug_mode
  ros::Time end_time = ros::Time::now();
  ros::Time t1 = ros::Time::now();
  ros::Duration d(end_time - t0);
  std::cout << "generate heuristic map costed " << d * 1000 << " ms" <<std::endl;
  #endif

  t = tf::getYaw(goal.pose.orientation);
  Node3D* goalPose = new Node3D(goal.pose.position.x, goal.pose.position.y, t, 999, 0, false, nullptr);
  std::unordered_map<int, Node3D*> open_set;
  std::unordered_map<int, Node3D*> closed_set;

  int dir;
  if (reverse_) dir = 6;
  else dir = 3;

  float resolution = 0.2;// 0.125
  int cells_x,cells_y;
  cells_x = cellsX / 2.5;
  cells_y = cellsY / 2.5;
  int iPred, iSucc;
  open_set.emplace(startPose->getindex(cells_x, Constants::headings, resolution, dx, dy), startPose);
  openSet.push(startPose);

  Node3D* tmpNode;
  Node3D* nSucc;
  int counter = 0;
  while (openSet.size() && counter < iterations_) 
  {
    ++ counter;
    tmpNode = openSet.top(); // 根据混合A*算法，取堆顶的元素作为下查找节点

    #ifdef SearchingVisulize
    publishSearchNodes(*tmpNode, pub, AstarpathNodes, counter);
    #endif

    openSet.pop(); // 出栈
    if (reachGoal(tmpNode, goalPose)) 
    {
      #ifdef debug_mode
      ros::Time t1 = ros::Time::now();
      ros::Duration d(t1 - t0);
      std::cout << "got plan in ms: " << d * 1000 << std::endl;
      #endif

      ROS_INFO("Got a plan,loop %d times", counter);
      nodeToPlan(tmpNode, plan);

      /* 清除指针 */
      // TODO:

      // if (startPose != nullptr)
      //   delete startPose;
      // if (goalPose != nullptr)
      //   delete goalPose;

      for (auto& pair : open_set) 
      {
        if (pair.second != nullptr)
          delete pair.second;
      }
      // for (auto& pair : closed_set) 
      // {
      //   if (pair.second != nullptr)
      //     delete pair.second;
      // }
      // while (!openSet.empty()) 
      // {
      //     Node3D* node = openSet.top();  // 获取堆顶元素
      //     openSet.pop();  // 弹出堆顶元素
      //     if (node != nullptr)
      //       delete node;  // 释放指针
      // }


      return true;
    }
    else 
    {
      if (Constants::dubinsShot && tmpNode->isInRange(*goalPose) && !tmpNode->isReverse()) 
      {
        nSucc = dubinsShot(*tmpNode, *goalPose, costmap);
        //如果Dubins方法能直接命中，即不需要进入Hybrid A*搜索了，直接返回结果
        if (nSucc != nullptr && reachGoal(nSucc, goalPose)) 
        {
          #ifdef debug_mode
          ros::Time t1 = ros::Time::now();
          ros::Duration d(t1 - t0);
          std::cout << "got plan in ms: " << d * 1000 << std::endl;
          #endif

          ROS_INFO("Got a plan,expored %d nodes ", counter);
          nodeToPlan(nSucc, plan);
          return true;//如果下一步是目标点，可以返回了
        }
      } 
      else if(reedsSheppShot_ && tmpNode->isInRange(*goalPose) && !tmpNode->isReverse()) 
      {
        nSucc = reedsSheppShot(*tmpNode, *goalPose, costmap, turning_radius_, ReedsSheppStepSize_);
        /* 如果Dubins方法能直接命中，即不需要进入Hybrid A*搜索了，直接返回结果 */
        if (nSucc != nullptr && reachGoal(nSucc, goalPose)) 
        {
          #ifdef debug_mode
          ros::Time t1 = ros::Time::now();
          ros::Duration d(t1 - t0);
          std::cout << "got plan in ms: " << d * 1000 << std::endl;
          #endif

          ROS_INFO("Got a plan,expored %d nodes ", counter);
          nodeToPlan(nSucc, plan);

          /* 清除指针 */
          // if (startPose != nullptr)
          //   delete startPose;
          // if (goalPose != nullptr)
          //   delete goalPose;

          for (auto& pair : open_set) 
          {
            if (pair.second != nullptr)
              delete pair.second;
          }
          // for (auto& pair : closed_set) 
          // {
          //   if (pair.second != nullptr)
          //     delete pair.second;
          // }
          // while (!openSet.empty()) 
          // {
          //     Node3D* node = openSet.top();  // 获取堆顶元素
          //     openSet.pop();  // 弹出堆顶元素
          //     if (node != nullptr)
          //       delete node;  // 释放指针
          // }


          return true; // 出口
        }
      }
    }

    /* 拓展tmpNode临时点目标周围的点 */
    std::vector<Node3D*> adjacentNodes = gatAdjacentPoints(dir, cellsX, cellsY, charMap, tmpNode);   
    /* 将 tmpNode点在pathNode3D中映射的点加入闭集合中 */
    // 根据坐标和方向生成唯一索引
    closed_set.emplace(tmpNode->getindex(cells_x ,Constants::headings, resolution, dx, dy), tmpNode); // 将 tmpNode 标记为已处理，防止重复扩展
    /* 逐个检查相邻节点是否需加入开放集合 */
    for (std::vector<Node3D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) 
    {
      // 使用stl标准库中的interator迭代器遍历相邻点
      Node3D* point = *it;
      iPred = point->getindex(cells_x, Constants::headings, resolution, dx, dy);

      /* 若相邻节点已在关闭集合中（已探索过），跳过处理 */
      if (closed_set.find(iPred)!= closed_set.end()) continue;
      g = point->calcG(); // 计算从起点到当前节点 point 的实际路径代价

      /* 处理已在开放集合中的节点 */
      // 若节点已在开放集合中且新代价更低，更新代价并重新入队
      if (open_set.find(iPred) != open_set.end()) 
      {
        if (g < open_set[iPred]->getG()) 
        {
          point->setG(g);
          open_set[iPred]->setG(g); //  // 更新开放集合中的节点代价
          openSet.push(point);    // 如果符合拓展点要求，则将此点加入优先队列中  
        }
      } 
      else 
      {
        point->setG(g);
        open_set.emplace(iPred, point);
        costmap->worldToMap(point->getX(), point->getY(), start_x, start_y);
        /* 计算启发式代价 h（如欧氏距离或势场值），此处使用预计算的 dp_map（Dijkstra势场）加速 */
        double dp_map_g_cost = 10000;
        if (dp_map.find(start_y * cellsX + start_x) != dp_map.end()) 
        {
          dp_map_g_cost=dp_map[start_y * cellsX + start_x]->getG()/20;
        }
        updateH(*point, *goalPose, NULL, NULL, cells_x, cells_y, dp_map_g_cost);
        openSet.push(point);    // 如果符合拓展点要求，则将此点加入优先队列中 
      }
    }
  }
  return false;
}

std::vector<Node3D*> hybridAstar::gatAdjacentPoints(int dir, int cells_x, 
    int cells_y, const unsigned char* charMap, Node3D *point) 
{
  std::vector<Node3D*> adjacentNodes;
  Node3D *tmpPtr;
  float xSucc;
  float ySucc;
  unsigned int startX, startY;
  // double move_step_size_ = 0.05;
  // int index;
  // unsigned int u32_x = int(x);
  // unsigned int u32_y = int(y);

  for (int i = 0; i < dir; i++) 
  {
    double x = point->getX();
    double y = point->getY();
    double t = point->getT();   
    for (int j = 1; j <= segment_length_discrete_num_; j++) // 
    {
      /* 前向拓展 */
      if (i < 3) 
      {
        double phi = 0.0;
        if (i == 0) phi = steering_angle_ * 3.14 / 180.0;
        if (i == 1) phi = 0.0;
        if (i == 2) phi = -steering_angle_ * 3.14 / 180.0;
        // xSucc = x + Constants::dx[i] * cos(t) - Constants::dy[i] * sin(t);
        // ySucc = y + Constants::dx[i] * sin(t) + Constants::dy[i] * cos(t);
        DynamicModel(move_step_size_, phi, x, y, t);
      } 
      else /* 后向拓展 */
      {
        double phi = 0.0;
        if (i == 3) phi = steering_angle_ * 3.14 / 180.0;
        if (i == 4) phi = 0.0;
        if (i == 5) phi = -steering_angle_ * 3.14 / 180.0;
        // xSucc = x - Constants::dx[i - 3] * cos(t) - Constants::dy[i - 3] * sin(t);
        // ySucc = y - Constants::dx[i - 3] * sin(t) + Constants::dy[i - 3] * cos(t);
        DynamicModel(-move_step_size_, phi, x, y, t);
      }
      /* 可行性检测 */
      if (costmap->worldToMap(x, y, startX, startY)) 
      {
        if (charMap[startX + startY * cells_x] < 250) 
        {
          if (i < 3) 
          {
            tmpPtr = new Node3D(x, y, t, 999, 0, false, point);
            tmpPtr->setCost(charMap[startX + startY * cells_x]);
          } 
          else 
          {
            tmpPtr = new Node3D(x, y, t, 999, 0, true, point);
            tmpPtr->setCost(charMap[startX + startY * cells_x]);
          }
          adjacentNodes.push_back(tmpPtr);
        }
        else
          break;
      }
      else
        break;
    }
  }

  return adjacentNodes;
}

void hybridAstar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const 
{
  x = x + step_size * std::cos(theta);
  y = y + step_size * std::sin(theta);
  theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi)); //TODO:
}

double hybridAstar::Mod2Pi(const double &x) 
{
    double v = fmod(x, 2 * M_PI);
  /*θ= 
        θ+2π, −2π<θ<−π
        θ−2π, π≤θ<2π
        θ, −π≤θ<π
        而且C++中的反三角函数给出的结果映射到了[-pi, pi]
 */
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    
    return v;
}

/* 判断当前节点 node 是否在位置（X, Y）和方向（T）上足够接近目标节点 goalPose，从而判定是否到达目标 */
bool hybridAstar::reachGoal(Node3D* node, Node3D* goalPose) 
{
  float nodeX = node->getX();
  float nodeY = node->getY();
  float goalX = goalPose->getX();
  float goalY = goalPose->getY();

  /* 位置判断 */
  if ((nodeX < (goalX + point_accuracy) && nodeX > (goalX - point_accuracy)) && // 检查 node 的 X 坐标是否在目标 X 坐标的 ±point_accuracy 范围内
      (nodeY < (goalY + point_accuracy) && nodeY > (goalY - point_accuracy)) ) // 检查 node 的 Y 坐标是否在目标 Y 坐标的 ±point_accuracy 范围内
  {
    /* 方向判断 */
    // 检查 node 的方向角 T 是否在目标方向角 ±theta_accuracy 范围内
    if (node->getT()  < (goalPose->getT()+theta_accuracy )&& 
        node->getT()  > (goalPose->getT()-theta_accuracy )) 
      return true;
  }
  return false;
}

int hybridAstar::calcIndix(float x, float y, int cells_x, float t) {
    return (int(x) * cells_x + int(y)) * Constants::headings + int(t / Constants::headings);
}

/* 将混合 A* 算法生成的路径节点链表（从目标节点回溯到起点）转换为 geometry_msgs::PoseStamped 类型的路径序列，
    并按 从起点到目标 的正确顺序存储在 plan 中，用于路径规划结果的可视化或执行 */
void hybridAstar::nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan) 
{
  Node3D* tmpPtr = node;
  geometry_msgs::PoseStamped tmpPose;
  std::vector<geometry_msgs::PoseStamped> replan;

  tmpPose.header.stamp = ros::Time::now();   

  /* 从目标节点（node）反向回溯到起点 */
  while (tmpPtr != nullptr) // 遍历直到链表头部（起点）
  {
    tmpPose.pose.position.x = tmpPtr->getX();
    tmpPose.pose.position.y = tmpPtr->getY();
    tmpPose.header.frame_id = frame_id_;
    tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(tmpPtr->getT());
    replan.push_back(tmpPose);
    tmpPtr = tmpPtr->getPerd(); // 转移到父节点
  }

  /* 路径顺序反转 */
  int size = replan.size();
  for (int i = 0; i < size; ++i) 
  {
    plan.push_back(replan[size-i-1]);
  }
}

}//end of namespace hybrid_astar_planner
