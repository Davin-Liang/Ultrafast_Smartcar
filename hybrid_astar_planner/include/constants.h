#ifndef _CONSTANTS
#define _CONSTANTS

#include <cmath>
#include "plan_container.hpp"

namespace hybrid_astar_planner {

namespace Constants {
/// A flag to toggle reversing (true = on; false = off)
/// 设置是否允许车辆后退的标志位 true表示可以倒退；false表示只能前进不能倒退   
static const bool reverse = true;  

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
/// 最大迭代次数
static const int iterations = 30000; 

/// [m] --- Uniformly adds a padding around the vehicle
/// 膨胀范围
static const double bloating = 0; 

/// [m] --- The width of the vehicle
static const double width = 0.18 + 2 * bloating;//车的宽度

/// [m] --- The length of the vehicle
static const double length = 0.3 + 2 * bloating;//车的长度

/*
* 车模需要转弯半径为0.75米的
* 车身长度需要0.15m(长) * 0.16m(轮宽)
*/
/// [m] --- the Minimum turning radius 车辆最小转弯半径 
static const double r = 1.0;

/// [m] --- The number of discretizations in heading
/// 车体朝向的离散数量
// static const int headings = 72;
static const int headings = 72;
// const float dy[] = { 0,        -0.0415893,  0.0415893};
// const float dx[] = { 0.7068582,   0.705224,   0.705224};
// const float dt[] = { 0,         0.1178097,   -0.1178097};

/* Constants::dx、dy、dt 定义了车辆的 离散化运动模型，模拟车辆在不同转向角度下的运动步长 */
const float dy[] = { 0,        -0.005198,  0.005198}; // 车辆在 局部坐标系 下沿车头方向（x轴）的位移增量（单位：米）
const float dx[] = { 0.0883573,   0.088153,   0.088153}; // 车辆在 局部坐标系 下横向（y轴）的位移增量（由转向引起）
const float dt[] = { 0,         0.1178097,   -0.1178097}; // 车辆航向角（t）的变化量（单位：弧度），即转向导致的方向变化
/// [°] --- The discretization value of the heading (goal condition)
/// 朝向离散度数(以度表示)
static const float deltaHeadingDeg = 360 / (float)headings; 

/// [c*M_PI] --- The discretization value of heading (goal condition)
// static const float deltaHeadingRad = 2 * M_PI / (float)headings; //朝向离散步长(以弧度表示)
static const float deltaHeadingRad = 2 * M_PI / 80.0; //朝向离散步长(以弧度表示)

/// [c*M_PI] --- The heading part of the goal condition 
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;

/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = false; //切换Dubin路径的开关

/// A flag to toggle the connection of the path via reedsSheppShot (true = on; false = off)
static const bool reedsSheppShot = true; //切换Dubin路径的开关

/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
static const bool dubins = false;//Dubin路径的切换开关: 若车子可以倒退，值为false
// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
// 用启发式的方法确定具有障碍的完整函数的可容许性
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
// 转弯时的移动代价惩罚
static const float penaltyTurning = 1.2; // 1.05
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
// 反转时的移动代价惩罚
static const float penaltyReversing = 1.5; // 1.5
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
// 改变方向的移动代价惩罚
static const float penaltyCOD = 1.7; //1.5
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
static const float dubinsStepSize = 0.088;

/* cube and inflate */
static const int max_inflate_iter = 10; // 膨胀迭代次数
static const int step_length = 1; // 安全走廊向外膨胀步长
static const double MAX_Vel = 1.0; // 最大速度
static const double MAX_Acc = 1.0; // 最大加速度

static const double wheel_base = 0.18; // 前后轮之车轮轴距离
static const double move_step_size = 0.05; // 离散路径点之间的步长

static const double max_jerk = 2.0; // 最大加加速度
static const double ctrl_pt_dist = 0.08; // 相邻控制点的距离
static const double feasibility_tolerance = 0.02;
// static const opt_planner::PlanParameters pp = //(2.0, 1.0, 4.0, 0.2, 0.05, 0.0, 0.0, 0.0, 0.0);
// {
//     2.0,   // max_vel_ 速度约束
//     1.0,   // max_acc_ 加速度约束
//     4.0,   // max_jerk_ 加加速度约束
//     0.2,   // ctrl_pt_dist 相邻控制点的距离
//     0.05,   // feasibility_tolerance_
//     0.0,  // planning_horizen_
//     0.0,   // time_search_
//     0.0,   // time_optimize_
//     0.0    // time_adjust_  
// };

}


}

#endif