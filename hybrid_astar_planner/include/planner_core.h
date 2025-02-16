#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
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
 *********************************************************************/
// #define POT_HIGH 1.0e10        // unassigned cell potential
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include "node2d.h"
#include "node3d.h"
#include "timer.h"
#include "type.h"
#include <tf/tf.h>
#include <costmap_2d/cost_values.h>
#include "constants.h"
#include "plan_container.hpp"
// #include <bspline_opt/uniform_bspline.h>
// #include "bspline_opt/bspline_optimizer.h"
#include <bspline_opt/uniform_bspline.h>
#include "bspline_opt/bspline_optimizer.h"


namespace hybrid_astar_planner {
//类插件实现的多态的时候，若在头文件中定义了函数，那么就必须有这个函数的实现，否则会报错！！！
/**
 * @class HybridAStarPlanner
 * @brief Provides a ROS wrapper for the HybridAStarPlanner planner which runs a fast, interpolated navigation function on a costmap.
 */

class HybridAStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the HybridAStarPlanner object
         */
        HybridAStarPlanner();

        /**
         * @brief  Default deconstructor for the HybridAStarPlanner object
         */
        ~HybridAStarPlanner();

        /**
         * @brief  Initialization function for the HybridAStarPlanner
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning.And the costmap is get from the topic name /map
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);
   
        /**
         * @brief Publish the plan to RVIZ 
         * @param path the vector contain the path
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
         * @brief Publish the path node to RVIZ 
         * @param path the vector contain the path
         */
        void publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
         * @brief The call back function of makeplan service
         * @param req 
         * @param resp the plan the planner made
         * @return True if a valid plan was found, false otherwise
        */
        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
    protected:

        bool initialized_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        ros::Publisher path_vehicles_pub_;//用于在路径上发布车子位置
        ros::Publisher path_pub;
        ros::Publisher corridor_pub_;
        ros::Publisher plan_pub_bspline;
        costmap_2d::Costmap2D* costmap;
        
    private:
        /**
         * @brief Check whethe the start pose is available
         * @param start A reference to start pose
         * @return True if the start pose is available
        */
        bool checkStartPose(const geometry_msgs::PoseStamped &start);

        /**
         * @brief Check whethe the goal pose is available
         * @param goal A reference to goal pose
         * @return True if the goal pose is available
        */
        bool checkgoalPose(const geometry_msgs::PoseStamped &goal);

        /**
         * @brief Check whethe the goal pose is available
         * @param goal A reference to goal pose
         * @return True if the goal pose is available
        */       
        bool TrajectoryPartition(const HybridAStartResult& result, std::vector<HybridAStartResult>* partitioned_result);

        void DataTransform(std::vector<geometry_msgs::PoseStamped>& plan, HybridAStartResult* result);

        /**
         * @brief 根据result中的静态信息x,y,phi，利用相邻点、逐点求动态信息v,a,steer
         * @param result 全局路径的一个分段
         * @return 如果成功求解动态信息返回 True
        */       
        bool GenerateSpeedAcceleration(HybridAStartResult* result);

        /**
         * @brief 为每段轨迹生成安全走廊
         * @param path_coord 轨迹段
         * @param segment 轨迹编号
         * @return 安全走廊
        */      
        VecCube corridorGeneration(const VectorVec3d &path_coord, int segment);

        /**
         * @brief 根据一个点生成一个长方形，但这个长方形还是一个点，只是长方形的四个点重合到了一块
         * @param pt 一个点 {x, y, 0}，属全局坐标
         * @return 平面长方形
        */     
        Cube generateCube(Vec3d pt);

        /**
         * @brief 检查一个长方形是否在另一个长方形内
         * @param cube 被检查的长方形
         * @param lstcube 用于检查的长方形
         * @return 平面长方形
        */
        std::pair<Cube, bool> inflateCube(const Cube &cube, const Cube &lstcube);

        bool HasObstacle(const int grid_index_x, const int grid_index_y) const;

        bool isContains(const Cube &cube1, const Cube &cube2);

        void timeAllocation(std::vector<Cube> & corridor, const Vec3d& start_pt_, const Vec3d& end_pt_);

        // void ConnectCorridors(std::vector<std::pair<VecCube, VecCube>>& cs, VecCube& connected_cs);
        void ConnectCorridors(std::vector<VecCube>& cs, VecCube& connected_cs);

        void PublishCorridor(const std::vector<Cube> &corridor);

        void getSample(const HybridAStartResult& trajectory,
                vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>* point_set,  
                int sample_count, double& start_angle, double& end_angle, double& total_distance);

        std::pair<bool,bool> CheckGear(const struct HybridAStartResult &trajectory);

        void publishPlan_bspline(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path);
        void publishPathFromCtrlPts(const Eigen::MatrixXd& ctrl_pts);

        /* 内部变量 */
        visualization_msgs::MarkerArray corridor_array;
        double resolution;
        ros::ServiceServer make_plan_srv_;
        bool use_hybrid_astar;
        std::vector<HybridAStartResult> partition_trajectories;
        // std::vector<std::pair<VecCube, VecCube>> corridors;
     
        double delta_t_ = 0.0;
        std::pair<bool, bool> gear_ = {false, false};
        opt_planner::BsplineOptimizer::Ptr bspline_optimizer_rebound_;
        // PlanParameters pp_; // 统一管理参数的结构体
};


} //end namespace hybrid_astar_planner

#endif
