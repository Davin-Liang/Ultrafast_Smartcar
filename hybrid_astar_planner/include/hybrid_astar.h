#ifndef _HYBRID_ASTAR_H
#define _HYBRID_ASTAR_H

#include <vector>
#include "algorithm.h"
#include "expander.h"
#include "node3d.h"
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>
// #define TEST
#define point_accuracy 0.5
#define theta_accuracy 2
namespace hybrid_astar_planner {

class hybridAstar : public Expander
{

    public:
    /**
     * @brief  Default constructor for the HybridAStarPlanner object
    */
    hybridAstar(std::string frame_id, costmap_2d::Costmap2D* _costmap)
    :Expander(frame_id, _costmap) {
        // double my_param = 0.0;
        ros::NodeHandle nh("~/");
        nh.getParam("UfsPlaner/reverse", reverse_);
        nh.getParam("UfsPlaner/iterations", iterations_);
        nh.getParam("UfsPlaner/move_step_size", move_step_size_);
        nh.getParam("UfsPlaner/segment_length_discrete_num", segment_length_discrete_num_);
        nh.getParam("UfsPlaner/steering_angle", steering_angle_);
        nh.getParam("UfsPlaner/wheel_base", wheel_base_);
        nh.getParam("UfsPlaner/turning_radius", turning_radius_);
        nh.getParam("UfsPlaner/ReedsSheppStepSize", ReedsSheppStepSize_);
        nh.getParam("UfsPlaner/reedsSheppShot", reedsSheppShot_);
        nh.getParam("UfsPlaner/dubinsShotDistance", dubinsShotDistance_);
        nh.getParam("UfsPlaner/heading", heading_);
        deltaHeadingRad_ = 2 * 3.14 / (double)heading_;


        // std::cout << "my_param = " << my_param << std::endl;
    }

    /**
     * @brief Find the path between the start pose and goal pose
     * @param start the reference of start pose 
     * @param goal the reference of goal pose 
     * @param cells_x the number of the cells of the costmap in x axis
     * @param cells_y the number of the cells of the costmap in y axis
     * @param plan the refrence of plan;
     * @return true if a valid plan was found.
    */
    bool calculatePath(
        const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal,
        int cellsX, int cellsY, std::vector<geometry_msgs::PoseStamped>& plan,
        ros::Publisher& pub, visualization_msgs::MarkerArray& pathNodes);
    
    /**
     * @brief Default deconstructor for the HybridAStarPlanner object
    */
    ~hybridAstar(){ }

    private:
 
    /**
     * @brief Get the adjacent pose of a given pose
     * @param cells_x the number of the cells of the costmap in x axis
     * @param cells_y the number of the cells of the costmap in y axis
     * @param charMap 
    */
    std::vector<Node3D*> gatAdjacentPoints(int dir, int cells_x, int cells_y, const unsigned char* charMap, Node3D *point);

    /**
     * @brief judge whether is reach the goal pose
     * @param node the refrence of the node
     * @param goalPose the goal of the planner
     * @return true if reach the goal
    */
    bool reachGoal(Node3D* node, Node3D* goalPose); 

    /**
     * @brief get the index of node
     * @param x the x position axis of the node
     * @param y the y position axis of the node
     * @param cells_x the scale of cells in x axis
     * @param t the depth of the 3D nodes
     * @return the index of node 
    */
    int calcIndix(float x, float y, int cells_x, float t); 

    /**
     * @brief transform the 2Dnode to geometry_msgs::PoseStamped
     * @param node the ptr of node
     * @param plan the refrence of plan
    */
    void nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan);

    /*!
     * Simplified car model. Center of the rear axle
     * refer to: http://planning.cs.uiuc.edu/node658.html
     * @param step_size Length of discrete steps
     * @param phi Car steering angle
     * @param x Car position (world frame)
     * @param y Car position (world frame)
     * @param theta Car yaw (world frame)
     */
    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const;
    static inline double Mod2Pi(const double &x);


    std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;

    double move_step_size_ = 0.0;
    int segment_length_discrete_num_ = 0;
    double steering_angle_ = 0.0;
    double wheel_base_ = 0.0;
    double turning_radius_ = 0.0;
    double ReedsSheppStepSize_ = 0.0;
    double dubinsShotDistance_ = 0.0;
    bool reedsSheppShot_ = true;
    bool reverse_ = true;
    int iterations_ = 0;
    int heading_ = 0;
    double deltaHeadingRad_ = 0;
};




}//end of namespace hybrid_astar_planner

#endif //the end of astar.h