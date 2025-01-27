#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/thread/mutex.hpp>

enum State 
{
    WAITING_FOR_GOAL,
    REPLAN_PATH_AND_MONITORING_PATH,
    TRACKING_ABNORMAL,
    TRACKING_COMPLETE
};

class StateMachine 
{
public:
    StateMachine() 
    {
        // 初始化订阅者和发布者
        replan_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
        global_path_sub_ = nh_.subscribe("/move_base/HybridAStarPlanner/plan_bspline", 10, &StateMachine::globalPathCallback, this);
        tracking_status_sub_ = nh_.subscribe("tracking_status", 10, &StateMachine::trackingStatusCallback, this);
        target_sub_ = nh_.subscribe("move_base_simple/goal", 10, &StateMachine::targetCallback, this);
        costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 10, &StateMachine::costmapCallback, this); // 新增代价地图订阅

        // 定时器
        path_monitor_timer_ = nh_.createTimer(ros::Duration(0.1), &StateMachine::pathMonitorCallback, this);
        replan_timer_ = nh_.createTimer(ros::Duration(1.0), &StateMachine::replanTimerCallback, this);

        state_ = WAITING_FOR_GOAL;
    }

    void run() 
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher replan_pub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber tracking_status_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber costmap_sub_;  // 新增代价地图订阅者
    ros::Timer path_monitor_timer_;
    ros::Timer replan_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
    
    nav_msgs::Path current_global_path_;
    geometry_msgs::PoseStamped current_target_;
    State state_;

    costmap_2d::Costmap2D global_costmap_;  // 存储代价地图数据
    boost::mutex costmap_mutex_;            // 保证线程安全

    // 代价地图回调函数
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
    {
        // boost::mutex::scoped_lock lock(costmap_mutex_);
        
        // 初始化/更新代价地图
        global_costmap_.resizeMap(
            msg->info.width, 
            msg->info.height, 
            msg->info.resolution,
            msg->info.origin.position.x,
            msg->info.origin.position.y
        );
        
        // 复制代价值（假设OccupancyGrid的data范围已经映射到0-255）
        for (unsigned int y = 0; y < msg->info.height; ++y) 
        {
            for (unsigned int x = 0; x < msg->info.width; ++x) 
            {
                unsigned int index = y * msg->info.width + x;
                unsigned char cost = static_cast<unsigned char>(msg->data[index]);
                global_costmap_.setCost(x, y, cost);
            }
        }
        
        ROS_INFO("Costmap updated. Size: %dx%d", msg->info.width, msg->info.height);
    }

    // 其他回调函数保持不变...
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) 
    {
        current_global_path_ = *msg;
        ROS_INFO("Received global path.");
        state_ = REPLAN_PATH_AND_MONITORING_PATH;
    }

    void trackingStatusCallback(const std_msgs::Int32::ConstPtr& msg) 
    {
        if (msg->data == 0) 
        {
            ROS_WARN("Tracking abnormal detected. Switching to TRACKING_ABNORMAL.");
            state_ = TRACKING_ABNORMAL;
        }
        else if (msg->data == 1)
        {
            ROS_INFO("Tracking complete. Switching to TRACKING_COMPLETE.");
            state_ = TRACKING_COMPLETE;
        }
    }

    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_target_ = *msg;
        ROS_INFO("New target received.");
    }

    // 修改障碍物检测函数
    bool checkPathForObstacles() 
    {
        // boost::mutex::scoped_lock lock(costmap_mutex_);
        
        if (current_global_path_.poses.empty()) 
        {
            ROS_WARN("Global path not initialized.");
            return false;
        }

        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) 
        {
            ROS_WARN("Failed to get robot pose.");
            return false;
        }

        size_t start_index = findClosestPathPoint(robot_pose);
        size_t end_index = std::min(start_index + 100, current_global_path_.poses.size());
        
        for (size_t i = start_index; i < end_index; ++i) 
        {
            unsigned int mx, my;
            const auto& pose = current_global_path_.poses[i].pose.position;
            
            if (global_costmap_.worldToMap(pose.x, pose.y, mx, my)) 
            {
                unsigned char cost = global_costmap_.getCost(mx, my);
                // ROS_WARN("World (%.2f, %.2f) -> Map (%d, %d), Cost: %d", 
                //          pose.x, pose.y, mx, my, cost);
                
                if (cost >=99) 
                {
                    ROS_WARN("Obstacle detected on path!");
                    return true;
                }
            }
            else 
            {
                ROS_WARN("Coordinate (%.2f, %.2f) out of map bounds", pose.x, pose.y);
            }
        }
        return false;
    }

    // 其余工具函数保持不变...
    size_t findClosestPathPoint(const geometry_msgs::PoseStamped& robot_pose) 
    {
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = 0;

        for (size_t i = 0; i < current_global_path_.poses.size(); ++i) 
        {
            double dx = current_global_path_.poses[i].pose.position.x - robot_pose.pose.position.x;
            double dy = current_global_path_.poses[i].pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) 
            {
                min_distance = distance;
                closest_index = i;
            }
        }

        return closest_index;
    }

    bool getRobotPose(geometry_msgs::PoseStamped& robot_pose) 
    {
        try 
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_link", ros::Time(0), ros::Duration(1.0));
            tf2::doTransform(robot_pose, robot_pose, transform);
            return true;
        } 
        catch (tf2::TransformException& ex) 
        {
            ROS_WARN("Failed to get robot pose: %s", ex.what());
            return false;
        }
    }

    void pathMonitorCallback(const ros::TimerEvent&) 
    {
        if (state_ != REPLAN_PATH_AND_MONITORING_PATH) return;

        if (checkPathForObstacles()) 
        {
            state_ = TRACKING_COMPLETE;
            requestReplan();
        }
    }

    void requestReplan() 
    {
        replan_pub_.publish(current_target_);
        ROS_INFO("Replan request sent.");
    }

    void replanTimerCallback(const ros::TimerEvent&) 
    {
        if (state_ == REPLAN_PATH_AND_MONITORING_PATH || state_ == TRACKING_ABNORMAL) 
        {
            // 保留原有逻辑
        }
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_machine");
    StateMachine state_machine;
    state_machine.run();
    return 0;
}