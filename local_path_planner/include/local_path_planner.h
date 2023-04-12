#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "point.h"

using namespace std;

namespace local{

class LocalPathPlanner : public nav_core::BaseLocalPlanner{
public:

    LocalPathPlanner();
    LocalPathPlanner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~LocalPathPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    double computeAngleBetweenPoints(pnt::Point first, pnt::Point second);

    pnt::Point getSubgoal();

    pnt::Point getCurrentPosition();

    bool isGoalReached();

    private:
        
        double PI;
        bool initialized_;

        int index_subgoal;
        int goal_reached;
        int threshold_angle;
              
        float angular_velocity;
        float linear_velocity;
        float max_linear_velocity;
        float max_angular_velocity;

        vector<geometry_msgs::PoseStamped> global_plan;

        costmap_2d::Costmap2DROS* costmap_ros_;

        tf2_ros::Buffer* tf_;

        boost::mutex m_odometry_mutex;
        nav_msgs::Odometry::ConstPtr m_odometry;

        ros::Subscriber m_odom_sub;
};
};

#endif