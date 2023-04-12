#include "local_path_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local::LocalPathPlanner, nav_core::BaseLocalPlanner)

namespace local{

LocalPathPlanner::LocalPathPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

LocalPathPlanner::LocalPathPlanner(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

LocalPathPlanner::~LocalPathPlanner() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void LocalPathPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle nh;
	ros::NodeHandle private_nh("~/" + name);

    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
    ROS_INFO("RRT* Local Planner initialized successfully.");
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&LocalPathPlanner::odomCallback, this, _1));
    
    this->angular_velocity=0;
    this->linear_velocity=0;
    this->index_subgoal=1;
    this->max_linear_velocity=0.22;
    this->max_angular_velocity=2.00;
    this->threshold_angle=90;
    this->PI=3.14159265359;
}

bool LocalPathPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    if(this->global_plan!=orig_global_plan)
    {
        this->index_subgoal=1;
        this->global_plan=orig_global_plan;
    }

    return true;
}

bool LocalPathPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    

    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    double quatx;
    double quaty;
    double quatz;
    double quatw;
    double siny_cosp;
    double cosy_cosp;
    double yaw;
    double yaw360;
    double angle;
    int diference;

    tf2::Transform local_pose;
    tf2::Quaternion quad;

    pnt::Point subgoal;
    pnt::Point current_position;

    quatx= this->m_odometry->pose.pose.orientation.x;
    quaty= this->m_odometry->pose.pose.orientation.y;
    quatz= this->m_odometry->pose.pose.orientation.z;
    quatw= this->m_odometry->pose.pose.orientation.w;
   
    siny_cosp = 2 * (quatw * quatz + quatx * quaty);
    cosy_cosp = 1 - 2 * (quaty * quaty + quatz * quatz);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180/this->PI;
    yaw360=yaw;

    if(yaw360<0)
        yaw360+=360;

   // ROS_INFO("Yaw: [%f]",yaw);

    geometry_msgs::Twist cmd;

    subgoal=this->getSubgoal();
    current_position = this->getCurrentPosition();

    angle= computeAngleBetweenPoints(current_position,subgoal);

    diference=(int)(angle-yaw360+360) %360;
    
    if(diference<=180)
        angular_velocity=min(diference*this->max_angular_velocity/threshold_angle,this->max_angular_velocity);    
    if(diference>180)
    {
        diference=(int)(yaw360-angle+360)%360;
        angular_velocity=max(diference*-this->max_angular_velocity/threshold_angle,-this->max_angular_velocity);
    }
    linear_velocity=max((float)0,this->max_linear_velocity-(diference*this->max_linear_velocity/threshold_angle));
    cout<<"angle "<<angle<<" yaw360 "<<yaw360<<" angular "<<angular_velocity<<" lineal "<<linear_velocity<<" diference " <<diference<<endl;
    if(pnt::euclidianDistanceSqrt(current_position,subgoal)<0.1)
    {
        this->index_subgoal++;
        cout<<"current point"<<this->index_subgoal<<endl;
    }
        

    cmd.linear.x=linear_velocity;
    cmd.linear.y = 0;
	cmd.linear.z = 0;
	cmd.angular.x = 0;
	cmd.angular.y = 0;
	cmd.angular.z = angular_velocity;

    cmd_vel=cmd;

    return true;
}

bool LocalPathPlanner::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    int len;
    pnt::Point goal;

    len=global_plan.size()-1;

    goal=pnt::Point(global_plan[len].pose.position.x, global_plan[len].pose.position.y);

    if(pnt::euclidianDistanceSqrt(getCurrentPosition(),goal)<0.1)
        return true;
    return false;
}
void LocalPathPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);
	m_odometry = msg;
}
double LocalPathPlanner::computeAngleBetweenPoints(pnt::Point first, pnt::Point second)
{
    double angle;

    second=second-first;
    angle= atan2(second.getY(),second.getX())* 180/this->PI;

    if(angle<0)
        angle+=360;

    return angle;

}
pnt::Point LocalPathPlanner::getSubgoal()
{
    return pnt::Point(global_plan[this->index_subgoal].pose.position.x, global_plan[this->index_subgoal].pose.position.y);
}

pnt::Point LocalPathPlanner::getCurrentPosition()
{
    return pnt::Point(m_odometry->pose.pose.position.x,m_odometry->pose.pose.position.y);
}

}
