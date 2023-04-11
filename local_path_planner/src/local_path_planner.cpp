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
    
    angular_velocity=0;
    linear_velocity=0;
    this->index_subgoal=1;
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
    //boost::mutex::scoped_lock lock(m_odometry_mutex);
    tf2::Transform local_pose;
	//tf2::fromMsg(m_odometry->pose.pose, local_pose);
    tf2::Quaternion quad;

   

    double quatx= m_odometry->pose.pose.orientation.x;
    double quaty= m_odometry->pose.pose.orientation.y;
    double quatz= m_odometry->pose.pose.orientation.z;
    double quatw= m_odometry->pose.pose.orientation.w;
   
    double siny_cosp = 2 * (quatw * quatz + quatx * quaty);
    double cosy_cosp = 1 - 2 * (quaty * quaty + quatz * quatz);
    double yaw = std::atan2(siny_cosp, cosy_cosp) * 180/3.14159265359;
    double yaw360=yaw;
    if(yaw360<0)
        yaw360+=360;

    ROS_INFO("Yaw: [%f]",yaw);

    geometry_msgs::Twist cmd;

    pnt::Point subgoal=this->getSubgoal();
    pnt::Point current_position = this->getCurrentPosition();

    double angle= computeAngleBetweenPoints(current_position,subgoal);

    cout<<angle<<" "<<yaw360<<endl;
    if(abs(angle-yaw360) >=2)
    {
        if((int)(angle-yaw360+360)%360>180)
            angular_velocity=-0.20;
        else 
           angular_velocity=0.20;
    }
    else
        angular_velocity=0;
    if(angular_velocity==0)
        this->linear_velocity=0.20;

    if(pnt::euclidianDistanceSqrt(current_position,subgoal)<0.1)
    {
        linear_velocity=0;
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
    int len=global_plan.size();
    len--;

    pnt::Point goal(global_plan[len].pose.position.x, global_plan[len].pose.position.y);
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
    second=second-first;
    double angle= atan2(second.getY(),second.getX())* 180/3.14159265359;
    if(angle<0)
        angle+=360;
    //angle=(int)angle%360 + angle-(int)angle;
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
