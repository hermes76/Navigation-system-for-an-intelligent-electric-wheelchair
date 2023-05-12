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
   

    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
    ros::NodeHandle nh;
	ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle amcl_topic;


    private_nh.param("max_angular_velocity",   this->max_angular_velocity,(float)1.0);
    private_nh.param("max_linear_velocity",    this->max_linear_velocity,(float)0.11);
    private_nh.param("threshold_angle_angular",this->threshold_angle_angular, 90);
    private_nh.param("threshold_angle_linear", this->threshold_angle_linear, 90);
    private_nh.param("deceleration_linear",    this->deceleration_linear, (float)0.03);
    private_nh.param("deceleration_angular",   this->deceleration_angular, (float)0.03);


    this->angular_velocity=0;
    this->linear_velocity=0;
    this->index_subgoal=1;
    this->PI=3.14159265359;


    this->m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&LocalPathPlanner::odomCallback, this, _1));
    this->amcl_sub   = amcl_topic.subscribe("amcl_pose" , 100, &LocalPathPlanner::amclCallback, this);
    ROS_INFO("RRT* Local Planner initialized successfully.");

   
}

bool LocalPathPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    this->index_subgoal=1;
    this->global_plan=orig_global_plan;

    return true;
}

bool LocalPathPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    double angle;
    double yaw;

    int len;
    int diference;

    pnt::Point subgoal;
    pnt::Point current_position;
    pnt::Point goal;
    pnt::Point start;

    geometry_msgs::Twist cmd;


    len=global_plan.size()-1;

    goal=pnt::Point(global_plan[len].pose.position.x, global_plan[len].pose.position.y);
    start=pnt::Point(global_plan[0].pose.position.x, global_plan[0].pose.position.y);

    if(pnt::euclidianDistanceSqrt(start,goal)<0.01)
    {
        this->linear_velocity=max((float)0.0,this->linear_velocity   -deceleration_linear);
        this->angular_velocity=max((float)0.0,this->angular_velocity -deceleration_angular);

        cmd.linear.x=this->linear_velocity;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = this->angular_velocity;
        cmd.angular.y = 0;
        cmd.angular.z = 0;

        cmd_vel=cmd;
        return true;
    }

    current_position = this->getCurrentPosition();
    subgoal=this->getSubgoal();

    angle= computeAngleBetweenPoints(current_position,subgoal);
    yaw=EstimatedYaw();

    diference=(int)(angle-yaw+360) %360;
    
    if(diference<=180)
        angular_velocity=min(diference*this->max_angular_velocity/this->threshold_angle_angular,this->max_angular_velocity);    
    if(diference>180)
    {
        diference=(int)(yaw-angle+360)%360;
        angular_velocity=max(diference*-this->max_angular_velocity/this->threshold_angle_angular,-this->max_angular_velocity);
    }
    linear_velocity=max((float)0,this->max_linear_velocity-(diference*this->max_linear_velocity/this->threshold_angle_linear));
    //cout<<"angle "<<angle<<" yaw360 "<<yaw<<endl;//" current x "<<current_position.getX()<<" current y "<<current_position.getY()<<yaw<<" mx "<<estimated_x<<" my "<<estimated_y<<endl;
    if(pnt::euclidianDistanceSqrt(current_position,subgoal)<0.1)
    {
        if(this->global_plan.size()>this->index_subgoal)
            this->index_subgoal++;
        //cout<<"current point"<<this->index_subgoal<<endl;
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
    pnt::Point start;

    len=global_plan.size()-1;

    goal= pnt::Point(global_plan[len].pose.position.x, global_plan[len].pose.position.y);
    start=pnt::Point(global_plan[0].pose.position.x, global_plan[0].pose.position.y);
    if(pnt::euclidianDistanceSqrt(getCurrentPosition(),goal)<0.1
     && pnt::euclidianDistanceSqrt(start,goal)>0.01)
    {
        cout<<"YEY"<<endl;
        return true;
    }
    return false;
}
void LocalPathPlanner::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
{
    this->estimated_x=msg->pose.pose.position.x;
    this->estimated_y=msg->pose.pose.position.y;

    double quatx = msg->pose.pose.orientation.x;
    double quaty = msg->pose.pose.orientation.y;
    double quatz = msg->pose.pose.orientation.z;
    double quatw = msg->pose.pose.orientation.w;


    double siny_cosp;
    double cosy_cosp;
    double yaw;
    double yaw360;


    siny_cosp = 2 * (quatw * quatz + quatx * quaty);
    cosy_cosp = 1 - 2 * (quaty * quaty + quatz * quatz);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180/this->PI;
    yaw360=yaw;

    if(yaw360<0)
        yaw360+=360;
    
    this->estimated_yaw=yaw360;
    //this->currentCovariance = msg->covariance;
}
void LocalPathPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    this->odom_x=msg->pose.pose.position.x;
    this->odom_y=msg->pose.pose.position.y;

    double quatx = msg->pose.pose.orientation.x;
    double quaty = msg->pose.pose.orientation.y;
    double quatz = msg->pose.pose.orientation.z;
    double quatw = msg->pose.pose.orientation.w;


    double siny_cosp;
    double cosy_cosp;
    double yaw;
    double yaw360;


    siny_cosp = 2 * (quatw * quatz + quatx * quaty);
    cosy_cosp = 1 - 2 * (quaty * quaty + quatz * quatz);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180/this->PI;
    yaw360=yaw;

    if(yaw360<0)
        yaw360+=360;
    
    this->odom_yaw=yaw360;
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
    if(this->last_estimated_x!= this->estimated_x && this->last_estimated_y != this->estimated_y 
    && this->last_estimated_yaw != this->estimated_yaw)
    //if(this->bestCovariance> this->currentCovariance)
    {
       // this->bestCovanriance=this->currentCovariance;
        this->last_estimated_x  =this->estimated_x;
        this->last_estimated_y  =this->estimated_y;
        this->last_estimated_yaw=this->estimated_yaw;
        this->diference_odom_x   = this->odom_x;
        this->diference_odom_y   = this->odom_y;
        this->diference_odom_yaw = this->odom_yaw;

    }
    pnt::Point odom_point(this->odom_x, this->odom_y);
    pnt::Point diference_odom(this->diference_odom_x, this->diference_odom_y);

    pnt::Point current_position(this->last_estimated_x,this->last_estimated_y);
    pnt::Point estimated_position=pnt::angledPoint(current_position,current_position+odom_point-diference_odom,-this->last_estimated_yaw);

    return estimated_position;
}
double LocalPathPlanner::EstimatedYaw()
{
    double diference_angle=this->odom_yaw-diference_odom_yaw;
    double yaw=this->last_estimated_yaw+diference_angle;
    double decimal=yaw-(int)yaw;
    yaw=(int)(yaw+360)%360+decimal;
    return yaw;
}
}
