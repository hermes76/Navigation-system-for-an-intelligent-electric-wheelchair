
#include <pluginlib/class_list_macros.h>
#include "global_path_planner.h"


PLUGINLIB_EXPORT_CLASS(global::GlobalPathPlanner, nav_core::BaseGlobalPlanner)

namespace global {

  GlobalPathPlanner::GlobalPathPlanner()
  :costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false)
  {

  }

  GlobalPathPlanner::GlobalPathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){

    initialize(name, costmap_ros);
  }

  void GlobalPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_)
    {
       this->costmap_ros_ = costmap_ros;
       this->costmap_ = costmap_ros_->getCostmap();
       this->global_frame= costmap_ros->getGlobalFrameID();
       this->width=costmap_->getSizeInCellsX();
       this->height=costmap_->getSizeInCellsY();
       this->resolution=costmap_->getResolution();

       ros::NodeHandle private_nh("~/" + name);

       private_nh.param("step_size", this->step_size, 5);
       private_nh.param("max_iterations", this->max_iterations, 5000);

       this->world_model_ = new base_local_planner::CostmapModel(*costmap_); 

       ros::NodeHandle nh("~/" + name);

       plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
  
       this->initialized_ = true;
       this->end=pnt::Point(-999999,-999999);

      ROS_INFO("Global Path Planner initialized successfully.");
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

 bool GlobalPathPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

  pnt::Point new_start(start.pose.position.x,start.pose.position.y);
  pnt::Point new_goal(goal.pose.position.x, goal.pose.position.y);


  if(this->end==new_goal && !global_plan.empty())
  {
    bool ok=true;
    for(int i=0; i< global_plan.size()-1; i++)
    {
        if(!path::obstacleFree(*this->costmap_,global_plan[i],global_plan[i+1]))
          ok=false;
    }
    if(ok)
      return true;
  }
 

  
  this->end=new_goal;

  plan.clear();
  
  path::PathTree routes;
  path::PathPlanning path;
  
  double x,y;

  worldToMap(new_start.getX(), new_start.getY(),x,y);
  new_start.setX(x);
  new_start.setY(y);
  
  worldToMap(new_goal.getX(), new_goal.getY(),x,y);
  new_goal.setX(x);
  new_goal.setY(y);

  if(!path::RRT(routes,*this->costmap_,new_start,new_goal,this->step_size,this->max_iterations,0,0,costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY()))
    return false;

  path=routes.getPathPlanning();

  routes=path::informedRRTStar(*this->costmap_,path,this->step_size,0,0,costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());

  path=routes.getPathPlanning();

  geometry_msgs::PoseStamped pose;


  ros::Time plan_time = ros::Time::now();
  global_plan=path.getRoute();
  for(pnt::Point i: global_plan)
  {
    mapToWorld(i.getX(), i.getY(),x,y);
    pose.header.stamp = plan_time;
    pose.header.frame_id = this->global_frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
  publishPlan(plan);

  return true;
 }
 void GlobalPathPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

bool GlobalPathPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}
void GlobalPathPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  // create a message for the plan
  nav_msgs::Path rviz_path;
  rviz_path.poses.resize(plan.size());

  if (plan.empty())
  {
    // still set a valid frame so visualization won't hit transform issues
    rviz_path.header.frame_id = "map";
    rviz_path.header.stamp = ros::Time::now();
  }
  else
  {
    rviz_path.header.frame_id = plan[0].header.frame_id;
    rviz_path.header.stamp = plan[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the plan is all in the same frame
  for (unsigned int i = 0; i < plan.size(); i++)
    rviz_path.poses[i] = plan[i];
    
  plan_pub_.publish(rviz_path);
}

 };
