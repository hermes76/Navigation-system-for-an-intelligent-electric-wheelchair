 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <nav_msgs/Path.h>


#include "pathPlanning.h"
#include "pgm.h"
#include "quadTree.h"

using namespace std;
using std::string;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

 namespace global {

 class GlobalPathPlanner : public nav_core::BaseGlobalPlanner {
 private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    
    base_local_planner::WorldModel* world_model_; 

    ros::Publisher plan_pub_;
    
    string global_frame;

    bool initialized_;

    double step_size_;
    double min_dist_from_robot_;
    double resolution;
    double width;
    double height;

    pnt::Point start;
    pnt::Point end;




 public:
    GlobalPathPlanner();
    GlobalPathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
                );
   void mapToWorld(double mx, double my, double& wx, double& wy);
   bool worldToMap(double wx, double wy, double& mx, double& my);
   void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  };
 };
 #endif