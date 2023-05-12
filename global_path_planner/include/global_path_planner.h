 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <nav_msgs/Path.h>
 #include <visualization_msgs/Marker.h>
 #include "geometry_msgs/PointStamped.h"


#include "pathPlanning.h"
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
            ros::Publisher rrt_tree_pub;
            ros::Publisher limit_area_pub; 
            ros::Publisher optimization_area_pub;
            ros::Publisher optimized_tree_pub;

            ros::Subscriber limit_area_sub;
    
            string global_frame;

            bool initialized_;
            bool enable_rrt_tree;
            bool optimization_area;
            bool optimized_tree;
    
            int step_size;
            int max_iterations;

            double min_dist_from_robot_;
            double resolution;

            pnt::Point start;
            pnt::Point end;

            vector<pnt::Point> global_plan;


            //x1 y1 x2 y2
            vector<pnt::Point> limit_area;
        
    public:
        GlobalPathPlanner();
        GlobalPathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
        void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void initLine(visualization_msgs::Marker& line,float width);
        void drawLine(ros::Publisher& rrt_tree_pub,
                                    visualization_msgs::Marker& line, 
                                    pnt::Point p1, pnt::Point p2,
                                    float r, float g, float b, float a);
    };
 };
 #endif