#include <pluginlib/class_list_macros.h>
#include "global_path_planner.h"


PLUGINLIB_EXPORT_CLASS(global::GlobalPathPlanner, nav_core::BaseGlobalPlanner)

namespace global {
     GlobalPathPlanner::GlobalPathPlanner()
    :costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false)
    {

    }

    GlobalPathPlanner::GlobalPathPlanner(std::string name, 
                                        costmap_2d::Costmap2DROS* costmap_ros)

    : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){

        initialize(name, costmap_ros);
    }
    void GlobalPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            this->costmap_ros_ = costmap_ros;
            this->costmap_ = costmap_ros_->getCostmap();
            this->global_frame= costmap_ros->getGlobalFrameID();
            this->resolution=costmap_->getResolution();

            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle nh_area;


            private_nh.param("step_size", this->step_size, 5);
            private_nh.param("max_iterations", this->max_iterations, 5000);
            private_nh.param("show_rrt_tree",this->enable_rrt_tree, false);
            private_nh.param("show_optimization_area", this->optimization_area, false);
            private_nh.param("show_optimized_tree", this->optimized_tree, false);


            this->world_model_ = new base_local_planner::CostmapModel(*costmap_); 

            ros::NodeHandle nh("~/" + name);

            plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
            this->limit_area_sub   = nh_area.subscribe("/clicked_point",
                                                100, 
                                                &GlobalPathPlanner::clickedPointCallback,
                                                this);

            this->initialized_ = true;
            this->end=pnt::Point(-999999,-999999);

            
            ROS_INFO("Global Path Planner initialized successfully.");

            this->limit_area_pub=nh.advertise<visualization_msgs::Marker>("limit_area",1);

            if(this->enable_rrt_tree)
                this->rrt_tree_pub = nh.advertise<visualization_msgs::Marker>("tree", 1);

            if(this->optimization_area)
                this->optimization_area_pub=nh.advertise<visualization_msgs::Marker>("optimization_area",1);

             if(this->optimized_tree)
                this->optimized_tree_pub=nh.advertise<visualization_msgs::Marker>("optimized_tree",1);
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool GlobalPathPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                     const geometry_msgs::PoseStamped& goal,
                                     std::vector<geometry_msgs::PoseStamped>& plan )
        {

        if(limit_area.empty())
        {
            ROS_WARN("Please publish Points");
            return false;
        }
        
        //draw valid area
        
        visualization_msgs::Marker area_line;
        initLine(area_line,0.02);
        drawLine(limit_area_pub,area_line,limit_area[0],pnt::Point(limit_area[0].getX(),limit_area[1].getY()),1,0,0,1);
        drawLine(limit_area_pub,area_line,pnt::Point(limit_area[0].getX(),limit_area[1].getY()),limit_area[1],1,0,0,1);
        drawLine(limit_area_pub,area_line,limit_area[1],pnt::Point(limit_area[1].getX(),limit_area[0].getY()),1,0,0,1);
        drawLine(limit_area_pub,area_line,pnt::Point(limit_area[1].getX(),limit_area[0].getY()),limit_area[0],1,0,0,1);

           

        double x,y;

        pnt::Point new_start(start.pose.position.x,start.pose.position.y);
        pnt::Point new_goal(goal.pose.position.x, goal.pose.position.y);

        geometry_msgs::PoseStamped pose;
        
        ros::Time plan_time;



        if(this->end==new_goal && !global_plan.empty())
        {
            bool ok=true;
            for(int i=0; i< global_plan.size()-1; i++)
            {
                if(!path::obstacleFree(*this->costmap_,global_plan[i],global_plan[i+1],200))
                    ok=false;
            }
            if(!ok)
            {
                plan.clear();
                global_plan.clear();
                ros::Time plan_time = ros::Time::now();

                pose.header.stamp = plan_time;
                pose.header.frame_id = this->global_frame;
                pose.pose=start.pose;
                plan.push_back(pose);

                plan_time = ros::Time::now();

                pose.header.stamp = plan_time;
                pose.header.frame_id = this->global_frame;
                pose.pose.position.x = new_start.getX()+0.005;
                pose.pose.position.y = new_start.getY()+0.005;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);

                this->end=new_start;
                publishPlan(plan);
            }
            return true;
        }
        
        this->end=new_goal;

        plan.clear();
        
        path::PathTree routes;
        path::PathPlanning path;
        

        worldToMap(new_start.getX(), new_start.getY(),x,y);
        new_start.setX(x);
        new_start.setY(y);
        
        worldToMap(new_goal.getX(), new_goal.getY(),x,y);
        new_goal.setX(x);
        new_goal.setY(y);

        bool ok=true;

        if(!path::RRT(routes,
                        *this->costmap_,
                        new_start,
                        new_goal,
                        this->step_size,
                        this->max_iterations,
                        this->limit_area[0].getX(),
                        this->limit_area[0].getY(),
                        this->limit_area[1].getX(),
                        this->limit_area[1].getY()))
        {
            ok=false;
        }
        if (this->enable_rrt_tree)
        {
            visualization_msgs::Marker line;
            initLine(line,0.02);
            vector<pnt::Point> nodes=routes.getNodes();
            vector<int> parent = routes.getParent();

            for (int i=1; i<nodes.size(); i++)
                drawLine(rrt_tree_pub,line,nodes[i],nodes[parent[i]],0,1,0,1);
           
        }
       

        
        if(!ok)
            return true;
        path=routes.getPathPlanning();

        routes=path::informedRRTStar(*this->costmap_,
                                        path,this->step_size,
                                        this->limit_area[0].getX(),
                                        this->limit_area[0].getY(),
                                        this->limit_area[1].getX(),
                                        this->limit_area[1].getY());

       
        path=routes.getPathPlanning();

         if(this->optimized_tree)
        {
            visualization_msgs::Marker optimized_tree_line;
            initLine(optimized_tree_line,0.02);
            vector<pnt::Point> nodes=routes.getNodes();
            vector<int> parent = routes.getParent();

            for (int i=1; i<nodes.size(); i++)
                drawLine(optimized_tree_pub,optimized_tree_line,nodes[i],nodes[parent[i]],1,0,0,0.5);
        }
        
        if(this->optimization_area)
        {
            visualization_msgs::Marker optimization_line;
            initLine(optimization_line,0.02);
            vector<pnt::Point> optimization_points=routes.getArea();

            for (int i=1; i<optimization_points.size(); i++)
                drawLine(optimization_area_pub,optimization_line,optimization_points[i-1],
                optimization_points[i],0,0,1,1);

            drawLine(optimization_area_pub,optimization_line,optimization_points[3],
            optimization_points[0],0,0,1,1);

        }
        

        plan_time = ros::Time::now();

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


    void GlobalPathPlanner::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        double mx,my;
        worldToMap(msg->point.x,msg->point.y,mx,my);
        pnt::Point publishPoint(mx,my);
        this->limit_area.push_back(publishPoint);
        publishPoint.printPoint();
    }

    void GlobalPathPlanner::mapToWorld(double mx, double my, double& wx, double& wy) 
    {
        wx = costmap_->getOriginX() + mx * costmap_->getResolution();
        wy = costmap_->getOriginY() + my * costmap_->getResolution();
    }

    bool GlobalPathPlanner::worldToMap(double wx, double wy, double& mx, double& my) 
    {
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
        nav_msgs::Path rviz_path;
        rviz_path.poses.resize(plan.size());

        if (plan.empty())
        {
            rviz_path.header.frame_id = "map";
            rviz_path.header.stamp = ros::Time::now();
        }
        else
        {
            rviz_path.header.frame_id = plan[0].header.frame_id;
            rviz_path.header.stamp = plan[0].header.stamp;
        }

        for (unsigned int i = 0; i < plan.size(); i++)
            rviz_path.poses[i] = plan[i];
            
        plan_pub_.publish(rviz_path);
    }
    void GlobalPathPlanner::initLine(visualization_msgs::Marker& line,float width)
    {
        line.header.frame_id = "map";
        line.id = 0;
        line.ns = "tree";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = width;  // in meters (width of segments)
    }
    void GlobalPathPlanner::drawLine(ros::Publisher& pub,
                                    visualization_msgs::Marker& line, 
                                    pnt::Point start, pnt::Point end,
                                    float r, float g, float b, float a)
    {
        double x1,y1,x2,y2;
        line.header.stamp = ros::Time::now();

        geometry_msgs::Point p1, p2;
        std_msgs::ColorRGBA c1, c2;

        mapToWorld(start.getX(),start.getY(),x1,y1);
        mapToWorld(end.getX(),end.getY(),x2,y2);
        

        p1.x = x1;
        p1.y = y1;
        p1.z = 1.0;

        p2.x = x2;
        p2.y = y2;
        p2.z = 1.0;

        c1.r = r;  // 1.0=255
        c1.g = g;
        c1.b = b;
        c1.a = a;  // alpha

        c2.r = r;  // 1.0=255
        c2.g = g;
        c2.b = b;
        c2.a = a;  // alpha

        line.points.push_back(p1);
        line.points.push_back(p2);

        line.colors.push_back(c1);
        line.colors.push_back(c2);

        pub.publish(line);
    }
    
}