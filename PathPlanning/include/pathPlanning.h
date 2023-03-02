#ifndef PATHPLANNING
#define PATHPLANNING
#include <bits/stdc++.h>
#include "point.h"
#include "quadTree.h"
#include <SFML/Graphics.hpp>
using namespace std;
namespace path{
    class PathPlanning
    {
       
        private:
            vector<pnt::Point> route;
            vector<double> cost;
            vector<pnt::Point> area;
         public:
            PathPlanning();
            PathPlanning(vector<pnt::Point> route, vector<double> cost,
            vector<pnt::Point> area);

            vector<double> getCost();
            vector<pnt::Point> getRoute();
            vector<pnt::Point> getArea();

            void setCost(vector<double> cost);
            void setRoute(vector<pnt:: Point> route);
            void setArea(vector<pnt::Point> area);

            void push_vertex(pnt::Point vertex);
            void drawPath(sf:: RenderWindow & window);
            //clockwise
            void drawArea(sf:: RenderWindow & window);


    };
    class PathTree{
        private:
            vector<pnt::Point> nodes;
            vector<double>cost;
            vector<int> parent;
            vector<pnt::Point> area;
            int goal;
        public:
            PathTree();
            PathTree(vector<pnt::Point> nodes, vector<double> cost, 
            vector<int> parent, vector<pnt::Point> area);

           vector<double> getCost();
            vector<pnt::Point> getNodes();
            vector<int> getParent();
            vector<pnt::Point> getArea();
            int getGoal();

            void setParent(vector<int> parent);
            void setCost(vector<double> cost);
            void setNodes(vector<pnt:: Point> route);
            void setArea(vector<pnt::Point> area);
            void setGoal(int goal);

            void push_node(pnt::Point vertex);
            void drawTree(sf::RenderWindow &window);
            void drawArea(sf:: RenderWindow & window);

            PathPlanning getPathPlanning();

    };
    bool obstacleFree(vector<vector<bool>> &grid,pnt::Point pointBegin, pnt::Point pointEnd);
    pnt::Point steer(pnt::Point near,pnt::Point pRand, int distance);
    PathTree RRT(vector<vector<bool>> &grid, pnt::Point start,pnt::Point end,int distance, int x1,int y1, int x2, int y2);
    PathTree informedRRTStar(vector<vector<bool>> &grid,PathPlanning &route, int distance,int x1, int y1, int x2, int y2);

}
#endif