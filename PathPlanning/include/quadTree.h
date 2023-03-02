#ifndef QUADTREE
#define QUADTREE
#include "point.h"
#include <SFML/Graphics.hpp>

namespace quad{
class Node{
    public:
        Node();
        Node(pnt::Point point, int id);
        pnt::Point point;
        int id;

};
class Area
{
private:
    double x1,y1,x2,y2;
public:
    Area(double x1, double y1, double x2, double y2);
    Area();
    void setX1(double x1);
    void setX2(double x2);
    void setY1(double y1);
    void setY2(double y2);
    double getX1();
    double getX2();
    double getY1();
    double getY2();
    bool pointInArea(pnt::Point point);
    bool pointInArea(double x,double y);
    bool intersectBox(Area range);


};
class QuadTree{
    private:
        bool divided;
        int allowedPoints;
        vector<Node> points;
        Area boundary;
        QuadTree* northWest;
        QuadTree* northEast;
        QuadTree* southWest;
        QuadTree* southEast;
        vector<Node> circleQueryCall(pnt::Point center, double radius);
        Node closestPointCall(pnt:: Point point,double &distance);

    public:
        QuadTree();
        QuadTree(double x1, double y1, double x2, double y2,int allowedPoints);
        QuadTree(Area boundary,int allowedPoints);
        //~QuadTree();
        bool insert(pnt::Point point,int id);
        void subdivided();
        void show(sf::RenderWindow &window);
        vector<Node> rangeQuery(double x1,double y1, double x2, double y2);
        vector<Node> rangeQuery(Area range);
        vector<Node> circleQuery(pnt::Point center, double radius);
        Node closestPoint(pnt:: Point point,double &distance);
        Node closestPoint(pnt:: Point point);


};

}
#endif