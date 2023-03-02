#include "include/quadTree.h"
namespace quad
{
Node::Node()
{

}
Node::Node(pnt::Point point, int id)
{
    this->point=point;
    this->id=id;    
}
QuadTree:: QuadTree()
{
    this->divided=false;
    this->southWest=NULL;
    this->southEast=NULL;
    this->northEast=NULL;
    this->northWest=NULL;
} 
QuadTree::QuadTree(double x1, double y1, double x2, double y2,int allowedPoints)
{
    this->boundary=Area(x1,y1,x2,y2);
    this->divided=false;
    this->southWest=NULL;
    this->southEast=NULL;
    this->northEast=NULL;
    this->northWest=NULL;
    this->allowedPoints=allowedPoints;
}
QuadTree::QuadTree(Area boundary,int allowedPoints)
{
    this->boundary=boundary;
    this->divided=false;
    this->southWest=NULL;
    this->southEast=NULL;
    this->northEast=NULL;
    this->northWest=NULL;
    this->allowedPoints=allowedPoints;
}
/*QuadTree::~QuadTree()
{
    if(this->divided)
    {
        delete this->northWest;
        delete this->northEast;
        delete this->southWest;
        delete this->southEast;
    }
}*/
bool QuadTree::insert(pnt::Point point, int id)
{
    if(!this->boundary.pointInArea(point))
        return false;
    if(this->points.size()<this->allowedPoints && !this->divided)
    {
        points.push_back(Node(point,id));
        return true;
    }
    if(!this->divided)
    {
        this->divided=true;
        subdivided();
    }
    if(this->northWest->insert(point,id))return true;
    if(this->northEast->insert(point,id))return true;
    if(this->southWest->insert(point,id))return true;
    if(this->southEast->insert(point,id))return true;
    return false;

}
void QuadTree::subdivided()
{
    double halfPartX= (boundary.getX2()-boundary.getX1())/2;
    double halfPartY= (boundary.getY2()-boundary.getY1())/2;
    double startX=this->boundary.getX1();
    double startY=this->boundary.getY1();
    double endX=this->boundary.getX2();
    double endY=this->boundary.getY2();

    
    this->northWest=new QuadTree(Area(startX,startY,startX+halfPartX,startY+halfPartY),this->allowedPoints);
    this->northEast=new QuadTree(Area(startX+halfPartX,startY,endX,startY+halfPartY),this->allowedPoints);
    this->southWest=new QuadTree(Area(startX,startY+halfPartY,startX+halfPartX,endY),this->allowedPoints);
    this->southEast=new QuadTree(Area(startX+halfPartX,startY+halfPartY,endX,endY),this->allowedPoints);
}
void QuadTree::show(sf::RenderWindow &window)
{
    for(int i=0; i<this->points.size(); i++)
    {
        sf::CircleShape startPoint(0.5);
        startPoint.setFillColor(sf::Color::Magenta);
        startPoint.setPosition(points[i].point.getX(),points[i].point.getY());
        window.draw(startPoint);
    }
        int x1=this->boundary.getX1();
        int x2=this->boundary.getX2();
        int y1=this->boundary.getY1();
        int y2=this->boundary.getY2();
        sf::Vertex line1[2];
        line1[0] = sf::Vertex(sf::Vector2f(x1, y1));
        line1[1] = sf::Vertex(sf::Vector2f(x2,y1));
        window.draw(line1, 2, sf::Lines);
        sf::Vertex line2[2];
        line2[0] = sf::Vertex(sf::Vector2f(x2, y1));
        line2[1] = sf::Vertex(sf::Vector2f(x2,y2));
        window.draw(line2, 2, sf::Lines);
        sf::Vertex line3[2];
        line3[0] = sf::Vertex(sf::Vector2f(x2, y2));
        line3[1] = sf::Vertex(sf::Vector2f(x1,y2));
        window.draw(line3, 2, sf::Lines);
        sf::Vertex line4[2];
        line4[0] = sf::Vertex(sf::Vector2f(x1, y2));
        line4[1] = sf::Vertex(sf::Vector2f(x1,y1));
        window.draw(line4, 2, sf::Lines);

    if(this->divided)
    {
        this->northWest->show(window);
        this->northEast->show(window);
        this->southWest->show(window);
        this->southEast->show(window);
    }

}
 vector<Node> QuadTree::rangeQuery(double x1,double y1, double x2, double y2)
{
    Area range=Area(x1,y1,x2,y2);
    return rangeQuery(range);
}

vector<Node> QuadTree::rangeQuery(Area range)
{
    vector<Node> searchPoints;
        if(!this->boundary.intersectBox(range))
            return searchPoints;

        for(int i=0; i<this->points.size(); i++)
            if(range.pointInArea(points[i].point))
                searchPoints.push_back(points[i]);
        
        if(!this->divided)
            return searchPoints;
        for(Node p: this->northWest->rangeQuery(range))
            searchPoints.push_back(p);
        for(Node p: this->northEast->rangeQuery(range))
            searchPoints.push_back(p);
        for(Node p: this->southWest->rangeQuery(range))
            searchPoints.push_back(p);
        for(Node p: this->southEast->rangeQuery(range))
            searchPoints.push_back(p);
        return searchPoints;
}
vector<Node> QuadTree::circleQueryCall(pnt::Point center, double radius)
{
    Area range(center.getX()-radius, center.getY()-radius, center.getX()+radius, center.getY()+radius);
    vector<Node> query= rangeQuery(range);
    vector<Node> circle;
    for(int i=0; i<query.size(); i++)
    {
        if(pnt::euclidianDistance(center,query[i].point)<=radius*radius)
            circle.push_back(query[i]);

    }
    return circle;
}
vector<Node> QuadTree::circleQuery(pnt::Point center, double radius)
{
    return circleQueryCall(center,radius);
}
Node QuadTree::closestPointCall(pnt:: Point point,double &distance)
{
    double x=point.getX();
    double y=point.getY();
    if(x<this->boundary.getX1()-distance || x>this->boundary.getX2() +distance || y>this->boundary.getY2()+distance || y<this->boundary.getY1()-distance)
        return Node(pnt::Point(1e9,1e9),0);
    
    Node best(pnt::Point(1e9,1e9),0);
    double d;
    for(int i=0; i<this->points.size(); i++)
    {
        d=pnt::euclidianDistanceSqrt(point,this->points[i].point);
        if(d<distance)
        {
            best=points[i];
            distance=d;
        }
    }
    if(this->divided)
    {
        Node a=this->northWest->closestPointCall(point,distance);
        if(a.point.getX()!=1e9)best=a;
        Node b=this->northEast->closestPointCall(point,distance);
        if(b.point.getX()!=1e9)best=b;
        Node c=this->southWest->closestPointCall(point,distance);
        if(c.point.getX()!=1e9)best=c;
        Node d=this->southEast->closestPointCall(point,distance);
        if(d.point.getX()!=1e9)best=d;


    }
    return best;


}
Node QuadTree::closestPoint(pnt:: Point point)
{
    double d=1e9;
    return closestPointCall(point,d);
}

Node QuadTree::closestPoint(pnt:: Point point,double &distance)
{
    distance=1e9;
    return closestPointCall(point,distance);
}
Area::Area(double x1, double y1, double x2, double y2){
    this->x1=x1;
    this->x2=x2;
    this->y1=y1;
    this->y2=y2;
}
Area::Area()
{

}
void Area::setX1(double x1)
{
    this->x1=x1;
}
void Area::setX2(double x2)
{
    this->x2=x2;
}
void Area::setY1(double y1)
{
    this->y1=y1;
}
void Area::setY2(double y2)
{
    this->y2=y2;
}
double Area::getX1()
{
    return this->x1;
}
double Area::getX2()
{
    return this->x2;
}
double Area::getY1()
{
    return this->y1;
}
double Area::getY2()
{
   return this->y2;
}
bool Area:: pointInArea(pnt::Point point)
{
    int x,y;
    x=point.getX();
    y=point.getY();
    if(x>=this->x1 && x<=this->x2 && y>= this->y1 && y<=this->y2)
        return true;
    return false;
}
bool Area:: pointInArea(double x,double y)
{
    if(x>=this->x1 && x<=this->x2 && y>= this->y1 && y<=this->y2)
        return true;
    return false;
}
bool Area::intersectBox(Area range)
{
    int a,b,c,d;
    a=range.getX1();
    b=range.getY1();
    c=range.getX2();
    d=range.getY2();
    if (a==c || b==d || this->x2==this->x1 || this->y1==this->y2)
        return false;
   
    if (a>this->x2 || this->x1>c)return false;
    if (d<this->y1 || this->y2<b)return false;
    return true;
}
}