#include "include/point.h"
namespace pnt{
Point::Point()
{

}
Point::Point(double x,double y)
{
    this->x=x;
    this->y=y;
}
double Point::getX()
{
    return this->x;
}
double Point::getY()
{
    return this->y;
}
void Point::setX(double x)
{
    this->x=x;
}
void Point::setY(double y)
{
    this->y=y;
}
void Point::printPoint()
{
    cout<<setprecision(10)<<"X: "<<this->x<<" Y: "<<this->y<<endl;
}

Point operator + (Point a,Point b)
{
    return Point (a.getX()+ b.getX(), a.getY()+ b.getY());
}

Point operator - (Point a,Point b)
{
   return Point (a.getX()- b.getX(), a.getY()- b.getY());
}

bool operator == (Point a, Point b)
{
    if(a.getX()==b.getX() && a.getY()==b.getY())
        return true;
    return false;
}

bool operator != (Point a, Point b)
{
    if(a.getX()!=b.getX() || a.getY()!=b.getY())
        return true;
    return false;
}

double operator * (Point a,Point b)
{
    return (a.getX()*b.getY())-(a.getY()*b.getX());
}
bool operator <= (Point a, Point b)
{
    if(a.getX()<= b.getX())
        return true;
    return a.getY()<= b.getY();
}
bool operator >= (Point a, Point b)
{
    if(a.getX()>= b.getX())
        return true;
    return a.getY()>= b.getY();
}
bool operator < (Point a,Point b)
{
    if(a.getX()< b.getX())
        return true;
    return a.getY()< b.getY();
}
bool operator > (Point a, Point b)
{
    if(a.getX()> b.getX())
        return true;
    return a.getY()> b.getY();
}
double euclidianDistance(Point a,Point b)
{
    double x,y;
    x=(a.getX()-b.getX());
    y=(a.getY()-b.getY());
    return (x*x+y*y);
}
double euclidianDistanceSqrt(Point a,Point b)
{
    double x,y;
    x=(a.getX()-b.getX());
    y=(a.getY()-b.getY());
    double d=sqrt((x*x)+(y*y));
    return d;
}

double slope(Point a,Point b)
{
    if(b.getX()==a.getX())
        return (b.getY()-a.getY())/(b.getX()-a.getX()+0.000000001);
    return (b.getY()-a.getY())/(b.getX()-a.getX());
}

Point generateRandomPoint(int x1,int y1, int x2, int y2)
{
    Point testPoint(rand()%(x2-x1)+x1,rand()%(y2-y1)+y1);
    return testPoint;
}
Point angledPoint(Point origin,Point p,double angle)
{
    double m=slope(origin,p);
  
    double alfa=atan(m);
    double d=euclidianDistanceSqrt(origin,p);
    if(p.getX()<origin.getX())       
        alfa=alfa+3.1416;
    p.setX(d*cos(alfa+angle));
    p.setY(d*sin(alfa+angle));
    p=p+origin;
    return p;
}
bool comparePointInLimits(Point compare,int width, int height,int x1,int y1, int x2, int y2)
{  
    if(compare.getX()>=max(x1,0) && compare.getX()<min(x2,width)
     && compare.getY()>=max(0,y1) && compare.getY()<min(height,y2))
        return true;
    return false;   
}
bool Point::closest(Point a, Point b)
{
    double dx1,dx2,dy1,dy2;
    dx1=a.getX()-this->x;
    dy1=a.getY()-this->y;
    dx2=b.getX()-this->x;
    dy2=b.getY()-this->y;
    if(dx1*dx1 + dy1*dy1< dx2*dx2 + dy2*dy2)
        return true;
    return false;
}
bool Point::closest(Point a, Point b,double &distance)
{
    double dx1,dx2,dy1,dy2;
    double d1,d2;
    dx1=a.getX()-this->x;
    dy1=a.getY()-this->y;
    dx2=b.getX()-this->x;
    dy2=b.getY()-this->y;
    d1=dx1*dx1 + dy1*dy1;
    d2=dx2*dx2 + dy2*dy2;

    if(d1< d2)
    {
        distance=d1;
        return true;
    }
    distance=d2;
    return false;
}
bool closest(Point origin,Point a, Point b)
{
    return origin.closest(a,b);
}
bool closest(Point origin,Point a, Point b,double &distance)
{
    return origin.closest(a,b,distance);
}

}