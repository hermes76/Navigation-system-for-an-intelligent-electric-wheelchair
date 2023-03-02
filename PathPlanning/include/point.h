#ifndef POINT
#define POINT

#include <bits/stdc++.h>
using namespace std;

namespace pnt
{
    class Point{
        private:
            double x,y;
        public:
            Point();
            Point(double x,double y);
            double getX();
            double getY();
            void setX(double x);
            void setY(double y);
            void printPoint();
            bool closest(Point a, Point b);
            bool closest(Point a, Point b,double &distance);


    };

    //operadores de puntos
    Point operator + (Point a,Point b);
    Point operator - (Point a,Point b);
    bool operator == (Point a, Point b);
    bool operator != (Point a, Point b);
    bool operator <= (Point a, Point b);
    bool operator >= (Point a, Point b);
    bool operator < (Point a,Point b);
    bool operator > (Point a, Point b);
    //punto cruz
    double operator * (Point a,Point b);

    //devuelve la distancia euclidiana
    double euclidianDistance(Point a, Point b);
    double euclidianDistanceSqrt(Point a,Point b);
    double slope(Point a,Point b);
    Point generateRandomPoint(int x1,int y1, int x2, int y2);
    Point angledPoint(pnt::Point origin,pnt::Point p,double angle);
    bool comparePointInLimits(Point compare,int width, int height,int x1,int y1, int x2, int y2);
    bool closest(Point origin,Point a, Point b);
    bool closest(Point origin,Point a, Point b, double &distance);



    using::operator +;
    using::operator -;
    using::operator ==;
    using::operator !=;
    using::operator <=;
    using::operator >=;
    using::operator <;
    using::operator >;
}
#endif