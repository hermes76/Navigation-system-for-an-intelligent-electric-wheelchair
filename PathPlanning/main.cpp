#include "include/pathPlanning.h"
#include "include/pgm.h"
#include "include/quadTree.h"
#include <SFML/Graphics.hpp>
using namespace std;

void drawObstacles( sf::RenderWindow &window, pgm::Pgm &image)
{
    for(int y=0; y<image.getHeight(); y++)
    {
        for(int x=0; x<image.getWidth(); x++)
        {
            if(image.data[y][x])
            {
                sf::ConvexShape convex;
                convex.setPointCount(4);
                convex.setPoint(0, sf::Vector2f(x, y));
                convex.setPoint(1, sf::Vector2f(x+1, y));
                convex.setPoint(2, sf::Vector2f(x+1, y+1));
                convex.setPoint(3, sf::Vector2f(x, y+1));
                convex.setFillColor(sf::Color(100, 250, 50));
                window.draw(convex);
            }
        }
    }
}
int main()
{
    srand(time(0));
    pgm::Pgm image("my_map.pgm");

    image.cutTrash();
    image.inflateGrid(3,3);
        cout<<image.getWidth()<<' '<<image.getHeight()<<endl;

    pnt::Point start(40,180);
    pnt::Point end(290, 180);
    //pnt::Point start(30,20);
    //pnt::Point end(100, 70);

    sf::RenderWindow window(sf::VideoMode(image.getWidth(), image.getHeight()),"Navigation");

    sf::CircleShape startPoint(1.f);
    sf::CircleShape endPoint(1.f);

    path::PathTree tree=path::RRT(image.data,start,end,10,0,0,image.getWidth(), image.getHeight());
    path::PathPlanning route=tree.getPathPlanning();
    path::PathTree optimized= path::informedRRTStar(image.data,route,10,0,0,image.getWidth(), image.getHeight());
    endPoint.setFillColor(sf::Color::Red);
    startPoint.setFillColor(sf::Color::Blue);
    startPoint.setPosition(start.getX(),start.getY());
    endPoint.setPosition(end.getX(), end.getY());
    
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        
        
        window.clear();
        drawObstacles(window,image);
         
        //tree.drawTree(window);
        //route.drawPath(window);
        optimized.drawTree(window);
        optimized.drawArea(window);

        window.draw(startPoint);
        window.draw(endPoint);
        window.display();
    }

   

}