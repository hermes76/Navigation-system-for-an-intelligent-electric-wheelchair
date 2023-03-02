#include "include/pathPlanning.h"
namespace path{
PathPlanning::PathPlanning()
{

}
PathPlanning::PathPlanning(vector<pnt::Point> route, vector<double> cost,
            vector<pnt::Point> area)
{
    this->route=route;
    this->cost=cost;
    this->area=area;
}
vector<double> PathPlanning::getCost()
{
    return this->cost;
}
vector<pnt::Point> PathPlanning::getRoute()
{
    return this->route;
}
void PathPlanning::setCost(vector<double> cost)
{
    this->cost=cost;
}
void PathPlanning::setRoute(vector<pnt:: Point> route)
{
    this->route=route;
}

void PathPlanning::push_vertex(pnt::Point vertex)
{
    this->route.push_back(vertex);
}

void PathPlanning::drawPath(sf::RenderWindow &window)
{
    for(int x=1; x<route.size(); x++)
    {
        sf::Vertex line[2];
        line[0] = sf::Vertex(sf::Vector2f(route[x-1].getX(), route[x-1].getY()));
        line[1] = sf::Vertex(sf::Vector2f(route[x].getX(), route[x].getY()));
        window.draw(line, 2, sf::Lines);
    }
}

vector<pnt::Point> PathPlanning::getArea()
{
    return this->area;
}

void PathPlanning::setArea(vector<pnt::Point> area)
{
    this->area=area;
}
PathTree::PathTree()
{

}
void PathPlanning:: drawArea(sf::RenderWindow & window)
{
       sf::ConvexShape convex;
        convex.setPointCount(4);
        convex.setPoint(0, sf::Vector2f(area[0].getX(), area[0].getY()));
        convex.setPoint(1, sf::Vector2f(area[1].getX(), area[1].getY()));
        convex.setPoint(2, sf::Vector2f(area[2].getX(), area[2].getY()));
        convex.setPoint(3, sf::Vector2f(area[3].getX(), area[3].getY()));
        convex.setFillColor(sf::Color(0,50,50,70));
        window.draw(convex);
}
vector<double> PathTree::getCost()
{
    return this->cost;
}
PathTree::PathTree(vector<pnt::Point> nodes, vector<double> cost, 
            vector<int> parent, vector<pnt::Point> area)
{
    this->nodes=nodes;
    this->cost=cost;
    this->parent=parent;
    this->area=area;
}
vector<pnt::Point> PathTree::getNodes()
{
    return this->nodes;
}
vector<int> PathTree::getParent()
{
    return this-> parent;
}
vector<pnt::Point> PathTree:: getArea()
{
    return this->area;
}
int PathTree:: getGoal()
{
    return this->goal;
}
void PathTree::setParent(vector<int> parent)
{
    this->parent=parent;
}
void PathTree:: setArea(vector<pnt::Point> area)
{
    this->area=area;
}
void PathTree::setCost(vector<double> cost)
{
    this-> cost=cost;
}
void PathTree::setNodes(vector<pnt:: Point> nodes)
{
    this->nodes=nodes;
}
void PathTree::push_node(pnt::Point node){
    this->nodes.push_back(node);
}
void PathTree::setGoal(int goal)
{
    this->goal=goal;
}
void PathTree::drawTree(sf::RenderWindow &window)
{
     for(int i=1; i<nodes.size(); i++)
        {
            sf::Vertex line[2];
            line[0] = sf::Vertex(sf::Vector2f(nodes[i].getX(), nodes[i].getY()));
            line[1] = sf::Vertex(sf::Vector2f(nodes[parent[i]].getX(), nodes[parent[i]].getY()));
            window.draw(line, 2, sf::Lines);
        }
}

void PathTree:: drawArea(sf::RenderWindow & window)
{
       sf::ConvexShape convex;
        convex.setPointCount(4);
        convex.setPoint(0, sf::Vector2f(area[0].getX(), area[0].getY()));
        convex.setPoint(1, sf::Vector2f(area[1].getX(), area[1].getY()));
        convex.setPoint(2, sf::Vector2f(area[2].getX(), area[2].getY()));
        convex.setPoint(3, sf::Vector2f(area[3].getX(), area[3].getY()));
        convex.setFillColor(sf::Color(0,50,50,70));
        window.draw(convex);
}
PathPlanning PathTree:: getPathPlanning()
{
    vector<pnt::Point> pathRoute;
    vector<double> pathCost;
    
    int lastInserted=this->goal;

    while(lastInserted!=0)
    {
        pathRoute.push_back(nodes[lastInserted]);
        pathCost.push_back(cost[lastInserted]);
        lastInserted=parent[lastInserted];
    }

    pathRoute.push_back(nodes[0]);
    pathCost.push_back(0);
    reverse(pathRoute.begin(), pathRoute.end());
    reverse(pathCost.begin(), pathCost.end());

    PathPlanning route(pathRoute,pathCost,this->area);
    return route;
}
bool obstacleFree(vector<vector<bool>> &grid,pnt::Point pointBegin, pnt::Point pointEnd)
{
    //dda algorithm

    int dx=pointEnd.getX()-pointBegin.getX();
    int dy=pointEnd.getY()-pointBegin.getY();

    int steps = abs(dx)>abs(dy)?abs(dx):abs(dy);

    if(steps==0)
        return !grid[pointBegin.getY()][pointBegin.getX()];
    double Xinc = dx/(double)steps;
    double Yinc = dy/(double)steps;

    double x=pointBegin.getX();
    double y=pointBegin.getY();
    for(int i=0; i<=steps; i++)
    {
        x+=Xinc;
        y+=Yinc;
        
        if(grid[y][x])
            return false;
    }
    return true;
}
pnt::Point steer(pnt::Point near,pnt::Point pRand,int distance)
{
    double angle;
    double x,y;

    angle=atan2(pRand.getY()-near.getY(),pRand.getX()-near.getX());
   
    x=near.getX()+distance*cos(angle);
    y=near.getY()+distance*sin(angle);

    return pnt::Point(x,y);
}
void calculateGenerationBox(vector<pnt::Point> route,int &x1, int &y1, int &x2,int &y2,double angle)
{
    pnt:: Point pointRotate;
    x1=x2=route[0].getX();
    y1=y2=route[0].getY();
    for(int i=1; i<route.size(); i++)
    {
        pointRotate=pnt::angledPoint(route[0],route[i],angle);
        x1=min(x1,(int)pointRotate.getX());
        y1=min(y1,(int)pointRotate.getY());
        x2=max(x2,(int)pointRotate.getX());
        y2=max(y2,(int)pointRotate.getY());
    }
    
}
PathTree RRT(vector<vector<bool>> &grid, pnt::Point start,pnt::Point end,int distance, int x1,int y1, int x2, int y2)
{
    quad::QuadTree tree(x1,y1,x2,y2,4);
    vector<pnt::Point> vertex;
    vector<double> cost;
    vector<int> parent;

    bool pathFind=false;
    pnt::Point pNew,pRand;
    quad::Node nearVertex;


    vertex.push_back(start);
    parent.push_back(0);
    cost.push_back(0);
    tree.insert(start,0);


    while(!pathFind)
    {   
        pRand=pnt:: generateRandomPoint(x1,y1,x2-1,y2-1);
        nearVertex= tree.closestPoint(pRand);
        pNew= steer(nearVertex.point,pRand,distance);
        if(!obstacleFree(grid,nearVertex.point,pNew))
            continue;
        cost.push_back(cost[nearVertex.id]+ pnt::euclidianDistanceSqrt(nearVertex.point,pNew));
        parent.push_back(nearVertex.id);
        vertex.push_back(pNew);
        tree.insert(pNew,vertex.size()-1);
        if(pnt::euclidianDistanceSqrt(pNew,end)<=distance && obstacleFree(grid,pNew,end))
            pathFind=true;

    }

    int lastInserted= vertex.size()-1;

    cost.push_back(cost[lastInserted]+ pnt:: euclidianDistanceSqrt(pNew,end));
    parent.push_back(lastInserted);
    vertex.push_back(end);

    PathTree pathTree;

    pathTree.setNodes(vertex);
    pathTree.setCost(cost);
    pathTree.setParent(parent);
    pathTree.setGoal(lastInserted);
   
    return pathTree;
}
vector<int> findClosestNodesInRange(pnt::Point origin, vector<pnt::Point> &nodes,vector<vector<bool>> &grid, double radious, double x1, double y1, double x2, double y2)
{
    vector<int>nearby;

    for(int i=0; i<nodes.size(); i++)
        if(pnt::euclidianDistance(origin,nodes[i])<=radious && obstacleFree(grid,nodes[i],origin))
            nearby.push_back(i);

    return nearby;
}

PathTree informedRRTStar(vector<vector<bool>> &grid,PathPlanning &route, int distance,int x1, int y1, int x2, int y2)
{
    vector<pnt::Point> vertex;
    vector<double> cost;
    vector<int> parent;
    quad::QuadTree tree(x1,y1,x2,y2,4);

    vertex=route.getRoute();
    cost=route.getCost();
    pnt::Point start,end;

    int goal =vertex.size()-1;

    start=vertex[0];
    end=vertex[goal];

    tree.insert(vertex[0],0);
    parent.push_back(0);

    for(int i=1; i<vertex.size(); i++)
    {
        tree.insert(vertex[i],i);
        parent.push_back(i-1);
    }

    double angle=atan(pnt::slope(start,end));
    vector<pnt::Point> corners;

    calculateGenerationBox(vertex,x1,y1,x2,y2,-angle);

    pnt::Point leftUpCorner    = pnt::angledPoint(vertex[0],pnt::Point(x1,y1),angle);
    pnt::Point leftDownCorner  = pnt::angledPoint(vertex[0],pnt::Point(x1,y2),angle);
    pnt::Point rightDownCorner = pnt::angledPoint(vertex[0],pnt::Point(x2,y2),angle);
    pnt::Point rightUpCorner   = pnt::angledPoint(vertex[0],pnt:: Point(x2,y1),angle);
    

    corners.push_back(leftUpCorner);
    corners.push_back(rightUpCorner);
    corners.push_back(rightDownCorner);
    corners.push_back(leftDownCorner);

    vector<quad::Node> circleRange;
    double bestCost;

    pnt::Point pRand;
    quad::Node nearVertex;
    for(int i=0; i<5000; i++)
    {
        pRand = pnt::generateRandomPoint(x1,y1,x2,y2);
        pRand = pnt::angledPoint(start,pRand, angle);
        //pRand.setX(floor(pRand.getX()));
        //pRand.setY(floor(pRand.getY()));
        circleRange= tree.circleQuery(pRand,distance);
       
        vector<quad::Node> nearby;

        for(quad::Node i : circleRange)
            if(obstacleFree(grid,i.point,pRand))
                nearby.push_back(i);

        if(nearby.empty())
            continue;
        
        nearVertex=nearby[0];
        bestCost=cost[nearVertex.id] + pnt::euclidianDistanceSqrt(pRand,vertex[nearby[0].id]);
        

        for(quad::Node i : nearby)
        {
            if(cost[i.id]+ pnt:: euclidianDistanceSqrt(vertex[i.id], pRand)<bestCost)
            {
                nearVertex=i;
                bestCost = cost[i.id]+ pnt:: euclidianDistanceSqrt(vertex[i.id], pRand);
            }
        }
    
        vertex.push_back(pRand);
        cost.push_back(bestCost);
        parent.push_back(nearVertex.id);
        tree.insert(pRand,vertex.size()-1);

        int lastInserted = vertex.size()-1;
	    for(int i=0; i<nearby.size(); i++) 
        {
        
            int par = lastInserted, cur = nearby[i].id;
            if((cost[par] + pnt::euclidianDistanceSqrt(vertex[par], vertex[cur])) < cost[cur])
            {
                parent[cur] = par; 
                cost[cur] = cost[par] + pnt::euclidianDistanceSqrt(vertex[par], vertex[cur]);            
            }

        }  

    } 

    PathTree optimizedTree(vertex,cost,parent,corners);
    optimizedTree.setGoal(goal);
    return optimizedTree;
}



vector<int> findClosestNodesInRange(pnt::Point origin, vector<pnt::Point> &nodes,vector<vector<bool>> &grid, double radious, int x1, int y1, int x2, int y2)
{
    vector<int>nearby;
    for(int i=0; i<nodes.size(); i++)
    {
        if(pnt::euclidianDistance(origin,nodes[i])<=radious*radious && obstacleFree(grid,nodes[i],origin))
        {
            nearby.push_back(i);
        }
    }
    return nearby;
}

}