#include <bits/stdc++.h>
#include "include/pgm.h"

using namespace std;
namespace pgm{
Pgm::Pgm()
{

}

Pgm::Pgm(string name)
{
    readImage(name);
}
void Pgm::readImage(string name)
{
    ifstream file(name);
    string trash;
    double ocuped_fresh=0.65;


    for(int x=0; x<6; x++)
        file>>trash;

    file>>this->width>>this->height;
    file>>trash;
    
    unsigned char byte;
    int value;

    this->data.resize(this->height);
    
    for(int x=0; x<this->height; x++)
    {
        for(int y=0; y<this->width; y++)
        {
            file>>byte;
            value=byte;
            if((255-value)/255.0>=ocuped_fresh)
                this->data[x].push_back(true);
            else this->data[x].push_back(false);
        }
    }
    file.close();
}

int Pgm::getWidth()
{
    return this->width;
}
int Pgm::getHeight()
{
    return this->height;
}
vector<vector<bool>> Pgm::getData()
{
    return this->data;
}

void Pgm::setWidth(int witdth)
{
    this->width=width;
}
void Pgm::setHeight(int height)
{
    this->height=height;
}
void Pgm::setData(vector<vector<bool>> data)
{
    this->data=data;
}
void Pgm::cutTrash()
{
    vector<vector<bool>> newData;
    int x1,x2,y1,y2;
    x1=this->width-1;
    y1=this->height-1;
    x2=y2=0;
    for(int y=0; y<this->height; y++)
    {
        for(int x=0; x<this->width; x++)
        {
            if(this->data[y][x])
            {
                x1=min(x1,x);
                x2=max(x,x2);
                y1=min(y,y1);
                y2=max(y2,y);
            }
        }
    }
    int newHeight=(y2-y1+1);
    int newWidth =(x2-x1+1);
    int row=0;

    newData.resize(newHeight);
    for(int y=y1; y<=y2; y++)
    {
        for(int x=x1; x<=x2; x++)
        {
            newData[row].push_back(this->data[y][x]);
        }
        row++;
    }
    this->width=newWidth;
    this->height=newHeight;
    this->data=newData;
}
void Pgm:: inflateGrid(int w, int h)
{
    vector<vector<bool>> originalGrid=this->data;
    for(int i =0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(originalGrid[i][j])
            {
                int ly,ry,lx,rx;
                ly=max(0,i-h);
                ry=min(this->height,i+h);
                lx=max(j-w,0);
                rx=min(j+w,this->width);
                for(int y=ly; y<ry; y++)
                {
                    for(int x=lx; x<rx; x++)
                    {
                        this->data[y][x]=true;
                    }
                }
            }
        }
    }
}

}
