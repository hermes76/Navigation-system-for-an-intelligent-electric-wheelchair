#ifndef PGM
#define PGM
#include <iostream>
#include <vector>
#include "point.h"
using namespace std;

namespace pgm{
class Pgm
{
    private:
        string name;
        int width;
        int height;
    public:
        vector<vector<bool>> data;
        Pgm();
        Pgm(string name);
        void readImage(string);
        void cutTrash();
        int getWidth();
        int getHeight();
        vector<vector<bool>> getData();
        void setWidth(int witdth);
        void setHeight(int height);
        void setData(vector<vector<bool>> data);
        void inflateGrid(int w, int h);

};

}
#endif