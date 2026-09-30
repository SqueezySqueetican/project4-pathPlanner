#pragma once

#include <cmath>
#include <list>
#include "Point.hpp"
#include <iostream>

using namespace std;

class Grid {
public:

    int w, h;            ///< weight and high of grid
    double dist;         ///< Distance between adjacent points
    list<Point> grid_dots;
    list<Point> obstacle_dots;

    Grid(int width, int height, double distance);

    bool isInGrid(Point point);

    void obstacle(Point obstacle_point);

    list<Point> findNeighbors(Point point); 

    list<Point> allNeighbors(Point point);


    //void show();

    
};