#pragma once
#include "Point.hpp"

struct Node
{
    Point position;

    enum State {STOP, START, CONTINUE, ROTATE};


    double g;
    double h;
    double f;
    //double cost;
    State state;
    Point parent;

    double pointer; //theta
    


    bool operator==(const Node& node) {
        return (position.x == node.position.x && position.y == node.position.y);
    }

    bool operator!=(const Node& node) {
        return (position.x != node.position.x || position.y != node.position.y);
    }
};