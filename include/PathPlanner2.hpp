#pragma once 
#include "Point.hpp"
#include "Grid.hpp"
#include "Node.hpp"

#include <vector>
#include <list>
#include <iostream>
using namespace std;

class PathPlanner2 
{
private:
    Grid g;
    Point goal;
    Point start;

    double move_cost = 1; 
    double rotate_cost = 3;
    double obs_cost = 3;
    double move_time = 1;
    double rotate_time = 5;

public: 
    double START_cost = 2;
    double STOP_cost = 2;
    double ROTATE_cost = 3;
    double CONTINUE_cost = 1;
    double OBSTACLE_cost = 0.5;

    vector<Node> path;

    PathPlanner2(Point Start_point, Point Goal_point, Grid grid);

    vector<Node> findPath();

    bool rotateNeed(Node bestNode, Point neighbor);
    bool nearObstacle(Point neighbor);
    double COST_calculator(Node& bestNode, Point neighbor);
    double normalizeAngle(double angle);
};

inline string stateToString(Node::State s) {
    switch(s) {
        case Node::START:    return "START";
        case Node::CONTINUE: return "CONTINUE";
        case Node::STOP:     return "STOP";
        case Node::ROTATE:   return "ROTATE";
        default:             return "UNKNOWN";
    }
}