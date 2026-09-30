#pragma once 
#include "Point.hpp"
#include "Grid.hpp"
#include "Node.hpp"
///#include ""


#include <vector>
#include <list>
#include <iostream>
using namespace std;

class PathPlanner 
{
private:
    Grid g;
    Point goal;
    Point start;

    double move_cost = 1; 
    double rotate_cost  = 3;

    double obs_cost  = 3;
    double move_time = 1;
    double rotate_time = 5;

    
    
public: 


    //enum State { START, STOP, ROTATE, CONTINUE};
    double START_cost = 2;
    double STOP_cost = 2;
    double ROTATE_cost = 3;
    double CONTINUE_cost = 1;
    double OBSTACLE_cost = 0.5;

    list<Node> path;
    list<Node::State> steps;

    PathPlanner(Point Start_point, Point Goal_point, Grid grid);

    list<Node> findPath();

    list<Node::State> getSteps() { return steps; }

    bool rotateNeed(Node bestNode, Point neighbor);
    //bool stopNeed(Node bestNode, Point neighbor);
    bool nearObstacle(Point neighbor);

    double COST_calculator(Node& bestNode, Point neighbor);  // ← reference



    double normalizeAngle(double angle);


    //vector<State> calculateStates(list<Point>& path);

};