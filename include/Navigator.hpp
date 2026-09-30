/**
 * @file Navigator.hpp
 * @brief Path navigation and state management for robot using Node-based path
 * 
 * Tracks robot progress along a path of Nodes, handles waypoint transitions,
 * and manages robot states based on Node states.
 */

#pragma once

#include "Grid.hpp"
#include "RobotState.hpp"
#include "Node.hpp"
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Navigator {
private:
    Grid g;
    vector<Node> path;           // Path of Nodes instead of Points
    int target_index;            // Current node index
    Robot_State robotState;                 // Robot state
    double pos_tolerance;
    double ang_tolerance;
    
public:

    Navigator(Grid grid, std::vector<Node> Path, 
              double pos_tol = 0.02, double ang_tol = 0.01);


    void update(Point position);

    // Getters
    Robot_State getState() const;
    Point getTarget() const;
    bool isFinished() const;
    int getIndex() const;
    Node getCurrentNode() const;

private:

    bool isRotationComplete(double current_angle, double target_angle);


    bool isPositionReached(Point current, Point target);


    double normalizeAngle(double angle);

    Robot_State mapNodeStateToRobotState(Node::State node_state);
};