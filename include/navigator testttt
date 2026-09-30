#include "Navigator.hpp"
#include <cmath> 
#include <iostream>

Navigator::Navigator(Grid grid, vector<Point> Path, double pos_tol, double ang_tol) 
    : g(grid), path(Path), target_index(0), state(MOVING), 
      pos_tolerance(pos_tol), ang_tolerance(ang_tol) {}

void Navigator::update(Point position) {

    Point target = path[target_index];

    // Obstacle check
    // if (!g.isInGrid(target)) {
    //     target = stopBeforeObstacle(target, position); 
    // }

    double dx = target.x - position.x;
    double dy = target.y - position.y;
    double error_distance = sqrt(dx * dx + dy * dy);

    // State transitions
    if (state == MOVING) {
        if (error_distance < pos_tolerance) {
            target_index++;
            if (target_index >= path.size()) {
                state = FINISHED;
            } else {
                Point next = path[target_index];
                if (needToRotate(target, next, position)) {
                    state = ROTATING;
                }
            }
        }
    }
    else if (state == ROTATING) {
        Point next = path[target_index];
        double dx_next = next.x - position.x;
        double dy_next = next.y - position.y;
        double next_angle = atan2(dy_next, dx_next);
        double error = next_angle - position.theta;
        error = normalizeAngle(error);

        if (fabs(error) < ang_tolerance) {
            state = MOVING;  // Rotation complete, resume movement
        }
    }
}

//check angle between robot position and target -> command rotation if its needed
bool Navigator::needToRotate(Point current, Point next, Point robot) {
    double current_angle = atan2(current.y - robot.y, current.x - robot.x);
    double next_angle = atan2(next.y - current.y, next.x - current.x);
    double diff = next_angle - current_angle;
    diff = normalizeAngle(diff);
    return fabs(diff) > ang_tolerance;
}

double Navigator::normalizeAngle(double angle) {
    while (angle > 2.0 * M_PI) angle -= 2.0 * M_PI;
    while (angle < - 2.0 * M_PI) angle += 2.0 * M_PI;
    if(fabs(angle - 2.0 * M_PI) < ang_tolerance) { angle = 0; }
    return angle;
}

// getters
State Navigator::getState()  {
    return state;
}

Point Navigator::getTarget()  {
    if (target_index < path.size())
        return path[target_index];
    return path.back();
}

bool Navigator::isFinished()  {
    return state == FINISHED;
}

int Navigator::getIndex()  {
    return target_index;
}


// findBestPoint: finds common neighbors between robot and obstacle,
// then selects point closest to nextGoal
// Point Navigator::findBestPoint(Point obstacle, Point robot, Point nextGoal) {
//     list<Point> obs_neighbors = g.findNeighbors(obstacle);
//     list<Point> rob_neighbors = g.findNeighbors(robot);
    
//     list<Point> commonPoints;
//     for (Point o : obs_neighbors) {
//         for (Point r : rob_neighbors) {
//             if (o.x == r.x && o.y == r.y) {
//                 commonPoints.push_back(o);
//             }
//         }
//     }
    
//     if (commonPoints.empty()) {
//         cout << "No common point found between robot and obstacle neighbors!" << endl; 
//     }
    
//     Point bestPoint = commonPoints.front();
//     double bestScore = 1000.0;
    
//     for (Point p : commonPoints) {
//         if (!g.isInGrid(p)) continue;
        
//         double dx_goal = p.x - nextGoal.x;
//         double dy_goal = p.y - nextGoal.y;
//         double distToGoal = sqrt(dx_goal*dx_goal + dy_goal*dy_goal);
        
//         if (distToGoal < bestScore) {
//             bestScore = distToGoal;
//             bestPoint = p;
//         }
//     }
    
//     return bestPoint;
// }

// Point Navigator::stopBeforeObstacle(Point obstacle, Point robot) {
//     list<Point> obs_neighbors = g.findNeighbors(obstacle);
    
//     Point bestPoint = robot;
//     double bestScore = 1000.0;
    
//     for (Point p : obs_neighbors) {
//         if (!g.isInGrid(p)) continue;
        
//         double dx = p.x - robot.x;
//         double dy = p.y - robot.y;
//         double distToRobot = sqrt(dx*dx + dy*dy);
        
//         if (distToRobot < bestScore) {
//             bestScore = distToRobot;
//             bestPoint = p;
//         }
//     }  
//     cout << "(" << bestPoint.x << ", " << bestPoint.y << ")\n";
//     return bestPoint;
// }