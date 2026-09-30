#include "Navigator.hpp"
#include <cmath> 
#include <iostream>

Navigator::Navigator(Grid grid, Point Start, Point Goal, double pos_tol, double ang_tol) :
    g(grid), start(Start), goal(Goal), target_index(0), 
    robot_state(Robot_State::MOVING), pos_tolerance(pos_tol), ang_tolerance(ang_tol) {}

void Navigator::setPath(const vector<Node>& new_path) {
    path = new_path;
    target_index = 0;
    robot_state = Robot_State::MOVING;
    
    if (!path.empty()) {
        Node first_node = path[0];
        if (first_node.state == Node::ROTATE) {
            robot_state = Robot_State::ROTATING;
        // } else if (first_node.state == Node::STOP) {
        //     robot_state = Robot_State::STOPPING;
        } else {
            robot_state = Robot_State::MOVING;
        }
    }
    
    cout << "Path set with " << path.size() << " nodes. Starting state: " 
         << robot_state << endl;
}

void Navigator::update(Point position) {
    if (path.empty() || target_index >= path.size()) {
        robot_state = Robot_State::FINISHED;
        return;
    }

    Node target = path[target_index];
    Node::State node_state = target.state;

    double dx = target.position.x - position.x;
    double dy = target.position.y - position.y;
    double error_distance = sqrt(dx * dx + dy * dy);
    
    double target_angle = atan2(dy, dx);
    double error_angle = target_angle - position.theta;
    error_angle = normalizeAngle(error_angle);

    switch(node_state) {
        case Node::ROTATE: {
            robot_state = Robot_State::ROTATING;
            if (fabs(error_angle) < ang_tolerance) {
                target_index++;

                if (target_index < path.size()) {
                    Node next = path[target_index];
                    if (next.state == Node::START) {
                        robot_state = Robot_State::MOVING;
                    } else if (next.state == Node::STOP) {
                        robot_state = Robot_State::MOVING;
                    }
                } else {
                    robot_state = Robot_State::FINISHED;
                }
            }
            break;
        }
        
        case Node::STOP: {
            robot_state = Robot_State::MOVING;
            if (error_distance < pos_tolerance) {
                target_index++;
                
                if (target_index < path.size()) {
                    Node next = path[target_index];
                    if (next.state == Node::ROTATE) {
                        robot_state = Robot_State::ROTATING;
                    } else if (next.state == Node::START || next.state == Node::CONTINUE) {
                        robot_state = Robot_State::MOVING;
                    }
                } else {
                    robot_state = Robot_State::FINISHED;
                }
            }
            break;
        }
        
        case Node::START: {
            robot_state = Robot_State::MOVING;
            
            if (error_distance < pos_tolerance) {
                target_index++;
                
                if (target_index < path.size()) {
                    Node next = path[target_index];
                    if (next.state == Node::STOP) {
                        robot_state = Robot_State::MOVING;
                    }
                } else {
                    robot_state = Robot_State::FINISHED;
                }
            }
            break;
        }
        
        case Node::CONTINUE: {
            robot_state = Robot_State::MOVING;
            
            if (error_distance < pos_tolerance) {
                target_index++;
                
                if (target_index < path.size()) {
                    Node next = path[target_index];
                    if (next.state == Node::ROTATE) {
                        robot_state = Robot_State::ROTATING;
                    } else if (next.state == Node::STOP) {
                        robot_state = Robot_State::MOVING;
                    }
                } else {
                    robot_state = Robot_State::FINISHED;
                }
            }
            break;
        }
        
        default:
            robot_state = Robot_State::FINISHED;
            break;
    }
    
    // Debug output
    // cout << "Target: (" << target.position.x << "," << target.position.y 
    //      << ") | State: " << node_state 
    //      << " | Robot State: " << robot_state 
    //      << " | Index: " << target_index << "/" << path.size() << endl;
}


double Navigator::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool Navigator::needToRotate(Point current, Point next, Point robot) {
    double current_angle = atan2(current.y - robot.y, current.x - robot.x);
    double next_angle = atan2(next.y - current.y, next.x - current.x);
    double diff = next_angle - current_angle;
    diff = normalizeAngle(diff);
    return fabs(diff) > ang_tolerance;
}

Robot_State Navigator::getState() const {
    return robot_state;
}

Node Navigator::getTarget() const {
    if (target_index < path.size()) {
        return path[target_index];
    }
    return path.back();
}

Node Navigator::getCurrentNode() const {
    if (target_index < path.size()) {
        return path[target_index];
    }
    return path.back();
}

bool Navigator::isFinished() const {
    return (robot_state == Robot_State::FINISHED || 
            target_index >= path.size());
}

int Navigator::getIndex() const {
    return target_index;
}