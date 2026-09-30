#include "Navigator.hpp"
#include <cmath> 
#include <iostream>



// Constructor
Navigator::Navigator(Grid grid, std::vector<Node> Path, double pos_tol, double ang_tol) 
: g(grid), path(Path), target_index(0), pos_tolerance(pos_tol), ang_tolerance(ang_tol) {}

// Main update method
void Navigator::update(Point position) {
    // Check if path is empty or already finished
    if (path.empty() || robotState == FINISHED) {
        robotState = FINISHED;
        return;
    }

    // Get current node
    Node current_node = path[target_index];
    Point target = current_node.position;

    // Calculate errors
    double dx = target.x - position.x;
    double dy = target.y - position.y;
    double error_distance = sqrt(dx * dx + dy * dy);
    
    double target_angle = current_node.pointer;
    double error_angle = normalizeAngle(target_angle - position.theta);

    // ============================================
    // STATE MANAGEMENT BASED ON NODE STATE
    // ============================================
    
    switch (current_node.state) {
        
        // ========================================
        // ROTATE: Rotate in place to target angle
        // ========================================
        case Node::ROTATE: {
            robotState = ROTATING;
            
            // Check if rotation is complete
            if (fabs(error_angle) < ang_tolerance) {
                // Rotation complete, move to next node
                target_index++;
                if (target_index >= path.size()) {
                    robotState = FINISHED;
                } else {
                    // Check if next node also needs rotation
                    Node next_node = path[target_index];

                    // Start moving or stopping based on next node
                    robotState = mapNodeStateToRobotState(next_node.state);

                }
            }
            break;
        }
        
        // ========================================
        // STOP: Stop at this point
        // ========================================
        case Node::STOP: {
            robotState = STOPPING;
            
            // Check if reached the stop position
            if (error_distance < pos_tolerance) {
                // Reached stop point, move to next node
                target_index++;
                if (target_index >= path.size()) {
                    robotState = FINISHED;
                } else {
                    // Move to next state
                    Node next_node = path[target_index];
                    robotState = mapNodeStateToRobotState(next_node.state);
                }
            }
            break;
        }
        
        // ========================================
        // CONTINUE: Keep moving straight
        // ========================================
        case Node::CONTINUE: {            
            target_index++;
            if (target_index >= path.size()) {
                robotState = FINISHED;
            } 
            update(position);  // ← recursive call
            return;
            
            
            // robotState = MOVING;
            
            // // Check if reached this node position
            // if (error_distance < pos_tolerance) {
            //     // Reached waypoint, move to next
            //     target_index++;
            //     if (target_index >= path.size()) {
            //         robotState = FINISHED;
            //     } else {
            //         // Update state based on next node
            //         Node next_node = path[target_index];
            //         robotState = mapNodeStateToRobotState(next_node.state);
            //     }
            // }
            break;
        }
        
        // ========================================
        // START: Start moving toward next node
        // ========================================
        case Node::START: {
            robotState = MOVING;
            
            // Check if reached this node position
            if (error_distance < pos_tolerance) {
                // Reached start point, move to next
                target_index++;
                if (target_index >= path.size()) {
                    robotState = FINISHED;
                } else {
                    // Update state based on next node
                    Node next_node = path[target_index];
                    robotState = mapNodeStateToRobotState(next_node.state);
                }
            }
            break;
        }
        
        // ========================================
        // Default / Unknown
        // ========================================
        default: {
            robotState = MOVING;
            break;
        }
    }
}

// Helper: Check if rotation is complete
bool Navigator::isRotationComplete(double current_angle, double target_angle) {
    double error = normalizeAngle(target_angle - current_angle);
    return fabs(error) < ang_tolerance;
}

// Helper: Check if position is reached
bool Navigator::isPositionReached(Point current, Point target) {
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    return sqrt(dx*dx + dy*dy) < pos_tolerance;
}

// Map Node::State to Robot State
Robot_State Navigator::mapNodeStateToRobotState(Node::State node_state) {
    switch(node_state) {
        case Node::ROTATE:   return ROTATING;
        case Node::STOP:     return MOVING;
        case Node::START:    return MOVING;
        case Node::CONTINUE: return MOVING;
        default:             return MOVING;
    }
}

// Normalize angle to [-PI, PI]
double Navigator::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ============================================
// GETTERS
// ============================================

Robot_State Navigator::getState() const {
    return robotState;
}

Point Navigator::getTarget() const {
    if (target_index < path.size()) {
        return path[target_index].position;
    }
    return path.empty() ? Point{} : path.back().position;
}

bool Navigator::isFinished() const {
    return robotState == FINISHED;
}

int Navigator::getIndex() const {
    return target_index;
}

Node Navigator::getCurrentNode() const {
    if (target_index < path.size()) {
        return path[target_index];
    }
    return Node{}; // Return empty node if out of bounds
}