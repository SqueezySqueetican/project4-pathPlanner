#include "Point.hpp"
#include "Path.hpp"
#include "Grid.hpp"
#include "RobotState.hpp"
#include "PID.hpp"
#include "Simulator.hpp"
#include "Navigator.hpp"
#include "Controller.hpp"
#include "PathPlanner2.hpp"

#include <fstream>
#include <iostream>
#include <iomanip>

int main() {
    // ============================================
    // 1. INITIALIZATION
    // ============================================
    
    // Create grid (10x10 with 1m spacing)
    Grid grid(6, 6, 1);
    
    // ============================================
    // 2. DEFINE START AND GOAL
    // ============================================

    Point start; start.x = 1; start.y = 0; start.theta = M_PI /2;

    Point goal; goal.x = 4; goal.y = 4; goal.theta = M_PI / 2;

    Point obs0{2, 2}; grid.obstacle(obs0);
    Point obs1{1, 1}; grid.obstacle(obs1);
    //Point obs3{2, 5}; grid.obstacle(obs3);
    Point obs4{3, 5}; grid.obstacle(obs4);
    //Point obs6{1, 3}; grid.obstacle(obs6);
    Point obs7{1, 4}; grid.obstacle(obs7);
    Point obs8{3, 2}; grid.obstacle(obs8);
    Point obs9{2, 0}; grid.obstacle(obs9);
    Point obs10{1, 2}; grid.obstacle(obs10);
    Point obs11{1, 5}; grid.obstacle(obs11);
    Point obs12{2, 4}; grid.obstacle(obs12);


    

    
    // ============================================
    // 3. PLAN PATH USING A* WITH STATES
    // ============================================

    cout << "=== Path Planning ===" << endl;
    PathPlanner2 path_planner2(start, goal, grid);
    vector<Node> path = path_planner2.findPath();

    cout << "\n=== Planned Path ===" << endl;
    for (size_t i = 0; i < path.size(); i++) {
        Node node = path[i];
        cout << "Step " << i << ": ";
        cout << "(" << node.position.x << ", " << node.position.y << ")  \t";
        cout << "| state: " << stateToString(node.state) << "   \t";
        cout << "| pointer: " << node.pointer * 180/M_PI << endl;
    }
    cout << "Total steps: " << path.size() << endl;

    // grid displaying
    cout << "\n=== Grid ===" << endl;
    for (int j = grid.h - 1; j >= 0; j--) {
        for (int i = 0; i < grid.w; i++) {
            Point p; p.x = i; p.y = j;

            bool isOnPath = false;
            for (Node node : path) {
                if (p.x == node.position.x && p.y == node.position.y) {
                    isOnPath = true;
                    break;
                }
            }

            bool isObstacle = false;
            for (Point obs : grid.obstacle_dots) {
                if (p.x == obs.x && p.y == obs.y) {
                    isObstacle = true;
                    break;
                }
            }

            if (p.x == start.x && p.y == start.y) {
                cout << "S ";
            } else if (p.x == goal.x && p.y == goal.y) {
                cout << "G ";
            } else if (isObstacle) {
                cout << "X ";
            } else if (isOnPath) {
                cout << "* ";
            } else {
                cout << ". ";
            }
        }
        cout << endl;
    }
    
    // ============================================
    // 4. INITIALIZE NAVIGATOR
    // ============================================
    
    Navigator navigator(grid, path, 0.02, 0.01);
    
    // ============================================
    // 5. INITIALIZE ROBOT AND CONTROLLER
    // ============================================
    
    RobotState robot_params;
    Simulator robot(robot_params);
    
    // Set initial robot pose
    robot.reset(start, Twist{0.0, 0.0});
    
    // PID gains
    double dt = 0.01;
    double kp_lin = 1.0;
    double ki_lin = 0.0;
    double kd_lin = 0.0;
    
    double kp_ang = 2.5;
    double ki_ang = 0.0;
    double kd_ang = 0.0;
    
    Controller controller(kp_lin, ki_lin, kd_lin,
                          kp_ang, ki_ang, kd_ang,
                          dt);
    
    // ============================================
    // 6. CSV OUTPUT FILE
    // ============================================
    
    ofstream file("4robot_path_planner.csv");
    if (!file.is_open()) {
        cout << "ERROR: Could not open CSV file!" << endl;
        return -1;
    }
    
    // CSV header
    file << "Time,x,y,theta,target_x,target_y,target_angle,"
         << "error_dist,error_angle,cmd_v,cmd_omega,"
         << "current_v,current_omega,robot_state,node_state\n";
    
    // ============================================
    // 7. SIMULATION LOOP
    // ============================================
    
    const int MAX_ITERATIONS = 10000;
    const double PRINT_INTERVAL = 1;  // Print every 1 second
    
    cout << "========================================" << endl;
    cout << "SIMULATION STARTED" << endl;
    cout << "========================================" << endl;
    
    for (int i = 0; i < MAX_ITERATIONS; i++) {
        // Get current robot pose
        Point position = robot.getPose();
        
        // Update navigator
        navigator.update(position);
        
        // Check if mission complete
        if (navigator.isFinished()) {
            cout << "\n Mission Complete!" << endl;
            cout << "Final position: (" << position.x << ", " 
                 << position.y << ") theta: " 
                 << position.theta * 180/M_PI  << endl;
            break;
        }
        
        // Get target and state
        Point target = navigator.getTarget();
        // Robot_State robotState
        Robot_State robotState = navigator.getState();
        Node current_node = navigator.getCurrentNode();
        
        // Calculate errors for logging
        double dx = target.x - position.x;
        double dy = target.y - position.y;
        double error_distance = sqrt(dx*dx + dy*dy);
        
        double target_angle = 0.0;
        if (robotState == ROTATING) {
            target_angle = current_node.pointer;  // Use pointer from Node
        } else {
            target_angle = atan2(dy, dx);
        }
        double error_angle = target_angle - position.theta;
        // Normalize angle
        while (error_angle > M_PI) error_angle -= 2.0 * M_PI;
        while (error_angle < -M_PI) error_angle += 2.0 * M_PI;
        
        // Compute velocity command
        Twist cmd = controller.computeCommand(position, target, robotState, target_angle);
        
        // Apply command to robot
        robot.setVelocityCommand(cmd);
        robot.update(dt);
        
        // Get new state after update
        Point new_pose = robot.getPose();
        Twist new_vel = robot.getVelocity();
        
        // ========================================
        // 8. LOG DATA
        // ========================================
        
        double time = i * dt;
        file << time << ","
             << new_pose.x << ","
             << new_pose.y << ","
             << new_pose.theta << ","
             << target.x << ","
             << target.y << ","
             << target_angle << ","
             << error_distance << ","
             << error_angle << ","
             << cmd.linear_x << ","
             << cmd.angular_z << ","
             << new_vel.linear_x << ","
             << new_vel.angular_z << ","
             << robotState << ","
             << (current_node.state) << "\n";
        
        // ========================================
        // 9. PRINT PROGRESS
        // ========================================
        
        if (i % int(PRINT_INTERVAL / dt) == 0 || i == 0) {
            string state_str;
            switch(robotState) {
                case MOVING:   state_str = "MOVING"; break;
                case STOPPING: state_str = "STOPPING"; break;
                case ROTATING: state_str = "ROTATING"; break;
                case FINISHED: state_str = "FINISHED"; break;
                default:       state_str = "UNKNOWN"; break;
            }
            
            cout << fixed << setprecision(3)
                 << "Time: " << setw(6) << time << "s"
                 << " | Pos: (" << setw(5) << new_pose.x << ", " 
                 << setw(5) << new_pose.y << ")"
                 << " | Target: (" << setw(5) << target.x << ", " 
                 << setw(5) << target.y << ")"
                 << " | State: " << setw(8) << state_str
                 << " | Node: " << setw(8) << stateToString(current_node.state)
                 << " | Dist err: " << setw(6) << error_distance
                 << " | Ang err: " << setw(6) << error_angle * 180/M_PI << "°"
                 << endl;
        }
    }
    
    // ============================================
    // 10. CLOSE AND FINISH
    // ============================================
    
    file.close();
    
    cout << "========================================" << endl;
    cout << "SIMULATION COMPLETE" << endl;
    cout << "Data saved to: robot_path_planner.csv" << endl;
    cout << "========================================" << endl;
    
    return 0;
}