#include "Point.hpp"
#include "Grid.hpp"
#include "RobotState.hpp"
#include "PID.hpp"
#include "Simulator.hpp"
#include "Navigator.hpp"
#include "Controller.hpp"
#include "PathPlanner2.hpp"

#include <iomanip>  
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

int main() {

    Grid grid(6, 6, 1);

    Point start; start.x = 0; start.y = 0; start.theta = M_PI;

    Point goal; goal.x = 4; goal.y = 4; //goal.theta = M_PI / 2;

    Point obs0{3, 0}; grid.obstacle(obs0);
    Point obs1{1, 1}; grid.obstacle(obs1);
    Point obs6{1, 3}; grid.obstacle(obs6);
    Point obs7{1, 4}; grid.obstacle(obs7);
    Point obs8{3, 2}; grid.obstacle(obs8);
    Point obs9{2, 0}; grid.obstacle(obs9);


    cout << "=== Path Planning ===" << endl;
    PathPlanner2 path_planner2(start, goal, grid);
    vector<Node> fullPath = path_planner2.findPath();


    cout << "\n=== Planned Path ===" << endl;
    for (size_t i = 0; i < fullPath.size(); i++) {
        Node node = fullPath[i];
        cout << "Step " << i << ": ";
        cout << "(" << node.position.x << ", " << node.position.y << ")  \t";
        cout << "| state: " << stateToString(node.state) << "   \t";
        cout << "| pointer: " << node.pointer * 180/M_PI << endl;
    }
    cout << "Total steps: " << fullPath.size() << endl;

    // grid displaying
    cout << "\n=== Grid ===" << endl;
    for (int j = grid.h - 1; j >= 0; j--) {
        for (int i = 0; i < grid.w; i++) {
            Point p; p.x = i; p.y = j;

            bool isOnPath = false;
            for (Node node : fullPath) {
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


    RobotState parameters;
    Simulator robot(parameters);
    robot.reset(start, {0,0});  

    double dt = 0.01;

    Controller controller(1.0, 0.0, 0.0,    // linear PID
                          2.5, 0.0, 0.0,    // angular PID
                          dt);
    
    Navigator navigator(grid, start, goal, 0.0002, 0.00002);
    navigator.setPath(fullPath);


    ofstream file("17robot_path_data.csv");
    if (!file.is_open()) {
        cout << "Error: Cannot open file!" << endl;
        return -1;
    }

    /////// 
    file << std::fixed << std::setprecision(6); 

    file << "Time,PosX,PosY,PosTheta,TargetX,TargetY,TargetState,"
         << "TargetPointer,ErrorDistance,ErrorAngle,CommandLinear,CommandAngular,"
         << "ActualLinear,ActualAngular,RobotState,NodeState,TargetIndex\n";

    //simulation loop
    cout << "\n=== Simulation Started ===" << endl;

    for (int i = 0; i < 5000; i++) {
        Point position = robot.getPose();

        navigator.update(position);
        
        if (navigator.isFinished()) {
            cout << "Mission Complete!\n";
            break;
        }

        Node target = navigator.getTarget();
        Robot_State robot_state = navigator.getState();
        int current_index = navigator.getIndex();

        Point target_pos; 
        target_pos.x = target.position.x; target_pos.y = target.position.y; target_pos.theta = target.pointer; 

        double dx = target_pos.x - position.x;
        double dy = target_pos.y - position.y;
        double error_distance = sqrt(dx * dx + dy * dy);
        double target_angle = atan2(dy, dx);
        double error_angle = target_angle - position.theta;
        error_angle = navigator.normalizeAngle(error_angle);

        Twist cmd = controller.computeCommand(position, target, robot_state);

        robot.setVelocityCommand(cmd);
        robot.update(dt);

        Point new_pose = robot.getPose();
        Twist new_vel = robot.getVelocity();

        double time = i * dt;
        file << time << ","
             << new_pose.x << ","
             << new_pose.y << ","
             << new_pose.theta * 180 / M_PI << ","
             << target_pos.x << ","
             << target_pos.y << ","
             << target.state << ","
             << target.pointer * 180 / M_PI << ","
             << error_distance << ","
             << error_angle * 180 / M_PI << ","
             << cmd.linear_x << ","
             << cmd.angular_z << ","
             << new_vel.linear_x << ","
             << new_vel.angular_z << ","
             << robot_state << ","
             << target.state << ","
             << current_index << "\n";

        // if (i % 10 == 0) {
        //     cout << "Time: " << time << "s "
        //          << "| Pos: (" << new_pose.x << ", " << new_pose.y << ")\t"
        //          << "| Target: (" << target_pos.x << "," << target_pos.y << ")\t"
        //          << "| Node State: " << stateToString(target.state) << "\t"
        //          << "| Robot State: " << robot_state << "\t"
        //          << "| Index: " << current_index << "/" << fullPath.size() << endl;
        // }
    }


    file.close();

    cout << "\n=== Simulation Complete ===" << endl;
    cout << "Data saved to: robot_path_data.csv" << endl;
    
    Point final_pose = robot.getPose();
    cout << "Final Position: (" << final_pose.x << ", " << final_pose.y << ")" << endl;
    cout << "Final Angle: " << final_pose.theta * 180/M_PI << "°" << endl;

    return 0;
}