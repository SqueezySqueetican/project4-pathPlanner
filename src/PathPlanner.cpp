#include "PathPlanner.hpp"
#include <cmath>
#include <iostream>

PathPlanner::PathPlanner(Point Start_point, Point Goal_point, Grid grid) : start(Start_point), goal(Goal_point), g(grid), steps(){
    //list<Node::State> steps;
}

list<Node> PathPlanner::findPath() {

    

    if (!g.isInGrid(start)) {
        cout << "Start point is not in grid!" << endl;
        return {};
    }
    if (!g.isInGrid(goal)) {
        cout << "Goal point is not in grid!" << endl;
        return {};
    }

    if (start.x == goal.x && start.y == goal.y) {
        cout << "Start and goal are the same!" << endl;
        return {};
    }

    Node start_node;
    start_node.position = start;
    start_node.parent = start;
    start_node.g = 0;
    start_node.h = abs(start.x - goal.x) + abs(start.y - goal.y);
    start_node.f = start_node.g + start_node.h;
    start_node.pointer = start.theta;

    Node goal_node;
    goal_node.position = goal;
    //start_node.parent = start;
    goal_node.pointer = goal.theta;

    list<Node> openSet;
    list<Node> closedSet;
    list<Node> path;

    openSet.push_back(start_node);

    //cout << "COSTS: node: 1   |  rotate: 5   |  near obs = 3 " << endl;

    while (!openSet.empty()) {
        Node bestNode = openSet.back();
        for (const Node& n : openSet) {
            if (n.f < bestNode.f) {
                bestNode = n;
            }
        }

        openSet.remove(bestNode);
        closedSet.push_front(bestNode);

        cout << "Exploring: (" << bestNode.position.x << "," << bestNode.position.y << ") | f=" << bestNode.f;

        string stateName;
    switch(bestNode.state) {
        case Node::START:    stateName = "START"; break;
        case Node::STOP:     stateName = "STOP"; break;
        case Node::CONTINUE: stateName = "CONTINUE"; break;
        case Node::ROTATE:   stateName = "ROTATE"; break;
        default:             stateName = "UNKNOWN";
    }
    cout << " | state=" << stateName << endl;


        if (bestNode.position == goal) {
        cout << "Goal position reached! Checking orientation..." << endl;
        
        double diff = normalizeAngle(bestNode.pointer - goal.theta);
        
        if (fabs(diff) < 0.01) {
            cout << "Goal reached with correct orientation!" << endl;
        } else {
            cout << "Adding rotation at goal: " << diff * 180/M_PI << " degrees" << endl;
            
            bestNode.pointer = goal.theta;
        }
            
            while (!(bestNode.position == start)) {
                path.push_front(bestNode);
                
                bool found = false;
                for (const Node& n : closedSet) {
                    if (n.position.x == bestNode.parent.x && n.position.y == bestNode.parent.y) {
                        bestNode = n;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    cerr << "Parent not found in closedSet!" << endl;
                    break;
                }
            }
            path.push_front(start_node);
            
            cout << "Path found! Length: " << path.size() << endl;
            return path;


            /////////////

            
        }

        list<Point> neighbors = g.findNeighbors(bestNode.position);


        for ( Point neighborPos : neighbors) {
            bool inClosed = false;
            for ( Node n : closedSet) {
                if (n.position == neighborPos) {
                    inClosed = true;
                    break;
                }
            }

            if (inClosed) continue;

            double new_g;            

            double cost = COST_calculator(bestNode, neighborPos);
            new_g = bestNode.g + cost;

            //if(nearObstacle(neighborPos)) { new_g += 0.5; }

            bool inOpen = false;
            for (Node n : openSet) {
                if (n.position == neighborPos) {
                    inOpen = true;
                    if (new_g < n.g) {
                        n.g = new_g;
                        n.parent = bestNode.position;
                        n.h = abs(n.position.x - goal.x) + abs(n.position.y - goal.y);
                        //n.pointer = atan2(n.position.y - bestNode.position.y, n.position.x - bestNode.position.x);
                        n.pointer = atan2(neighborPos.y - bestNode.position.y, neighborPos.x - bestNode.position.x);
                        n.pointer = normalizeAngle(n.pointer);
                        n.state = bestNode.state; 
                        steps.push_back(n.state);


                        n.f = n.g + n.h;
                    }
                    break;
                }
            }

            if (!inOpen) {
                Node n;
                n.position = neighborPos;
                n.parent = bestNode.position;
                n.g = new_g;
                n.h = abs(neighborPos.x - goal.x) + abs(neighborPos.y - goal.y);
                n.f = n.g + n.h;
                n.state = bestNode.state;  
                steps.push_back(n.state);                                      
                n.pointer = atan2(neighborPos.y - bestNode.position.y, neighborPos.x - bestNode.position.x);
                n.pointer = normalizeAngle(n.pointer);


                openSet.push_back(n);
            }
        }        
    }

    cout << "No path found from start to goal!" << endl;
    return path;
    
}


bool PathPlanner::rotateNeed(Node bestNode, Point neighbor) {
    Point parent = bestNode.parent;
    // double current_angle;
    // if (bestNode.position == start && bestNode.parent == start) { 
    //     current_angle = start.theta; 
    // } else {
    //     Point parent = bestNode.parent;
    //     current_angle = atan2(bestNode.position.y - parent.y, bestNode.position.x - parent.x);
    // }
    double current_angle = bestNode.pointer;  //pointer

    double next_angle = atan2(neighbor.y - bestNode.position.y, neighbor.x - bestNode.position.x);
    double diff = normalizeAngle(next_angle - current_angle);
    return fabs(diff) > 0.01;
}


bool PathPlanner::nearObstacle(Point neighbor) {
    list<Point> neighbors_of_neighbor = g.allNeighbors(neighbor);
    bool near_obstacle = false;
    for (Point n : neighbors_of_neighbor) {
        if (!g.isInGrid(n)) {
            near_obstacle = true;
            break;
        }
    }
    return near_obstacle;
}

double PathPlanner::COST_calculator(Node& bestNode, Point neighbor) {
    double time = 0;
    
    if (bestNode.position == start) {
        if (rotateNeed(bestNode, neighbor)) {
            time += ROTATE_cost;
            bestNode.state = Node::ROTATE;
            //steps.push_back(Node::ROTATE);
        } else {
            time += START_cost;
            bestNode.state = Node::START;
            //steps.push_back(Node::START);
        }
    }
    
    else if (rotateNeed(bestNode, neighbor)) {
        time += STOP_cost;
        bestNode.state = Node::STOP;
        //steps.push_back(Node::STOP);
        
        time += ROTATE_cost;
        bestNode.state = Node::ROTATE;
        //steps.push_back(Node::ROTATE);
        
        time += START_cost;
        bestNode.state = Node::START;
        //steps.push_back(Node::START);
    }

    else if (neighbor == goal) {
        time += STOP_cost;
        bestNode.state = Node::STOP;
        //steps.push_back(Node::STOP);


        if (fabs(normalizeAngle(bestNode.pointer - goal.theta)) > 0.01) { // pointer
            time += ROTATE_cost;
            bestNode.state = Node::ROTATE;
            //steps.push_back(Node::ROTATE);
        }

        cout << "finish" << endl;
    }
    

    else {
        time += CONTINUE_cost;
        bestNode.state = Node::CONTINUE;
        //steps.push_back(Node::CONTINUE);
    }
    

    if (nearObstacle(neighbor)) {
        time += OBSTACLE_cost;
        //cout << "obs " << endl;
    }
    
    return time;
}


double PathPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI; 
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}





// double PathPlanner::time_calculator(Node bestNode, Point neighbor){
//     double time = 0; 
//     if (bestNode.position == start) {
//         if (!rotateNeed(bestNode, neighbor)) { 
//             time += START_cost;
//             bestNode.state = START;
//             //cout << "start from start point: " << time << endl;
//         }
//     }

//     else if (rotateNeed(bestNode, neighbor)) {
//         if (bestNode.position == start) {
//             time += ROTATE_cost;
//             bestNode.state = ROTATE;
//             //cout << "rotate: " << time << endl;
//             time += START_cost;
//             bestNode.state = START;
//             //cout << "start: " << time << endl;
//         } else {
//             time += STOP_cost;
//             bestNode.state = STOP;
//             //cout << "stop: " << time << endl;
//             time += ROTATE_cost;
//             bestNode.state = ROTATE;
//             //cout << "rotate: " << time << endl;
//             time += START_cost;
//             bestNode.state = START;
//             //cout << "start: " << time << endl;
//         }
//     }

//     else if(neighbor == goal) {
//         time += STOP_cost;
//         bestNode.state = STOP;
//         cout << "stop (goal reached): " << time << endl;
//     } else { 
//         time += CONTINUE_cost;
//         bestNode.state = CONTINUE;
//     }

//     return time;
// }




// vector<PathPlanner::State> PathPlanner::calculateStates(list<Point>& path) {

//     vector<State> states;
    
//     if (path.size() < 2) cout << "No way to goal" << endl;
    
//     vector<Point> points(path.begin(), path.end());
    
//     for (int i = 0; i < points.size() - 1; i++) {
//         Node current;
//         current.position = points[i];
//         current.parent = (i == 0) ? points[i] : points[i-1];
        
//         Point next = points[i + 1];
//         Point nextNext = (i + 2 < points.size()) ? points[i + 2] : next;
        
//         bool needRotateNow = rotateNeed(current, next);
        
//         Node nextNode;
//         nextNode.position = next;
//         nextNode.parent = points[i];
//         bool needRotateNext = rotateNeed(nextNode, nextNext);
        
        
//         if (i == 0) {
//             if (needRotateNow) {
//                 //states.push_back(ROTATING);
//                 states.push_back(STARTING);
//             } else {
//                 states.push_back(STARTING);
//             }
//         }

//         else if (i == points.size() - 2) {
//             if (needRotateNow) {
//                 //states.push_back(ROTATING);
//                 states.push_back(STARTING);
//             } else {
//                 states.push_back(CONTINUEE);
//             }
//             states.push_back(STOPPING);  
//         }

//         else {
//             if (needRotateNow) {
//                 //states.push_back(ROTATING);
//                 states.push_back(STARTING);
//             } 
//             else if (needRotateNext) {
//                 states.push_back(STOPPING);
//             }
//             else {
//                 states.push_back(CONTINUEE);
//             }
//         }
//     }
    
//     return states;
// }