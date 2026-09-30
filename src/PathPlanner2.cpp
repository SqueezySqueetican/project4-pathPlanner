#include "PathPlanner2.hpp"
#include <cmath>
#include <iostream>

// Constructor: Initializes planner with start, goal, and grid
PathPlanner2::PathPlanner2(Point Start_point, Point Goal_point, Grid grid) 
    : start(Start_point), goal(Goal_point), g(grid), path() {}

// Main A* pathfinding algorithm with state-aware node generation
vector<Node> PathPlanner2::findPath() {
    // Input validation 
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


    // Initialize start node 
    Node start_node;
    start_node.position = start;
    start_node.parent = start;
    start_node.g = 0;
    start_node.h = abs(start.x - goal.x) + abs(start.y - goal.y);
    start_node.f = start_node.g + start_node.h;
    start_node.pointer = start.theta;    
    start_node.state = Node::START;       

    list<Node> openSet;
    list<Node> closedSet;

    openSet.push_back(start_node);

    // A* main loop 
    while (!openSet.empty()) {
        // Find node with lowest f-cost in openSet
        Node bestNode = openSet.back();
        for (const Node& n : openSet) {
            if (n.f < bestNode.f) {
                bestNode = n;
            }
        }

        openSet.remove(bestNode);
        closedSet.push_front(bestNode);

        if (bestNode.position == goal) {
            cout << "Goal reached!" << endl;
            
            // Reconstruct raw path from closedSet 
            Node current = bestNode;
            list<Node> tempPath;
            
            while (!(current.position == start)) {
                tempPath.push_front(current);
                
                bool found = false;
                for (const Node& n : closedSet) {
                    if (n.position == current.parent) {
                        current = n;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    cerr << "Parent not found in closedSet!" << endl;
                    break;
                }
            }
            tempPath.push_front(start_node);
            
            // ======== Expand path with states =========
            path.clear();
            
            vector<Node> pathVec(tempPath.begin(), tempPath.end()); 
            
            for (size_t i = 0; i < pathVec.size(); i++) {
                Node currentNode = pathVec[i];
                Node nextNode = (i + 1 < pathVec.size()) ? pathVec[i + 1] : currentNode;
                bool isLastNode = (i == pathVec.size() - 1);
                
                // Calculate required angle for next movement
                double nextAngle;
                if (!isLastNode) {
                    nextAngle = atan2(nextNode.position.y - currentNode.position.y,
                                    nextNode.position.x - currentNode.position.x);
                    nextAngle = normalizeAngle(nextAngle);
                } else {
                    nextAngle = goal.theta;
                }
                
                // Check if rotation is needed
                double diff = normalizeAngle(nextAngle - currentNode.pointer);
                bool needRotate = fabs(diff) > 0.01;
                
                // Handle each node type
                
                // Case 1: Start node
                if (i == 0) {
                    if (needRotate) {
                        Node rotateNode = currentNode;
                        rotateNode.state = Node::ROTATE;
                        rotateNode.pointer = nextAngle;
                        path.push_back(rotateNode);

                        Node startNode = currentNode;
                        startNode.state = Node::START;
                        startNode.pointer = nextAngle;
                        path.push_back(startNode);
                    } else {
                        Node startNode = currentNode;
                        startNode.state = Node::START;
                        startNode.pointer = currentNode.pointer;
                        path.push_back(startNode);
                    }
                }
                
                // Case 2: Intermediate nodes
                else if (!isLastNode) {
                    if (needRotate) {
                        // STOP before rotation
                        Node stopNode = currentNode;
                        stopNode.state = Node::STOP;
                        stopNode.pointer = currentNode.pointer;
                        path.push_back(stopNode);
                        
                        // ROTATE in place
                        Node rotateNode = currentNode;
                        rotateNode.state = Node::ROTATE;
                        rotateNode.pointer = nextAngle;
                        path.push_back(rotateNode);
                        
                        // START after rotation
                        Node startNode = currentNode;
                        startNode.state = Node::START;
                        startNode.pointer = nextAngle;
                        path.push_back(startNode);
                    } else {
                        Node continueNode = currentNode;
                        continueNode.state = Node::CONTINUE;
                        continueNode.pointer = currentNode.pointer;
                        path.push_back(continueNode);
                    }
                }
                
                // Case 3: Goal node
                else {
                    // STOP at goal
                    Node stopNode = currentNode;
                    stopNode.state = Node::STOP;
                    stopNode.pointer = currentNode.pointer;
                    path.push_back(stopNode);
                    
                    // ROTATE at goal if needed
                    double finalDiff = normalizeAngle(currentNode.pointer - goal.theta);
                    if (fabs(finalDiff) > 0.01) {
                        Node rotateNode = currentNode;
                        rotateNode.state = Node::ROTATE;
                        rotateNode.pointer = goal.theta;
                        path.push_back(rotateNode);
                    }
                }
}
            
            cout << "Full path built! Total nodes: " << path.size() << endl;
            return path;
        }

        // Expand neighbors 
        list<Point> neighbors = g.findNeighbors(bestNode.position);

        for (Point neighborPos : neighbors) {
            // Skip if already explored
            bool inClosed = false;
            for (Node n : closedSet) {
                if (n.position == neighborPos) {
                    inClosed = true;
                    break;
                }
            }
            if (inClosed) continue;

            // Calculate cost and determine state for this move
            Node tempNode = bestNode;
            double cost = COST_calculator(tempNode, neighborPos);
            double new_g = bestNode.g + cost;

            // Check if neighbor is already in openSet
            bool inOpen = false;
            for (Node& n : openSet) {
                if (n.position == neighborPos) {
                    inOpen = true;
                    if (new_g < n.g) {
                        n.g = new_g;
                        n.parent = bestNode.position;
                        n.h = abs(n.position.x - goal.x) + abs(n.position.y - goal.y);
                        n.pointer = atan2(neighborPos.y - bestNode.position.y, 
                                        neighborPos.x - bestNode.position.x);
                        n.pointer = normalizeAngle(n.pointer);
                        n.state = tempNode.state;
                        n.f = n.g + n.h;
                    }
                    break;
                }
            }

            // Add new node to openSet
            if (!inOpen) {
                Node n;
                n.position = neighborPos;
                n.parent = bestNode.position;
                n.g = new_g;
                n.h = abs(neighborPos.x - goal.x) + abs(neighborPos.y - goal.y);
                n.f = n.g + n.h;
                n.state = tempNode.state;
                n.pointer = atan2(neighborPos.y - bestNode.position.y, 
                                neighborPos.x - bestNode.position.x);
                n.pointer = normalizeAngle(n.pointer);
                openSet.push_back(n);
            }
        }
    }

    cout << "No path found!" << endl;
    return {};
}



bool PathPlanner2::rotateNeed(Node bestNode, Point neighbor) {
    double current_angle = bestNode.pointer;
        double next_angle = atan2(neighbor.y - bestNode.position.y, neighbor.x - bestNode.position.x);
    next_angle = normalizeAngle(next_angle);
    
    double diff = normalizeAngle(next_angle - current_angle);
    
    
    return fabs(diff) > 0.01;
}

// Method: Check if a cell is near an obstacle (for cost penalty)
bool PathPlanner2::nearObstacle(Point neighbor) {
    list<Point> neighbors_of_neighbor = g.allNeighbors(neighbor);
    for (Point n : neighbors_of_neighbor) {
        if (!g.isInGrid(n)) {  // Out of grid = obstacle boundary
            return true;
        }
    }
    return false;
}

// Method: Calculate movement cost and determine robot state for each transition
double PathPlanner2::COST_calculator(Node& bestNode, Point neighbor) {
    double cost = 0;
    
    // Case 1: Moving from start position 
    if (bestNode.position == start) {
        if (rotateNeed(bestNode, neighbor)) {
            cost += ROTATE_cost;   // Need to rotate first
            //bestNode.state = Node::ROTATE;
        } else {
            cost += START_cost;     // Can start moving directly
            //bestNode.state = Node::START;
        }
    }
    
    // Case 2: Rotation needed during movement 
    else if (rotateNeed(bestNode, neighbor)) {
        cost += STOP_cost;      // Stop before rotating
        cost += ROTATE_cost;    // Rotate in place
        cost += START_cost;     // Start moving again
        //bestNode.state = Node::ROTATE;
    }
    
    //  Case 3: Reaching goal 
    else if (neighbor == goal) {
        cost += STOP_cost;      // Stop at goal
        //bestNode.state = Node::STOP;
        
        // Additional rotation at goal if orientation mismatch
        if (fabs(normalizeAngle(bestNode.pointer - goal.theta)) > 0.01) {
            cost += ROTATE_cost;
        }
    }
    
    //  Case 4: Normal straight movement 
    else {
        cost += CONTINUE_cost;   // Continue moving forward
        //bestNode.state = Node::CONTINUE;
    }
    
    // Additional penalty for near-obstacle cells 
    if (nearObstacle(neighbor)) {
        cost += OBSTACLE_cost;
    }
    
    return cost;
}

// Method: Normalize angle to [-PI, PI] range
double PathPlanner2::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
