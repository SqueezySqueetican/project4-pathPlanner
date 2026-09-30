# Grid-Based Differential Drive Robot Control

This project implements a control system for a differential drive robot navigating on a two-dimensional grid. The robot moves along grid lines, stops at intersections, performs 90-degree rotations when needed, and continues to the next target. All motion control is handled by PID controllers.

The software is built with an object-oriented structure in C++17 using CMake. It includes separate modules for grid management, path navigation, PID control, velocity command generation, and robot simulation. The robot's velocity and acceleration limits are fully respected during motion.

Multiple test scenarios are supported, including straight paths, left and right turns, and multi-segment routes. During execution, the program displays the robot's pose, target position, errors, velocities, and navigation state in real time, while also logging all data to a CSV file.