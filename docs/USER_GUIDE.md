## USER_GUIDE.md 

```markdown
# User Guide

## Running the Simulation

The simulation is configured in `main.cpp`. To customize:

### 1. Choose a Path

In `main.cpp`, 3 predefined paths are available (or you can define your own path).
to select a path change this line:

```cpp
vector<Point> path = path2;
``` 

### 2. Add Obstacles
Uncomment and modify:

```cpp
Point obs; obs.x = 3; obs.y = 3;
grid.obstacle(obs);
```


### 3. Tune PID Gains
In main.cpp:

```cpp
Controller controller(
    1.0, 0.0, 0.0,    // Linear: Kp, Ki, Kd
    2.5, 0.0, 0.0,    // Angular: Kp, Ki, Kd
    0.01              // Time step
);
```

Tuning tips:
- Increase Kp_linear for faster approach to target
- Increase Kp_angular for faster rotation
- Add Ki to eliminate steady-state error (use carefully)
- Add Kd to reduce overshoot (use carefully)

### 4. Adjust Tolerances
```cpp
Navigator navigator(grid, path, 
    0.005,    // Position tolerance (meters) - how close to waypoint
    0.005     // Angle tolerance (radians) - how accurate rotation
);
```

### 5. Robot Parameters
In RobotState.hpp, modify the RobotState struct:

```cpp
struct RobotState {
    double wheel_radius{0.1};           // Wheel size
    double max_linear_velocity{1.0};    // Max speed (m/s)
    double max_linear_acceleration{0.5}; // Acceleration (m/s²)
    // ... etc
};
```