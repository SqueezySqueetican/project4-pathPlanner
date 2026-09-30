# API Overview

Quick reference for all public interfaces.

## Point

```cpp
struct Point {
    double x, y;       // Position (meters)
    double theta;      // Orientation (radians)
    bool operator==(Point point);
};

class Grid {
public:
    Grid(int width, int height, double distance);
    bool isInGrid(Point point);
    void obstacle(Point obstacle_point);
    list<Point> findNeighbors(Point point);
};

class PID {
public:
    PID(double kp, double ki, double kd, double dt);
    double calculate(double error);
    void reset();
};

class Controller {
public:
    Controller(double kp_lin, double ki_lin, double kd_lin,
               double kp_ang, double ki_ang, double kd_ang,
               double dt);
    Twist computeCommand(Point robot, Point target, State state);
};

class Simulator {
public:
    explicit Simulator(const RobotState& parameters);
    void setVelocityCommand(const Twist& command);
    void update(double dt);
    Point getPose() const;
    Twist getVelocity() const;
    void reset(const Point& pose = Point{}, const Twist& velocity = Twist{});
};

class Navigator {
public:
    Navigator(Grid grid, vector<Point> Path, double pos_tol = 0.02, double ang_tol = 0.01);
    void update(Point position);
    Point getTarget();
    State getState();
    bool isFinished();
    int getIndex();
};

enum State { MOVING, STOPPING, ROTATING, FINISHED };

struct Twist {
    double linear_x;   // Linear velocity (m/s)
    double angular_z;  // Angular velocity (rad/s)
};

struct RobotState {
    double wheel_radius;
    double wheel_separation;
    double max_linear_velocity;
    double max_angular_velocity;
    double max_linear_acceleration;
    double max_linear_deceleration;
    double max_angular_acceleration;
    double max_angular_deceleration;
};

// 1. Setup environment
Grid grid(10, 10, 1.0);
vector<Point> path = { {0,0}, {3,0}, {3,3} };

// 2. Initialize components
RobotState params;
Simulator robot(params);
Controller controller(1.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.01);
Navigator navigator(grid, path);

// 3. Simulation loop
while (!navigator.isFinished()) {
    Point pose = robot.getPose();
    navigator.update(pose);
    Twist cmd = controller.computeCommand(pose, navigator.getTarget(), navigator.getState());
    robot.setVelocityCommand(cmd);
    robot.update(0.01);
}
```