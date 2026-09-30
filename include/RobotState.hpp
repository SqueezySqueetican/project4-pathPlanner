#pragma once


struct Twist
{
    double linear_x{0.0};
    double angular_z{0.0};
};

struct RobotState
{
    // Robot geometry
    double wheel_radius{0.1};               // [m]
    double wheel_separation{0.5};           // [m]

    // Velocity limits
    double max_linear_velocity{1.0};        // [m/s]
    double max_angular_velocity{1.5};       // [rad/s]

    // Acceleration limits
    double max_linear_acceleration{0.5};    // [m/s^2]
    double max_linear_deceleration{1.0};    // [m/s^2]

    double max_angular_acceleration{1.0};   // [rad/s^2]
    double max_angular_deceleration{1.5};   // [rad/s^2]
};


enum Robot_State { MOVING, STOPPING, ROTATING, FINISHED};