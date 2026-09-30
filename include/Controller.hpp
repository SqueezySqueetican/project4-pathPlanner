#pragma once

#include "Grid.hpp"
#include "PID.hpp"
#include "RobotState.hpp"
#include "Node.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Controller {
private:
    PID linear_pid;
    PID angular_pid;
    
    // Previous errors for smooth transitions
    double prev_linear_error{0.0};
    double prev_angular_error{0.0};

public:

    Controller(double kp_lin, double ki_lin, double kd_lin,
               double kp_ang, double ki_ang, double kd_ang,
               double dt);

    Twist computeCommand(Point robot, Point target, Robot_State robotState, 
                         double target_angle = 0.0);

    void reset();

private:
    static double normalizeAngle(double angle);

    double computeLinearVelocity(Robot_State robotState, double error_distance);

    double computeAngularVelocity(Robot_State robotState, double error_angle);
};