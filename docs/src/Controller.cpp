#include "Controller.hpp"
#include <cmath>
#include <iostream>

Controller::Controller(double kp_lin, double ki_lin, double kd_lin,
                       double kp_ang, double ki_ang, double kd_ang,
                       double dt)
    : linear_pid(kp_lin, ki_lin, kd_lin, dt),
      angular_pid(kp_ang, ki_ang, kd_ang, dt) {}

Twist Controller::computeCommand(Point robot, Node target, Robot_State robot_state) {
    Twist cmd;
    
    double dx = target.position.x - robot.x;
    double dy = target.position.y - robot.y;
    double error_distance = sqrt(dx * dx + dy * dy);
    
    double target_angle = atan2(dy, dx);
    double error_angle = target_angle - robot.theta;
    error_angle = normalizeAngle(error_angle);

    switch(robot_state) {
        case Robot_State::MOVING:
            cmd.linear_x = linear_pid.calculate(error_distance);
            cmd.angular_z = angular_pid.calculate(error_angle);
            break;
            
        case Robot_State::ROTATING:
            cmd.linear_x = 0.0;
            cmd.angular_z = angular_pid.calculate(error_angle);
            break;
            
        case Robot_State::STOPPING:
            cmd.linear_x = linear_pid.calculate(error_distance);
            cmd.angular_z = angular_pid.calculate(error_angle);
            break;
            
        case Robot_State::FINISHED:
            cmd.linear_x = 0.0;
            cmd.angular_z = 0.0;
            break;
    }

    return cmd;
}

double Controller::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}