#include "Controller.hpp"
#include <cmath>

// Constructor
Controller::Controller(double kp_lin, double ki_lin, double kd_lin,
                       double kp_ang, double ki_ang, double kd_ang,
                       double dt)
    : linear_pid(kp_lin, ki_lin, kd_lin, dt),
      angular_pid(kp_ang, ki_ang, kd_ang, dt),
      prev_linear_error(0.0),
      prev_angular_error(0.0) {}

// Main compute method
Twist Controller::computeCommand(Point robot, Point target, Robot_State robotState,
                                 double target_angle) {
    Twist cmd{0.0, 0.0};

    // Calculate errors
    double dx = target.x - robot.x;
    double dy = target.y - robot.y;
    double error_distance = sqrt(dx * dx + dy * dy);
    
    // Use target_angle if provided (for ROTATE state), otherwise calculate from target
    double error_angle;
    if (robotState == ROTATING) {
        // For rotation, use the target_angle from Node pointer
        error_angle = normalizeAngle(target_angle - robot.theta);
    } else {
        // For movement, calculate angle to target
        double target_angle_from_pos = atan2(dy, dx);
        error_angle = normalizeAngle(target_angle_from_pos - robot.theta);
    }

    
    switch (robotState) {
        
        // ========================================
        // MOVING: Move toward target
        // ========================================
        case MOVING: {
            cmd.linear_x = linear_pid.calculate(error_distance);
            cmd.angular_z = angular_pid.calculate(error_angle);
            
            break;
        }
        
        // ========================================
        // ROTATING: Rotate in place
        // ========================================
        case ROTATING: {
            // Zero linear velocity
            cmd.linear_x = 0.0;            
            cmd.angular_z = angular_pid.calculate(error_angle);
        
        }
        
        // ========================================
        // STOPPING: Stop all motion
        // ========================================
        case STOPPING: {
            cmd.linear_x = linear_pid.calculate(error_distance);
            cmd.angular_z = angular_pid.calculate(error_angle);

            break;
        }
        
        // ========================================
        // FINISHED: Stop all motion
        // ========================================
        case FINISHED: {
            cmd.linear_x = 0.0;
            cmd.angular_z = 0.0;
            
            // Reset PID errors
            linear_pid.reset();
            angular_pid.reset();
            break;
        }
        
    }

    return cmd;
}

// Compute linear velocity based on state
double Controller::computeLinearVelocity(Robot_State robotState, double error_distance) {
    if (robotState == MOVING) {
        return linear_pid.calculate(error_distance);
    }
    return 0.0;  // STOPPING, ROTATING, FINISHED
}

// Compute angular velocity based on state
double Controller::computeAngularVelocity(Robot_State robotState, double error_angle) {
    if (robotState == MOVING || robotState == ROTATING) {
        return angular_pid.calculate(error_angle);
    }
    return 0.0;  // STOPPING, FINISHED
}

// Reset PID controllers
void Controller::reset() {
    linear_pid.reset();
    angular_pid.reset();
    prev_linear_error = 0.0;
    prev_angular_error = 0.0;
}

// Normalize angle to [-PI, PI]
double Controller::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}







































