/**
 * @file Simulator.hpp
 * @brief Robot dynamics simulator with velocity and acceleration limits
 * 
 * Simulates differential-drive robot motion with realistic constraints
 * on velocity and acceleration.
 */

#pragma once

#include <cmath>
#include <algorithm>
#include "RobotState.hpp"
#include "Point.hpp"
#include "PID.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Simulator
{
public:
    
    explicit Simulator(const RobotState& parameters);

    
    void setVelocityCommand(const Twist& command);

    
    void update(double dt);

    // ---- Getters ----
    Point getPose() const;      ///< Get current pose (x, y, theta)
    Twist getVelocity() const;  ///< Get current velocity (linear, angular)

    
    void reset(const Point& pose = Point{},
               const Twist& velocity = Twist{});

private:
    RobotState parameters_;     ///< Robot physical parameters

    Point pose_;                ///< Current pose
    Twist velocity_;            ///< Current velocity
    Twist commanded_velocity_;  ///< Desired velocity (before limits)

    
    double limitVelocity(double current, double target,
                         double acceleration, double deceleration,
                         double dt) const;

    
    static double normalizeAngle(double angle);
};