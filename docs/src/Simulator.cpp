#include "Simulator.hpp"
#include <cmath>

// Helper function for clamping values
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

Simulator::Simulator(
    const RobotState& parameters)
    : parameters_(parameters)
{
}

void Simulator::setVelocityCommand(const Twist& command) {
    // Clamp commanded velocities to robot limits
    commanded_velocity_.linear_x =
        clamp(
            command.linear_x,
            -parameters_.max_linear_velocity,
             parameters_.max_linear_velocity);

    commanded_velocity_.angular_z =
        clamp(
            command.angular_z,
            -parameters_.max_angular_velocity,
             parameters_.max_angular_velocity);
}

void Simulator::update(double dt)
{
    if (dt <= 0.0)
        return;

    // Simulate the robot's velocity response.
    velocity_.linear_x =
        limitVelocity(
            velocity_.linear_x,
            commanded_velocity_.linear_x,
            parameters_.max_linear_acceleration,
            parameters_.max_linear_deceleration,
            dt);

    velocity_.angular_z =
        limitVelocity(
            velocity_.angular_z,
            commanded_velocity_.angular_z,
            parameters_.max_angular_acceleration,
            parameters_.max_angular_deceleration,
            dt);


    // Update pose using differential drive kinematics
    pose_.x += velocity_.linear_x *
               std::cos(pose_.theta) * dt;

    pose_.y += velocity_.linear_x *
               std::sin(pose_.theta) * dt;

    pose_.theta += velocity_.angular_z * dt;

    pose_.theta = normalizeAngle(pose_.theta); 
}

Point Simulator::getPose() const{
    return pose_;
}

Twist Simulator::getVelocity() const{
    return velocity_;
}

void Simulator::reset(const Point& pose, const Twist& velocity){
    pose_ = pose;
    velocity_ = velocity;
    commanded_velocity_ = Twist{};
}

double Simulator::limitVelocity(
    double current,
    double target,
    double acceleration,
    double deceleration,
    double dt) const{

    const double difference = target - current;

    if (std::abs(difference) < 1e-9)
        return target;

    double max_delta;

    // Speeding up
    if (std::abs(target) > std::abs(current))
    {
        max_delta = acceleration * dt;
    }
    else
    {
        // Slowing down
        max_delta = deceleration * dt;
    }

    if (difference > 0.0){
        return std::min(
            current + max_delta,
            target);
    }
    else{
        return std::max(
            current - max_delta,
            target);
    }
}

double Simulator::normalizeAngle(double angle)
{
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
        
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    return angle;
}