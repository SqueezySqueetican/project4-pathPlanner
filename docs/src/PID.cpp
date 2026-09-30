#include "PID.hpp"

PID::PID(double kp, double ki, double kd, double t_step)
    : Kp(kp), Ki(ki), Kd(kd), dt(t_step),
      integral(0.0), previous_error(0.0) {}

double PID::calculate(double error) {
    double P = Kp * error;

    integral += error * dt;
    double I = Ki * integral;

    double D = Kd * (error - previous_error) / dt;
    previous_error = error;

    double command = P + I + D;
    return command;
}

void PID::reset() {
    integral = 0.0;
    previous_error = 0.0;
}