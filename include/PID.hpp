/**
 * @file PID.hpp
 * @brief Simple PID controller implementation
*/

#pragma once

/*
 * @class PID
 * @brief PID controller with fixed time step
 * Formula: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 */

class PID {
private:
    double Kp, Ki, Kd;       ///< PID gains
    double dt;               ///< Time step (seconds)
    double integral;         ///< Integral accumulator
    double previous_error;   ///< Previous error for derivative


public:
    PID(double kp, double ki, double kd, double t_step);


    /**
     * @brief Calculate PID output
     * @param error Current error
     * @return Control output
     */
    double calculate(double error);

    /**
     * @brief Reset internal state (integral and previous error)
     */
    void reset();
};