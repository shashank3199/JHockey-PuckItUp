// PIDController/PIDController.h

#pragma once

#include <Arduino.h>
#include <math.h> // For isnan, fabsf
#include <stdint.h>

/**
 * @brief Defines the operational mode of the PID controller.
 * Determines which terms (Proportional, Integral, Derivative) are active.
 */
enum class PIDMode : uint8_t
{
    P,  // Proportional only
    PD, // Proportional and Derivative
    PID // Proportional, Integral, and Derivative
};

/**
 * @brief Implements a PID (Proportional-Integral-Derivative) controller.
 *
 * This class provides a standard PID controller implementation suitable for
 * embedded systems like Arduino. It avoids dynamic memory allocation and the STL.
 * It supports P, PD, and PID modes and includes configurable integral accumulation factors.
 * It also offers an overload for handling coupled XY control loops with shared gains but separate error states.
 */
class PIDController
{
public:
    /**
     * @brief Constructor for the PIDController.
     *
     * @param gains An array of 3 doubles: {Kp, Ki, Kd}.
     * @param mode The control mode (P, PD, or PID). Defaults to PID.
     * @param accFactorPresent The factor applied to the current error when updating the integral term (0.0 to 1.0). Defaults to 1.0.
     * @param accFactorPast The factor applied to the previous accumulated error (0.0 to 1.0, typically < 1.0). Defaults to 0.995.
     */
    PIDController(const double gains[3],
                  PIDMode mode = PIDMode::PID,
                  double accFactorPresent = 1.0,
                  double accFactorPast = 0.995);

    /**
     * @brief Calculates the PID correction value for a single control loop.
     *
     * @param setpoint The desired target value for the controlled variable.
     * @param measuredValue The current measured value of the controlled variable.
     * @param scale A factor to multiply the final PID output by. Defaults to 1.0.
     * @return The calculated correction value, scaled by the 'scale' factor. Returns NAN if inputs are NAN.
     */
    double getCorrection(double setpoint,
                         double measuredValue,
                         double scale = 1.0);

    /**
     * @brief Calculates PID corrections for coupled X and Y axes and returns the combined linear speed.
     *
     * Calculates independent PID corrections for X and Y axes using shared gains.
     * Determines the magnitude of the combined correction vector to represent linear speed,
     * then constrains this speed to a maximum value and returns it.
     * The raw, unscaled X and Y corrections are provided via output reference parameters.
     *
     * @param setpointX The desired target value for the X axis.
     * @param measuredX The current measured value for the X axis.
     * @param setpointY The desired target value for the Y axis.
     * @param measuredY The current measured value for the Y axis.
     * @param corrX_out Output reference parameter for the raw, unscaled calculated X correction value.
     * @param corrY_out Output reference parameter for the raw, unscaled calculated Y correction value.
     * @param maxSpeed The maximum allowable magnitude for the output linear speed.
     * @return The calculated linear speed magnitude, constrained by maxSpeed. Returns NAN if inputs are NAN.
     */
    double getCorrection(double setpointX, double measuredX,
                         double setpointY, double measuredY,
                         double &corrX_out, double &corrY_out,
                         double maxSpeed);

    /**
     * @brief Updates the PID gains (Kp, Ki, Kd).
     *
     * @param gains An array of 3 doubles: {Kp, Ki, Kd}.
     */
    void setGains(const double gains[3]);

    /**
     * @brief Resets the integral accumulator term to zero.
     * Useful when the system undergoes a large change or enters a new state.
     * Also resets the previous error terms.
     */
    void resetIntegral();

private:
    double _Kp;    // Proportional gain
    double _Ki;    // Integral gain
    double _Kd;    // Derivative gain
    PIDMode _mode; // Current PID mode (P, PD, PID)

    double _accFactorPresent; // Factor for current error in integral accumulation
    double _accFactorPast;    // Factor for past accumulation in integral

    // State variables for single getCorrection()
    double _prevError;        // Error from the previous calculation step
    double _accumulatedError; // Accumulated error for the integral term

    // Separate state variables for XY getCorrection()
    double _prevErrorX;        // Previous error for the X axis
    double _accumulatedErrorX; // Accumulated error for the X axis integral term
    double _prevErrorY;        // Previous error for the Y axis
    double _accumulatedErrorY; // Accumulated error for the Y axis integral term
};