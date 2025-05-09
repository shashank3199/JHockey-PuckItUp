// PIDController/PIDController.cpp

#include "PIDController.h"
#include <Arduino.h>

/**
 * @brief Constructor implementation. Initializes gains, mode, factors, and state variables.
 */
PIDController::PIDController(const double gains[3],
                             PIDMode mode,
                             double accFactorPresent,
                             double accFactorPast)
    : _mode(mode),
      _accFactorPresent(constrain(accFactorPresent, 0.0, 1.0)), // Ensure factors are within [0, 1]
      _accFactorPast(constrain(accFactorPast, 0.0, 1.0)),
      _prevError(0.0),
      _accumulatedError(0.0),
      _prevErrorX(0.0),
      _accumulatedErrorX(0.0),
      _prevErrorY(0.0),
      _accumulatedErrorY(0.0)
{
    setGains(gains); // Use the setter to copy gains
}

/**
 * @brief Calculates the PID correction value for a single loop.
 */
double PIDController::getCorrection(double setpoint,
                                    double measuredValue,
                                    double scale)
{
    // Check for invalid inputs
    if (isnan(setpoint) || isnan(measuredValue))
    {
        // Returning NAN signals an issue upstream.
        Serial.println("Error: NAN input detected in PID calculation.");
        return NAN;
    }

    double currError = setpoint - measuredValue;

    // Calculate terms based on mode
    double kpTerm = _Kp * currError;
    double kdTerm = 0.0;
    double kiTerm = 0.0;

    if (_mode == PIDMode::PID || _mode == PIDMode::PD)
    {
        // Calculate derivative term only if Kd is active and mode requires it
        if (_Kd != 0.0)
        {
            kdTerm = _Kd * (currError - _prevError);
        }
    }

    if (_mode == PIDMode::PID)
    {
        // Calculate integral term only if Ki is active and mode is PID
        if (_Ki != 0.0)
        {
            _accumulatedError = currError * _accFactorPresent + _accumulatedError * _accFactorPast;
            kiTerm = _Ki * _accumulatedError;
        }
    }

    // Store current error for next derivative calculation
    _prevError = currError;

    // Calculate final output
    double output = (kpTerm + kdTerm + kiTerm) * scale;
    return output;
}

/**
 * @brief Calculates PID corrections for coupled X and Y axes using shared gains.
 */
double PIDController::getCorrection(double setpointX, double measuredX,
                                    double setpointY, double measuredY,
                                    double &corrX_out, double &corrY_out,
                                    double maxSpeed)
{
    // Check for invalid inputs
    if (isnan(setpointX) || isnan(measuredX) || isnan(setpointY) || isnan(measuredY) || isnan(maxSpeed))
    {
        Serial.println("Error: NAN input detected in PID calculation.");
        corrX_out = NAN;
        corrY_out = NAN;

        // Return NAN to indicate speed calculation error
        return NAN;
    }

    // X Axis Calculation (Raw Correction)
    double currErrorX = setpointX - measuredX;
    double kpTermX = _Kp * currErrorX;
    double kdTermX = 0.0;
    double kiTermX = 0.0;

    // Derivative term
    if ((_mode == PIDMode::PID || _mode == PIDMode::PD) && _Kd != 0.0)
    {
        kdTermX = _Kd * (currErrorX - _prevErrorX);
    }
    // Integral term
    if (_mode == PIDMode::PID && _Ki != 0.0)
    {
        _accumulatedErrorX = currErrorX * _accFactorPresent + _accumulatedErrorX * _accFactorPast;
        kiTermX = _Ki * _accumulatedErrorX;
    }
    _prevErrorX = currErrorX;                      // Update previous error for X
    double rawCorrX = kpTermX + kdTermX + kiTermX; // Calculate raw X correction

    // Y Axis Calculation (Raw Correction)
    double currErrorY = setpointY - measuredY;
    double kpTermY = _Kp * currErrorY;
    double kdTermY = 0.0;
    double kiTermY = 0.0;

    // Derivative term
    if ((_mode == PIDMode::PID || _mode == PIDMode::PD) && _Kd != 0.0)
    {
        kdTermY = _Kd * (currErrorY - _prevErrorY);
    }
    // Integral term
    if (_mode == PIDMode::PID && _Ki != 0.0)
    {
        _accumulatedErrorY = currErrorY * _accFactorPresent + _accumulatedErrorY * _accFactorPast;
        kiTermY = _Ki * _accumulatedErrorY;
    }
    _prevErrorY = currErrorY;                      // Update previous error for Y
    double rawCorrY = kpTermY + kdTermY + kiTermY; // Calculate raw Y correction

    // Calculate Combined Speed Magnitude
    double calculatedSpeed = sqrt(rawCorrX * rawCorrX + rawCorrY * rawCorrY);

    // Constrain Speed
    double constrainedSpeed = constrain(calculatedSpeed, 0.0, abs(maxSpeed)); // Ensure maxSpeed is non-negative

    // Assign Reference Outputs & Return Speed
    corrX_out = rawCorrX; // Output the raw, unscaled X correction via reference
    corrY_out = rawCorrY; // Output the raw, unscaled Y correction via reference

    return constrainedSpeed; // Return the final, constrained linear speed
}

/**
 * @brief Updates the PID gains.
 */
void PIDController::setGains(const double gains[3])
{
    // Basic validation: ensure gains are not NAN
    _Kp = isnan(gains[0]) ? 0.0 : gains[0];
    _Ki = isnan(gains[1]) ? 0.0 : gains[1];
    _Kd = isnan(gains[2]) ? 0.0 : gains[2];
}

/**
 * @brief Resets integral accumulators and previous error states.
 */
void PIDController::resetIntegral()
{
    _accumulatedError = 0.0;
    _prevError = 0.0;

    _accumulatedErrorX = 0.0;
    _prevErrorX = 0.0;
    _accumulatedErrorY = 0.0;
    _prevErrorY = 0.0;
}