// RobotBase/RobotBase.cpp

#include "RobotBase.h"
#include <Arduino.h> // Includes math functions and DEG_TO_RAD

/**
 * @brief Constructor implementation.
 */
RobotBase::RobotBase(
    const int pwmPins[3],
    const int dirPins[3],
    const bool reverse[3])
{
    // Copy pin configurations from input arrays to member arrays
    for (int i = 0; i < 3; ++i)
    {
        _pwmPins[i] = pwmPins[i];
        _dirPins[i] = dirPins[i];
        _reverse[i] = reverse[i];
    }
}

/**
 * @brief Default Constructor implementation. Uses default pin configurations.
 */
RobotBase::RobotBase()
{
    // Copy default pin configurations to member arrays
    for (int i = 0; i < 3; ++i)
    {
        _pwmPins[i] = DEFAULT_PWM_PINS[i];
        _dirPins[i] = DEFAULT_DIR_PINS[i];
        _reverse[i] = DEFAULT_REVERSE_FLAGS[i];
    }
}

/**
 * @brief Initializes motor driver pins as outputs.
 */
void RobotBase::begin()
{
    for (int i = 0; i < 3; ++i)
    {
        pinMode(_pwmPins[i], OUTPUT);
        pinMode(_dirPins[i], OUTPUT);
        digitalWrite(_dirPins[i], LOW); // Default direction state
        analogWrite(_pwmPins[i], 0);    // Ensure motors are stopped initially
    }
}

/**
 * @brief Allows changing motor pin configuration using C-style arrays.
 */
void RobotBase::setMotorPins(
    const int pwmPins[3],
    const int dirPins[3],
    const bool reverse[3])
{
    // Stop motors before changing pins
    stop();

    // Copy new pin configurations
    for (int i = 0; i < 3; ++i)
    {
        _pwmPins[i] = pwmPins[i];
        _dirPins[i] = dirPins[i];
        _reverse[i] = reverse[i];
    }

    // Re-initialize new pins
    begin();
}

/**
 * @brief Moves the robot based on specified linear and angular velocities.
 */
void RobotBase::moveRobot(double linearSpeed, double thetaDeg, double angularSpeed)
{
    // Convert theta from degrees to radians
    double thetaRad = (thetaDeg * DEG_TO_RAD);

    // Decompose linear velocity into X and Y components
    double v_x = linearSpeed * sin(thetaRad); // Velocity component along robot's X-axis (right)
    double v_y = linearSpeed * cos(thetaRad); // Velocity component along robot's Y-axis (forward)
    double omega = angularSpeed;              // Angular velocity (counter-clockwise positive)

    // Inverse Kinematics for 3 Omni Wheels at 120 degrees
    // Assumes:
    // Wheel 1 @ +60 deg from +Y (Forward)
    // Wheel 2 @ -60 deg from +Y (Left)
    // Wheel 3 @ -180 deg from +Y (Backward)

    double v1 = ONE_THIRD * (-v_x + SQRT3 * v_y + omega); // Motor 0
    double v2 = ONE_THIRD * (-v_x - SQRT3 * v_y + omega); // Motor 1
    double v3 = ONE_THIRD * (2.0 * v_x + omega);          // Motor 2

    // Assign calculated speeds to an array
    double wheelSpeeds[3] = {v1, v2, v3};
    bool wheelDirs[3] = {true, true, true}; // true = forward

    // Determine direction and magnitude for each wheel
    for (int i = 0; i < 3; ++i)
    {
        if (wheelSpeeds[i] < 0.0)
        {
            wheelDirs[i] = false;             // Move backward
            wheelSpeeds[i] = -wheelSpeeds[i]; // Speed magnitude is positive
        }
        // Apply motor reversal flag if needed
        if (_reverse[i])
        {
            wheelDirs[i] = !wheelDirs[i];
        }
    }

    // Set motor speeds (magnitude clamped to MAX_PWM)
    for (int i = 0; i < 3; ++i)
    {
        setMotorSpeed(i, wheelSpeeds[i], wheelDirs[i]);
    }
}

/**
 * @brief Stops all motors.
 */
void RobotBase::stop()
{
    for (int i = 0; i < 3; ++i)
    {
        setMotorSpeed(i, 0.0, true); // Speed 0, direction doesn't matter
    }
}

/**
 * @brief Tests a specific motor by setting its speed and direction.
 */
void RobotBase::testMotor(int motorIndex)
{
    if (motorIndex < 0 || motorIndex > 2)
    {
        Serial.println("Invalid motor index. Must be 0, 1, or 2.");
        return; // Basic bounds check
    }

    // Test motor by setting speed and direction
    Serial.print("Testing motor " + String(motorIndex) + " forward: ");
    setMotorSpeed(motorIndex, MAX_PWM, true); // Set to maximum speed forward
    delay(1000);                              // Run for 1 second
    Serial.print("Stopping motor: ");
    setMotorSpeed(motorIndex, 0.0, true); // Stop the motor
    delay(1000);                          // Wait for 1 second

    Serial.print("Testing motor " + String(motorIndex) + " backward: ");
    setMotorSpeed(motorIndex, MAX_PWM, false); // Set to maximum speed backward
    delay(1000);                               // Run for 1 second
    Serial.print("Stopping motor: ");
    setMotorSpeed(motorIndex, 0.0, true); // Stop the motor
}

/**
 * @brief Sets individual motor speed and direction.
 */
void RobotBase::setMotorSpeed(int motorIndex, double speed, bool forward)
{
    if (motorIndex < 0 || motorIndex > 2)
        return; // Basic bounds check

    // Clamp speed to PWM range [0, MAX_PWM]
    int pwmValue = static_cast<int>(round(speed));
    pwmValue = constrain(pwmValue, 0, MAX_PWM);

    // Determine direction pin state based on 'forward' flag and motor driver type
    // Assuming HIGH = forward, LOW = backward for the specific driver used.
    digitalWrite(_dirPins[motorIndex], forward ? HIGH : LOW);
    analogWrite(_pwmPins[motorIndex], pwmValue);
}