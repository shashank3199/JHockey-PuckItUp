// RobotBase/RobotBase.h

#pragma once

#include <Arduino.h>
#include <math.h>

// Forward declaration
class RobotBase;

/**
 * @brief Controls a 3-omni-wheel triangular robot base.
 *
 * This class handles the inverse kinematics and motor control for a robot
 * with three omni-directional wheels arranged symmetrically around a central point.
 * Wheel angles relative to Forward (+Y):
 * Wheel 1 (Motor 0): +60 deg (Front-Right)
 * Wheel 2 (Motor 1): -60 deg (Front-Left)
 * Wheel 3 (Motor 2): 180 deg (Rear)
 *
 * Coordinate System:
 * - +X: Right
 * - +Y: Forward
 * - Linear velocity angle (thetaDeg): Angle in degrees relative to the +Y axis,
 *   measured clockwise. 0 = Forward, 90 = Right, 180 = Backward, 270 = Left.
 */
class RobotBase
{
public:
    /**
     * @brief Default PWM pins for motors 0, 1, 2.
     */
    static constexpr int DEFAULT_PWM_PINS[3] = {4, 5, 6}; // Example PWM pins

    /**
     * @brief Default direction pins for motors 0, 1, 2.
     */
    static constexpr int DEFAULT_DIR_PINS[3] = {49, 47, 45}; // Example DIR pins

    /**
     * @brief Default motor direction inversion flags.
     * @note Set to true if a motor spins backward relative to convention.
     */
    static constexpr bool DEFAULT_REVERSE_FLAGS[3] = {false, false, false};

    /**
     * @brief Maximum PWM value
     */
    static constexpr int MAX_PWM = 255;

    /**
     * @brief Constructor for RobotBase.
     *
     * Initializes the robot base with specified or default pin configurations.
     *
     * @param pwmPins Array of 3 PWM pin numbers for motors 0, 1, 2.
     * @param dirPins Array of 3 direction pin numbers for motors 0, 1, 2.
     * @param reverse Array of 3 booleans indicating if motor direction should be reversed.
     */
    RobotBase(
        const int pwmPins[3],
        const int dirPins[3],
        const bool reverse[3]);

    /**
     * @brief Constructor using default pin configurations.
     */
    RobotBase(); // Default constructor

    /**
     * @brief Initializes motor pins. Call this in your setup() function.
     */
    void begin();

    /**
     * @brief Moves the robot based on linear speed, direction angle, and angular speed.
     *
     * Calculates required wheel velocities using inverse kinematics and sets motor speeds/directions.
     *
     * @param linearSpeed Desired linear speed (magnitude).
     * Positive values move the robot. PWM output is scaled based on this.
     * @param thetaDeg Desired angle of linear motion in degrees (clockwise from forward +Y axis).
     * 0 = Forward, 90 = Right, 180 = Backward, 270 = Left, 45 = Forward-Right, 135 = Backward-Right, etc.
     * @param angularSpeed Desired rotational speed (counter-clockwise is positive).
     */
    void moveRobot(double linearSpeed, double thetaDeg, double angularSpeed);

    /**
     * @brief Stops all motors immediately.
     *
     * Sets PWM output for all motors to 0.
     */
    void stop();

    /**
     * @brief Allows changing the motor pin configuration after initialization.
     * @param pwmPins New PWM pins (array of 3).
     * @param dirPins New Direction pins (array of 3).
     * @param reverse New motor reversal flags (array of 3).
     */
    void setMotorPins(
        const int pwmPins[3],
        const int dirPins[3],
        const bool reverse[3]);

    /**
     * @brief Test Specific Motor
     * @param motorIndex The index of the motor (0, 1, or 2).
     */
    void testMotor(int motorIndex);

private:
    /**
     * @brief Sets the speed and direction for a single motor.
     * @param motorIndex The index of the motor (0, 1, or 2).
     * @param speed The desired speed (magnitude). Will be clamped to [0, MAX_PWM].
     * @param forward The desired direction (true for forward, false for backward).
     */
    void setMotorSpeed(int motorIndex, double speed, bool forward);

    int _pwmPins[3];
    int _dirPins[3];
    bool _reverse[3];

    // Kinematic constants
    static constexpr double SQRT3 = 1.73205081;    // sqrt(3)
    static constexpr double ONE_THIRD = 1.0 / 3.0; // Defined inline where needed
    static constexpr double TWO_THIRD = 2.0 / 3.0; // Defined inline where needed
};