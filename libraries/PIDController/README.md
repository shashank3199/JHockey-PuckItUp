# PIDController Library for Arduino

A robust Proportional-Integral-Derivative (PID) controller library designed specifically for Arduino and similar embedded platforms. This library provides a clean implementation of PID control algorithms for various applications like robot navigation, motor control, temperature regulation, and position tracking.

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Installation](#installation)
4. [Mathematical Model](#mathematical-model)
5. [Usage](#usage)
6. [Examples](#examples)
   - [Straight Line Control](#straight-line-control)
   - [Block Orientation Control](#block-orientation-control)
   - [Zigbee Aruco Navigation](#zigbee-aruco-navigation)
7. [Class Reference](#class-reference)
8. [Files in the Repository](#files-in-the-repository)
9. [Tuning Guide](#tuning-guide)
10. [Integral Accumulation Factors](#integral-accumulation-factors)
11. [Troubleshooting](#troubleshooting)

## Overview

This PIDController library follows Arduino coding standards, making it suitable for resource-constrained environments. It implements the standard PID control algorithm with configurable operational modes, tunable gains, and customizable integral accumulation behavior.

## Features

- **Multiple Control Modes**: Supports three operational modes:
  - P (Proportional only)
  - PD (Proportional-Derivative)
  - PID (Proportional-Integral-Derivative)

- **Configurable Integral Behavior**: Fine-tune integral term accumulation using customizable present and past accumulation factors.

- **XY Position Control**: Special overload for coupled XY systems where the same PID gains apply to both axes but error states need to be tracked independently.

- **Simple API**: Clean, easy-to-use interface with well-documented methods.

- **Resource Efficient**: Uses fixed memory allocation patterns suitable for embedded systems.

## Installation

1. Download the library files
2. Place the `PIDController` folder in your Arduino `libraries` directory
   - Windows: `Documents\Arduino\libraries\`
   - Mac/Linux: `~/Documents/Arduino/libraries/`
3. Restart the Arduino IDE
4. The library should now appear in the Arduino IDE under Sketch > Include Library

## Mathematical Model

The PIDController implements the standard discrete-time PID algorithm:

### Single-Axis Control

For a single control variable, the algorithm calculates the correction output using the following equations:

1. Calculate current error:

   $e(t) = \text{setpoint} - \text{measuredValue}$

2. Calculate proportional term:

   $P(t) = K_p \cdot e(t)$

3. Calculate derivative term (if in PD or PID mode):

   $D(t) = K_d \cdot (e(t) - e(t-1))$

4. Calculate integral term (if in PID mode):

   $I_{accum}(t) = e(t) \cdot \text{accFactorPresent} + I_{accum}(t-1) \cdot \text{accFactorPast}$
   $I(t) = K_i \cdot I_{accum}(t)$

5. Calculate final output:

   $\text{output} = (P(t) + I(t) + D(t)) \cdot \text{scale}$

The pseudocode implementation is as follows:

```python
currError = setpoint - measuredValue

kpTerm = Kp * currError

if mode is PD or PID:
    kdTerm = Kd * (currError - prevError)
else:
    kdTerm = 0

if mode is PID:
    accumulatedError = currError * accFactorPresent + accumulatedError * accFactorPast
    kiTerm = Ki * accumulatedError
else:
    kiTerm = 0

output = (kpTerm + kdTerm + kiTerm) * scale
prevError = currError  # Store for next iteration
```

### XY Coupled Control

For coupled XY control, the algorithm calculates corrections for both axes independently, but returns a combined speed magnitude:

For X-axis:

$$e_x(t) = \text{setpointX} - \text{measuredX}$$
$$P_x(t) = K_p \cdot e_x(t)$$
$$D_x(t) = K_d \cdot (e_x(t) - e_x(t-1))$$
$$I_{accum,x}(t) = e_x(t) \cdot \text{accFactorPresent} + I_{accum,x}(t-1) \cdot \text{accFactorPast}$$
$$I_x(t) = K_i \cdot I_{accum,x}(t)$$
$$\text{rawCorrX} = P_x(t) + I_x(t) + D_x(t)$$

Similar equations apply for the Y-axis.

The combined speed magnitude is calculated as:

$$\text{speed} = \sqrt{(\text{rawCorrX})^2 + (\text{rawCorrY})^2}$$
$$\text{constrainedSpeed} = \text{constrain}(\text{speed}, 0, \text{maxSpeed})$$

Pseudocode:

```cpp
// X-Axis
currErrorX = setpointX - measuredX
kpTermX = Kp * currErrorX
kdTermX = Kd * (currErrorX - prevErrorX)  // If applicable
accumulatedErrorX = currErrorX * accFactorPresent + accumulatedErrorX * accFactorPast  // If applicable
kiTermX = Ki * accumulatedErrorX  // If applicable
rawCorrX = kpTermX + kdTermX + kiTermX

// Y-Axis (similar to X)
currErrorY = setpointY - measuredY
kpTermY = Kp * currErrorY
kdTermY = Kd * (currErrorY - prevErrorY)  // If applicable
accumulatedErrorY = currErrorY * accFactorPresent + accumulatedErrorY * accFactorPast  // If applicable
kiTermY = Ki * accumulatedErrorY  // If applicable
rawCorrY = kpTermY + kdTermY + kiTermY

// Combined speed magnitude
speed = sqrt(rawCorrX^2 + rawCorrY^2)
constrainedSpeed = constrain(speed, 0.0, maxSpeed)

// Return raw corrections by reference and speed by return value
```

### PID Gains and Their Effects

- **Kp (Proportional)**: Produces an output proportional to the current error. Higher values provide faster response but can cause oscillation.
- **Ki (Integral)**: Accumulates error over time to eliminate steady-state error. Higher values reduce steady-state error but can cause overshoot.
- **Kd (Derivative)**: Responds to the rate of change of error. Higher values provide damping to reduce overshoot and oscillation.

## Usage

### Basic Initialization

```cpp
#include <PIDController.h>

// Define PID gains {Kp, Ki, Kd}
const double myGains[3] = {1.5, 0.02, 0.1};

// Create a PID controller with default accumulation factors
PIDController myPID(myGains, PIDMode::PID);

// OR with custom accumulation factors
PIDController myPIDCustom(myGains, PIDMode::PID, 1.0, 0.98);
```

### Single-Axis Control

```cpp
// Calculate correction for a single variable
double correction = myPID.getCorrection(
    setpoint,     // Desired value
    measuredValue, // Current measured value
    scale         // Optional: Scale the output (default: 1.0)
);

// Apply correction to your system (e.g., motor, heater, etc.)
```

### XY Coupled Control

```cpp
double rawCorrX, rawCorrY;
double speed = myPID.getCorrection(
    setpointX, measuredX,   // X-axis setpoint and measured value
    setpointY, measuredY,   // Y-axis setpoint and measured value
    rawCorrX, rawCorrY,     // References to receive raw corrections
    maxSpeed                // Maximum allowed speed magnitude
);

// Apply corrections to your system
// speed: combined magnitude (constrained by maxSpeed)
// rawCorrX, rawCorrY: individual axis corrections (not constrained)
```

### Updating Gains and Resetting

```cpp
// Update PID gains at runtime
double newGains[3] = {2.0, 0.05, 0.2};
myPID.setGains(newGains);

// Reset integral accumulator (useful when changing setpoints or after large disturbances)
myPID.resetIntegral();
```

## Examples

The library includes three comprehensive examples demonstrating different control applications:

### Straight Line Control

**File:** `examples/StraightLineControllerExample/StraightLineControllerExample.ino`

This example demonstrates how to maintain a straight line path or a specific heading using an IMU sensor. The robot traces an octagonal path, driving in straight line segments at different absolute headings.

**Mathematical Concept:**
The straight line controller uses the IMU's yaw (rotation around vertical axis) to detect deviations from the desired path. The PID controller calculates angular corrections to maintain the initial heading.

**Pseudocode:**

```plaintext
1. Initialize robot, IMU, and PID controller
2. Reset IMU yaw reference (set current orientation as 0)
3. For each heading in octagon pattern:
   a. Reset IMU yaw reference
   b. Start timer for leg duration
   c. While leg time not elapsed:
      i. Read current relative yaw from IMU
      ii. Calculate angular correction using PID:
          correction = pid.getCorrection(0, currentYaw, scale)
      iii. Command robot motion:
           robot.moveRobot(linearSpeed, targetHeading, angularCorrection)
   d. Stop robot briefly between legs
4. Complete octagon pattern
```

### Block Orientation Control

**File:** `examples/BlockOrientationControllerExample/BlockOrientationControllerExample.ino`

This example uses a Pixy2 camera to track a colored block and align the robot with it. The PID controller keeps the target block centered in the camera's field of view.

**Mathematical Concept:**
The block orientation controller uses the pixel error between the block's center and the camera's center as the input to the PID. The controller produces angular corrections to minimize this error.

**Pseudocode:**

```plaintext
1. Initialize robot, Pixy2 camera, ultrasonic sensor, and PID controller
2. Enter state machine:
   a. SEARCHING state:
      i. Rotate robot to scan for block
      ii. If block found, transition to ALIGNING
   b. ALIGNING state:
      i. If block lost, start timeout timer
         - If timeout exceeded, return to SEARCHING
      ii. If block visible:
         - Calculate pixel error: errorX = blockX - cameraCenter
         - Calculate PID correction:
           correction = pid.getCorrection(0, errorX, scale)
         - Apply rotation: robot.moveRobot(0, 0, correction)
         - If error < tolerance for specified time, transition to ALIGNED
   c. ALIGNED state:
      i. Read distance from ultrasonic sensor
      ii. Report alignment achieved and distance to block
      iii. After delay, return to ALIGNING
```

### Zigbee Aruco Navigation

**File:** `examples/ZigbeeArucoControllerExample/ZigbeeArucoControllerExample.ino`

This example demonstrates precision navigation to specific coordinates using external position tracking via Zigbee and Aruco markers. It uses two PID controllers: one for XY position and another for heading maintenance.

**Mathematical Concept:**
The XY navigation controller uses distance errors in X and Y to calculate a velocity vector toward the target. The magnitude of this vector is constrained to the maximum speed, while its direction determines the robot's heading. A separate PID controller maintains the robot's orientation during travel.

**Pseudocode:**

```plaintext
1. Initialize robot, IMU, Zigbee communication, and two PID controllers
2. Define target coordinates (X, Y)
3. Main loop:
   a. Update pose data from Zigbee
   b. If valid pose:
      i. Calculate vector to target: (dx, dy)
      ii. Calculate distance to target
      iii. If distance < tolerance:
           - Target reached, stop robot
      iv. Otherwise:
          - Calculate XY corrections using pidXY:
            linearSpeed = pidXY.getCorrection(
                0, dx, 0, dy, rawCorrX, rawCorrY, maxSpeed)
          - Calculate yaw correction using straightLineYaw:
            yawCorrection = straightLineYaw.getCorrection(
                0, currentYaw, scale)
          - Calculate heading to target: atan2(dx, dy)
          - Command robot motion:
            robot.moveRobot(linearSpeed, targetHeading, yawCorrection)
   c. If no valid pose, stop robot
```

## Class Reference

### PIDController Class

#### Enumerations

```cpp
enum class PIDMode : uint8_t
{
    P,   // Proportional only
    PD,  // Proportional and Derivative
    PID  // Proportional, Integral, and Derivative
};
```

#### Constructor

```cpp
PIDController(const double gains[3],
              PIDMode mode = PIDMode::PID,
              double accFactorPresent = 1.0,
              double accFactorPast = 0.995);
```

- `gains`: Array of 3 doubles representing {Kp, Ki, Kd}
- `mode`: Control mode (P, PD, or PID)
- `accFactorPresent`: Factor applied to current error in integral calculation (0.0 to 1.0)
- `accFactorPast`: Factor applied to previous accumulated error (0.0 to 1.0)

#### Methods

```cpp
// Single-axis control
double getCorrection(double setpoint,
                    double measuredValue,
                    double scale = 1.0);

// XY coupled control
double getCorrection(double setpointX, double measuredX,
                    double setpointY, double measuredY,
                    double &corrX_out, double &corrY_out,
                    double maxSpeed);

// Update PID gains
void setGains(const double gains[3]);

// Reset integral accumulators and previous error states
void resetIntegral();
```

#### Private Variables

- `_Kp`, `_Ki`, `_Kd`: PID gain coefficients
- `_mode`: Current PID mode (P, PD, or PID)
- `_accFactorPresent`, `_accFactorPast`: Integral accumulation factors
- `_prevError`, `_accumulatedError`: State variables for single-axis control
- `_prevErrorX`, `_accumulatedErrorX`, `_prevErrorY`, `_accumulatedErrorY`: State variables for XY control

## Files in the Repository

- **PIDController.h**: Class declaration, enumerations, and method prototypes
- **PIDController.cpp**: Implementation of the PID controller algorithms
- **keywords.txt**: Arduino IDE syntax highlighting definitions
- **library.properties**: Library metadata for the Arduino Library Manager
- **README.md**: This documentation file
- **examples/**: Directory containing example sketches
  - **StraightLineControllerExample/**: Demonstrates heading maintenance
  - **BlockOrientationControllerExample/**: Demonstrates visual target tracking
  - **ZigbeeArucoControllerExample/**: Demonstrates precision XY navigation

## Tuning Guide

Tuning PID controllers is often specific to the system being controlled, but these guidelines can help:

### Manual Tuning Method

1. **Set Ki and Kd to 0**
2. **Increase Kp** until the system responds quickly but may oscillate
3. **Increase Kd** to dampen oscillations
4. **Increase Ki** to eliminate steady-state error

### Binary Search Tuning Method (Used in Examples)

This method systematically finds optimal PID values through a form of binary search:

1. **Start with Kp only** (Set Ki = Kd = 0)
   - Begin with Kp = 1.0
   - Test the system response:
     - If it undershoots (too slow), double Kp (e.g., from 1 to 2)
     - If it overshoots (oscillates), halve Kp (e.g., from 2 to 1)
   - Continue until you find both an upper limit (overshooting) and lower limit (undershooting)
   - Then average the upper and lower limits and test again, refining the search range

2. **Example for finding Kp = 3.25:**
   - Start with Kp = 1 → Undershoots → Try Kp = 2
   - Kp = 2 → Still undershoots → Try Kp = 4
   - Kp = 4 → Overshoots → Now we have upper (4) and lower (2) bounds
   - Try average: Kp = (2+4)/2 = 3 → Undershoots → New lower bound is 3
   - Try average: Kp = (3+4)/2 = 3.5 → Overshoots → New upper bound is 3.5
   - Try average: Kp = (3+3.5)/2 = 3.25 → Performs well

3. **After finding Kp:**
   - Apply the same binary search process for Kd (starting with 0.1)
   - Finally, tune Ki (also starting with 0.1)

This approach is more systematic than pure trial-and-error and often converges to good values faster than other methods.

### Ziegler-Nichols Method

1. Set Ki and Kd to 0
2. Increase Kp until the system oscillates with consistent amplitude
3. Record this "ultimate gain" (Ku) and the oscillation period (Tu)
4. Set gains based on the desired controller type:
   - P: `Kp = 0.5 * Ku`
   - PD: `Kp = 0.45 * Ku, Kd ≈ Kp * Tu / 8`
   - PID: `Kp = 0.6 * Ku, Ki ≈ 2 * Kp / Tu, Kd ≈ Kp * Tu / 8`

### Effects of Each Gain

- **Increasing Kp**: Faster response, potentially more overshoot
- **Increasing Ki**: Reduces steady-state error, potentially more overshoot
- **Increasing Kd**: Reduces overshoot and oscillations, potentially more sensitive to noise

## Integral Accumulation Factors

The library uses two factors to fine-tune the integral behavior:

```cpp
_accumulatedError = currError * _accFactorPresent + _accumulatedError * _accFactorPast;
```

- **accFactorPresent**: Controls the weight of the current error in accumulation (default: 1.0)
- **accFactorPast**: Controls how much of the past accumulated error is retained (default: 0.995)

### Recommended Settings

- **Fast Response Systems** (e.g., heading hold):
  `accFactorPresent = 1.0, accFactorPast = 0.995` (default - slight decay)

- **Medium Response Systems** (e.g., block centering):
  `accFactorPresent = 1.0, accFactorPast = 0.99` (faster decay)

- **Slow Response Systems** (e.g., position control):
  `accFactorPresent = 1.0, accFactorPast = 0.98` (even faster decay)

Setting `accFactorPast` below 1.0 creates a "leaky integrator" that prevents excessive integral windup.

## Troubleshooting

### Common Issues

- **Oscillation**: Decrease Kp or increase Kd
- **Slow Response**: Increase Kp
- **Steady-State Error**: Increase Ki or check for mechanical limitations
- **Integral Windup**: Use `resetIntegral()` when changing setpoints, or adjust accumulation factors
- **NaN Results**: Check for invalid inputs, especially during initialization

### Debugging

The library includes error detection for NaN inputs and outputs. If the controller returns NaN, it indicates that one of the inputs was invalid, or an internal calculation resulted in an invalid value.

```cpp
if (isnan(correction)) {
  Serial.println("PID calculation resulted in NaN. Check inputs and gains.");
}
```
