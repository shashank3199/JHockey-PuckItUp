# JHockey-PuckItUp

An autonomous robot hockey project using Arduino-based sensors and control systems.

## Project Overview

JHockey-PuckItUp is the codebase for creating an autonomous hockey-playing robot. The project combines three-wheel omni-directional mobility, Pixy Camera Detection for puck detection, and precision navigation to create a robot capable of:

1. Finding and securing hockey pucks using visual detection
2. Navigating to specific goal positions on the playing field
3. Combining these capabilities in a complete state machine for autonomous gameplay

The system uses a triangular omni-wheel configuration, Pixy2 camera for puck detection, IMU for orientation, and Zigbee/Aruco markers for positioning.

## Repository Structure

```plaintext
JHockey-PuckItUp
├── Arduino Codes         # Main robot hockey code implementations
│   ├── GoToGoal          # Goal navigation using position tracking
│   ├── StateMachine      # Complete hockey playing state machine
│   └── TrackNSecurePuck  # Puck tracking and securing
├── libraries             # Supporting libraries
│   ├── PIDController     # PID control algorithms
│   ├── RobotBase         # Omni-wheel robot control
│   └── SensorSuite       # Sensor interfaces (IMU, Pixy2, etc.)
└── README.md             # This file
```

## System Components

### Arduino Codes

The main robot hockey functionality is implemented in three Arduino sketches:

- **[TrackNSecurePuck](Arduino%20Codes/README.md#tracknsecurepuck)**: Uses a Pixy2 camera to locate an orange hockey puck, center the robot on it, and secure it.
- **[GoToGoal](Arduino%20Codes/README.md#gotogoal)**: Uses external positioning and IMU data to navigate the robot to a specific goal location.
- **[StateMachine](Arduino%20Codes/README.md#statemachine)**: Combines puck tracking and goal navigation in a complete game strategy.

For detailed information on these implementations, see the [Arduino Codes README](Arduino%20Codes/README.md).

### Libraries

The project includes three custom libraries:

1. **[RobotBase](libraries/RobotBase/README.md)**: Controls a three-wheel omni-directional robot with a triangular wheel configuration.
   - Handles inverse kinematics for translational and rotational movement
   - Provides precise control over robot motion in any direction

2. **[SensorSuite](libraries/SensorSuite/README.md)**: Provides interfaces for various sensors used in robotics applications.
   - **IMU**: Interface for BNO055 9-DOF orientation sensor
   - **UltrasonicSensor**: Interface for distance measurement
   - **Pixy2Handler**: Interface for Pixy2 camera and object detection
   - **ZigbeeAruco**: Interface for receiving position data from external tracking

3. **[PIDController](libraries/PIDController/README.md)**: Implements robust PID control algorithms.
   - Multiple control modes (P, PD, PID)
   - XY position control for coupled systems
   - Configurable integral accumulation behavior

## Hardware Requirements

### Base Robot

- Arduino Mega or compatible microcontroller
- Three omni-directional wheels in triangular configuration
- Three DC motors with appropriate motor drivers
- Battery power supply

### Sensors

- Pixy2 camera (for puck detection)
- BNO055 9-DOF IMU (for orientation)
- Ultrasonic sensor(s) for obstacle detection
- Zigbee module for communication with external positioning system
- External Aruco marker tracking system (camera + computer)

### Indicators

- Status LEDs for error indication and puck securing

## Installation and Setup

1. Clone this repository or download as ZIP
2. Install the required libraries:
   - Copy the libraries from the `libraries` folder to your Arduino libraries directory
   - Use the Arduino Library Manager to install dependencies:
     - Adafruit Unified Sensor
     - Adafruit BNO055
     - Pixy2
3. Connect your hardware according to the pin configurations in the code
4. Upload the desired sketch to your Arduino
   - Start with individual components (TrackNSecurePuck, GoToGoal)
   - Progress to the full StateMachine once components are working

## Configuration

Key parameters that may need adjustment for your specific setup:

- **Motor Pins**: Update pin configurations in RobotBase initialization
- **PID Gains**: Tune values based on your robot's physical characteristics
- **Pixy2 Signatures**: Configure signature 1 for your orange puck color
- **Goal Positions**: Set appropriate coordinates for your hockey field

## Troubleshooting

Common issues and solutions are documented in the respective README files:

- [Arduino Codes Troubleshooting](Arduino%20Codes/README.md#troubleshooting)
- [PIDController Troubleshooting](libraries/PIDController/README.md#troubleshooting)
- [SensorSuite Troubleshooting](libraries/SensorSuite/README.md#usage-examples) (see examples)

## Examples

Each library includes example sketches to help you get started:

- **RobotBase**: Basic drive examples showing how to control the robot
- **SensorSuite**: Examples for each sensor type (IMU, Ultrasonic, Pixy2, Zigbee)
- **PIDController**: Examples for different control applications (straight line, block orientation, navigation)

## Team Members

- [Shashank Goyal](mailto:sgoyal18@jhu.edu)
- [Angel Casanova](mailto:acasano1@jhu.edu)
- [Jennifer Taylor](mailto:jtayl214@jhu.edu)
- [Valeria Vasquez-Barros](mailto:vvasque7@jhu.edu)
