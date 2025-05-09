# SensorSuite

A comprehensive Arduino library for interfacing with common robotics sensors.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Class Logic and Implementation](#class-logic-and-implementation)
  - [IMU](#imu)
  - [UltrasonicSensor](#ultrasonicsensor)
  - [Pixy2Handler](#pixy2handler)
  - [ZigbeeAruco](#zigbeearuco)
- [Usage Examples](#usage-examples)
  - [IMU Example](#imu-example)
  - [Ultrasonic Example](#ultrasonic-example)
  - [Pixy2 Example](#pixy2-example)
  - [Zigbee/Aruco Example](#zigbeearuco-example)
- [API Reference](#api-reference)
  - [IMU Class](#imu-class)
  - [UltrasonicSensor Class](#ultrasonicsensor-class)
  - [Pixy2Handler Class](#pixy2handler-class)
  - [ZigbeeAruco Class](#zigbeearuco-class)
- [Repository Structure](#repository-structure)

## Overview

SensorSuite is a collection of sensor interface classes designed for robotics applications. It provides simple, consistent interfaces for working with various sensors:

- **IMU**: Interface for the Adafruit BNO055 9-DOF Inertial Measurement Unit
- **UltrasonicSensor**: Interface for single-pin ultrasonic distance sensors (PING))) style)
- **Pixy2Handler**: Interface for the Pixy2 camera for color-based object detection
- **ZigbeeAruco**: Interface for receiving position data from a Zigbee/Serial device using Aruco markers

## Installation

1. Download the library as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library...
3. Select the downloaded ZIP file
4. The library and examples should now be available

## Dependencies

The library depends on:

- Adafruit Unified Sensor
- Adafruit BNO055
- Pixy2

You can install these through the Arduino Library Manager.

## Class Logic and Implementation

### IMU

The IMU class provides an interface to the Adafruit BNO055 9-DOF sensor, which combines an accelerometer, gyroscope, and magnetometer to provide absolute orientation data.

**Core Features:**

- Initialization and calibration management
- Reading absolute orientation (relative to magnetic north)
- Tracking relative orientation (relative to a defined "zero" position)
- Accessing raw pitch, roll, and yaw data

**Implementation Logic:**

```plaintext
Constructor:
- Initialize BNO055 object with default address

begin():
- Try to initialize the BNO055 sensor
- Store initial yaw as reference point if successful
- Return success/failure status

getAbsoluteYaw():
- If not initialized, return NAN
- Read Euler angles from BNO055
- Return X component (heading/yaw)

getRelativeYaw():
- If not initialized, return NAN
- Get current absolute yaw
- Calculate difference from initial yaw
- Normalize to range (-180, 180]
- Return normalized value

getPitch()/getRoll():
- If not initialized, return NAN
- Read Euler angles from BNO055
- Return appropriate component

resetYaw():
- If not initialized, return false
- Get current absolute yaw
- Set as new initial yaw
- Return success/failure
```

### UltrasonicSensor

The UltrasonicSensor class provides an interface to single-pin ultrasonic distance sensors, which measure distance by sending ultrasonic pulses and timing the echo return.

**Core Features:**

- Simple single-pin interface
- Distance measurement in centimeters
- Built-in timeout prevention

**Implementation Logic:**

```plaintext
Constructor:
- Store the trigger/echo pin

begin():
- Set pin as OUTPUT initially
- Set initial state to LOW

readDistanceCm():
- Set pin as OUTPUT
- Send LOW-HIGH-LOW pulse sequence (trigger)
- Set pin as INPUT
- Measure pulse duration with timeout
- Convert duration to distance using speed of sound
- Return distance or NAN on timeout/error
```

### Pixy2Handler

The Pixy2Handler class provides an interface to the Pixy2 camera, which can detect colored objects and provide information about their position and size.

**Core Features:**

- Camera initialization
- Block detection and tracking
- Focus on largest detected block
- Access to block properties (position, size, signature)

**Implementation Logic:**

```plaintext
Constructor:
- Initialize internal state

begin():
- Initialize Pixy2 camera
- Return result code (0 = success)

updateBlocks():
- Request new block data from Pixy2
- Reset largest block tracking
- If blocks found, find the largest one based on area
- Store largest block's properties
- Return number of blocks found or error code

getNumBlocks():
- Return number of blocks detected in last update

getBlock():
- If no blocks detected, return false
- If index specified, get that block
- Otherwise, return largest block information
- Return true if successful
```

### ZigbeeAruco

The ZigbeeAruco class provides an interface to receive position data from a Zigbee/Serial device that uses Aruco markers for positioning.

**Core Features:**

- Serial communication with a Zigbee device
- Custom protocol handling
- Position data parsing and validation
- Access to X/Y coordinates and timestamp

**Implementation Logic:**

```plaintext
Constructor:
- Store reference to communication stream
- Initialize buffer and state

begin():
- Clear any existing data in stream
- Reset buffer and state

requestPose():
- Send request character ('?') to Zigbee device

updatePoseData():
- Request new pose data
- Read available bytes from stream
- Add to buffer until complete message received
- Parse message when complete
- Return true if new, valid message processed

parseResponse():
- Check message format and delimiters
- Extract match flag, timestamp, X, Y components
- Validate data
- Update internal state
- Set validity flag based on parsing result

getPose()/getX()/getY()/getTimestamp():
- Return current position data or defaults if invalid
```

## Usage Examples

### IMU Example

This example demonstrates how to initialize and read orientation data from the BNO055 IMU sensor.

```cpp
#include <SensorSuite.h>

IMU imuSensor;

void setup() {
  Serial.begin(115200);

  // Initialize the IMU
  if (!imuSensor.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) delay(10);
  }

  // Reset relative yaw to use current position as reference
  imuSensor.resetYaw();
}

void loop() {
  // Read orientation data
  double relativeYaw = imuSensor.getRelativeYaw();
  double pitch = imuSensor.getPitch();
  double roll = imuSensor.getRoll();

  // Print data if valid (not NAN)
  if (!isnan(relativeYaw)) {
    Serial.print("Yaw: ");
    Serial.print(relativeYaw);
    Serial.print(" | Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Roll: ");
    Serial.println(roll);
  }

  delay(100);  // Update at 10Hz
}
```

### Ultrasonic Example

This example demonstrates how to initialize and read distance data from an ultrasonic sensor.

```cpp
#include <SensorSuite.h>

// Default pin is 23, but you can specify any pin
UltrasonicSensor ultrasonicSensor(23);

void setup() {
  Serial.begin(115200);

  // Initialize the sensor
  ultrasonicSensor.begin();
}

void loop() {
  // Read distance in centimeters
  float distance = ultrasonicSensor.readDistanceCm();

  if (!isnan(distance)) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("Error reading distance");
  }

  delay(100);  // Update at 10Hz
}
```

### Pixy2 Example

This example demonstrates how to initialize the Pixy2 camera and detect colored objects.

```cpp
#include <SensorSuite.h>

Pixy2Handler pixyHandler;
PixyBlock largestBlock;

void setup() {
  Serial.begin(115200);

  // Initialize the Pixy2 camera
  int8_t result = pixyHandler.begin();
  if (result != 0) {
    Serial.print("Failed to initialize Pixy2: ");
    Serial.println(result);
    while (1) delay(10);
  }
}

void loop() {
  // Update block data from camera
  int8_t blocksFound = pixyHandler.updateBlocks();

  if (blocksFound > 0) {
    // Get data for the largest block
    if (pixyHandler.getBlock(largestBlock)) {
      Serial.print("Block detected: Sig=");
      Serial.print(largestBlock.signature);
      Serial.print(" X=");
      Serial.print(largestBlock.x);
      Serial.print(" Y=");
      Serial.print(largestBlock.y);
      Serial.print(" W=");
      Serial.print(largestBlock.width);
      Serial.print(" H=");
      Serial.println(largestBlock.height);
    }
  } else {
    Serial.println("No blocks detected");
  }

  delay(100);  // Update at 10Hz
}
```

### ZigbeeAruco Example

This example demonstrates how to receive position data from a Zigbee device using Aruco markers.

```cpp
#include <SensorSuite.h>

// Using Serial1 for Zigbee communication
ZigbeeAruco zigbeePose(Serial1);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // Zigbee serial port

  // Initialize the handler
  zigbeePose.begin();
}

void loop() {
  // Update and parse position data
  if (zigbeePose.updatePoseData()) {
    // Get the latest position data
    PoseData currentPose = zigbeePose.getPose();

    if (currentPose.valid) {
      Serial.print("Position: X=");
      Serial.print(currentPose.x);
      Serial.print(" Y=");
      Serial.print(currentPose.y);
      Serial.print(" Time=");
      Serial.println(currentPose.timestamp);
    } else {
      Serial.println("Invalid position data");
    }
  }

  delay(200);  // Update at 5Hz
}
```

## API Reference

### IMU Class

#### **Constructor and Initialization**

```cpp
IMU();                  // Constructor
bool begin();           // Initialize the sensor, returns success/failure
```

#### **Orientation Reading**

```cpp
double getAbsoluteYaw() const;  // Get absolute yaw (0-360°)
double getRelativeYaw() const;  // Get relative yaw (-180 to 180°)
double getPitch() const;        // Get pitch angle (-90 to 90°)
double getRoll() const;         // Get roll angle (-180 to 180°)
```

#### **Calibration**

```cpp
bool resetYaw();  // Reset relative yaw reference point to current position
```

### UltrasonicSensor Class

#### **Constants**

```cpp
static constexpr uint8_t DEFAULT_ULTRASONIC_PIN = 23;    // Default pin
static constexpr float SPEED_OF_SOUND_CM_US = 0.0343;    // Speed of sound
static constexpr unsigned long PULSE_TIMEOUT_US = 15000;  // Pulse timeout
```

#### **Constructor and Initialization**

```cpp
UltrasonicSensor(uint8_t trigEchoPin = DEFAULT_ULTRASONIC_PIN);  // Constructor
void begin();  // Initialize the sensor
```

#### **Distance Reading**

```cpp
float readDistanceCm();  // Read distance in centimeters
```

### Pixy2Handler Class

#### **Constants**

```cpp
static constexpr int FRAME_WIDTH = 316;   // Pixy2 frame width
static constexpr int FRAME_HEIGHT = 208;  // Pixy2 frame height
```

#### **Constructor and Initialization**

```cpp
Pixy2Handler();      // Constructor
int8_t begin();      // Initialize the camera, returns 0 on success
```

#### **Block Detection**

```cpp
int8_t updateBlocks();                           // Update block data, returns number of blocks
uint8_t getNumBlocks() const;                    // Get number of detected blocks
bool getBlock(PixyBlock &block, uint8_t index);  // Get block data by index (-1 for largest)
```

#### **PixyBlock Structure**

```cpp
struct PixyBlock {
  uint16_t signature;  // Color signature ID
  uint16_t x;          // X center position (0-315)
  uint16_t y;          // Y center position (0-207)
  uint16_t width;      // Block width
  uint16_t height;     // Block height
  uint8_t age;         // Tracking age (frames)
  uint32_t area;       // Calculated area
};
```

### ZigbeeAruco Class

#### **Constants**

```cpp
static constexpr size_t RESPONSE_LENGTH = 14;            // Expected response length
static constexpr unsigned long RESPONSE_TIMEOUT_MS = 5000;  // Response timeout
static constexpr char REQUEST_CHAR = '?';                // Request character
static constexpr char DELIMITER_CHAR = ',';              // Response delimiter
```

#### **Constructor and Initialization**

```cpp
ZigbeeAruco(Stream &communicationStream);  // Constructor
void begin();                              // Initialize communication
```

#### **Position Data Handling**

```cpp
void requestPose();            // Request position data
bool updatePoseData();         // Update and parse data, returns true if new valid data
PoseData getPose() const;      // Get latest position data
bool hasValidPose() const;     // Check if position data is valid
double getX() const;           // Get X position
double getY() const;           // Get Y position
double getTimestamp() const;   // Get timestamp
```

#### **PoseData Structure**

```cpp
struct PoseData {
  bool valid;      // Data validity flag
  double timestamp;  // Timestamp
  double x;        // X position
  double y;        // Y position
};
```

## Repository Structure

```plaintext
SensorSuite/
├── examples/                      # Example sketches
│   ├── IMUExample/                # IMU usage example
│   │   └── IMUExample.ino
│   ├── PixyExample/               # Pixy2 usage example
│   │   └── PixyExample.ino
│   ├── UltrasonicExample/         # Ultrasonic sensor usage example
│   │   └── UltrasonicExample.ino
│   └── ZigbeeExample/             # Zigbee/Aruco usage example
│       └── ZigbeeExample.ino
├── keywords.txt                   # Arduino IDE keywords for syntax highlighting
├── library.properties             # Library metadata
├── SensorSuite.cpp                # Implementation of all classes
└── SensorSuite.h                  # Header file defining all classes
```

### Files Description

- **SensorSuite.h**: Header file containing all class declarations, structures, and constants.
- **SensorSuite.cpp**: Implementation file containing all method definitions.
- **examples/**: Directory containing example sketches for each sensor type.
- **keywords.txt**: Defines keywords for Arduino IDE syntax highlighting.
- **library.properties**: Metadata file for Arduino Library Manager.
