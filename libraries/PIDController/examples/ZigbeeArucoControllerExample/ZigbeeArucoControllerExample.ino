// PIDController/examples/ZigbeeArucoControllerExample/ZigbeeArucoControllerExample.ino
#include <PIDController.h>
#include <SensorSuite.h> // Requires SensorSuite (ZigbeeAruco, IMU)
#include <RobotBase.h>   // Requires RobotBase
#include <math.h>        // For sqrt, atan2, fabsf, isnan

// Configuration - Robot Hardware
const int pwmPins[3] = {4, 5, 6};
const int dirPins[3] = {49, 47, 45};
const bool reverseFlags[3] = {true, false, true};
RobotBase robot(pwmPins, dirPins, reverseFlags);

// Sensors
IMU imuSensor;

// Define the Serial port which is connected to the Zigbee module
// For Mega: Serial1 (pins 19/18), Serial2 (17/16), Serial3 (15/14)
#define ZIGBEE_SERIAL Serial1
#define ZIGBEE_BAUD_RATE 115200

ZigbeeAruco zigbee(ZIGBEE_SERIAL);

// Target Coordinates (Units should match Zigbee output, i.e., cm)
#define TARGET_X 55
#define TARGET_Y 120

#define TOL_CM 2.0                 // Tolerance radius in cm to consider target reached
#define MAX_ANGULAR_CORRECTION 1.0 // Max PWM output scaling for yaw correction
#define MAX_LINEAR_SPEED 180.0     // Max PWM output for linear correction

// PID Parameters: XY Navigation
#define XY_Kp 15
#define XY_Ki 0.01
#define XY_Kd 0.15

// PID Parameters: Yaw Hold
#define YAW_Kp -16
#define YAW_Ki 0.1
#define YAW_Kd 0.5

#define PRINT_INTERVAL_MS 500 // Print status every 500 ms

// PID Controllers
// Gains for XY position control
const double xyGains[3] = {XY_Kp, XY_Ki, XY_Kd};
// Using accumulation factors suitable for slower navigation tasks
PIDController pidXY(xyGains, PIDMode::PID, 1.0, 0.98);

// Gains for Yaw Hold
const double yawGains[3] = {YAW_Kp, YAW_Ki, YAW_Kd};
PIDController straightLineYaw(yawGains, PIDMode::PID);

// Timing
unsigned long lastPrintTime = 0;

// Initial Pose
bool initialPose = false;

// Heading offset - used to adjust the robot's heading based on initial position
double headingOffset = 0.0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("Zigbee Aruco Controller Example - XY Navigation");

    // Initialize Zigbee Serial Port
    ZIGBEE_SERIAL.begin(ZIGBEE_BAUD_RATE);
    while (!ZIGBEE_SERIAL)
        delay(10); // Wait for port

    Serial.println("Initializing Robot Base...");
    robot.begin();

    Serial.println("Initializing Zigbee Handler...");
    zigbee.begin(); // Clear buffers

    Serial.println("Initializing IMU...");
    if (!imuSensor.begin())
    {
        Serial.println("FATAL: IMU failed. Check wiring.");
        while (1)
            delay(100);
    }
    Serial.println("Resetting IMU Yaw...");
    imuSensor.resetYaw(); // Set current orientation as 0 relative yaw

    Serial.println("Initialization Complete. Starting navigation...");
    Serial.print("Target: (");
    Serial.print(TARGET_X);
    Serial.print(", ");
    Serial.print(TARGET_Y);
    Serial.println(")");
}

void loop()
{
    unsigned long currentTime = millis();

    // Update Pose Data
    // updatePoseData sends '?' and reads/parses response if available
    bool poseUpdated = zigbee.updatePoseData();

    // Control Logic
    if (zigbee.hasValidPose())
    {
        PoseData currentPose = zigbee.getPose();

        if (!initialPose)
        {
            initialPose = true; // Set initial pose flag

            // If the robot is facing downwards (y > 110), adjust heading offset
            if (currentPose.y > 110)
            {
                headingOffset = 180.0;
            }
        }

        // Calculate vector to target
        double dx = TARGET_X - currentPose.x; // Error in X
        double dy = TARGET_Y - currentPose.y; // Error in Y
        double distanceToTarget = sqrt(dx * dx + dy * dy);

        // Check if Target Reached
        if (distanceToTarget < TOL_CM)
        {
            robot.stop();
            if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
            {                                // Print once reached
                lastPrintTime = currentTime; // Prevent spamming
                Serial.println("\n TARGET REACHED! ");
                Serial.print("Final Pose: (");
                Serial.print(currentPose.x, 1);
                Serial.print(", ");
                Serial.print(currentPose.y, 1);
                Serial.print(")");
                Serial.print(" Distance: ");
                Serial.println(distanceToTarget, 1);
            }
            // Halt execution for this example once target is reached
            while (1)
                delay(100);
        }
        else
        {
            // Target Not Reached - Calculate PID Corrections

            // 1. XY Position Correction
            //    Using the PID's XY overload. We want to drive the error (-dx, -dy) towards zero.
            //    Setpoint is 0 error. Measured value is the negative error (-dx, -dy).

            double rawCorrX, rawCorrY; // Variables to receive raw corrections by reference
            double calculatedLinSpeed = pidXY.getCorrection(
                0.0, dx,            // X setpoint, X measurement (-dx)
                0.0, dy,            // Y setpoint, Y measurement (-dy)
                rawCorrX, rawCorrY, // Refs to store raw X/Y corrections
                MAX_LINEAR_SPEED    // The maximum speed limit
            );

            // Check if PID returned NAN
            if (isnan(calculatedLinSpeed))
            {
                Serial.println("PID calculation resulted in NAN. Stopping.");
                robot.stop();
                delay(100);
                return; // Skip rest of loop iteration
            }

            // 2. Yaw Correction (Maintain 0 relative yaw - i.e., hold initial heading)
            double currentRelativeYaw = imuSensor.getRelativeYaw();
            double yawCorrection = 0.0;
            if (!isnan(currentRelativeYaw))
            {
                yawCorrection = straightLineYaw.getCorrection(
                    0.0,                   // Setpoint (target relative yaw = 0)
                    currentRelativeYaw,    // Measured relative yaw
                    MAX_ANGULAR_CORRECTION // Scale factor
                );
            }
            else
            {
                Serial.println("Warning: IMU read error, yaw correction disabled.");
            }

            // Convert Corrections to Robot Motion Command
            // Heading: Calculate the angle of the vector pointing from the robot (x,y) to the target (Tx,Ty).
            // atan2(dy, dx) gives the angle of vector (dx, dy) CCW from the positive X-axis.
            // We need the angle relative to the Robot's +Y (Forward) axis, Clockwise positive.
            // Vector from robot to target is (dx, dy).
            // Angle of (dx, dy) relative to +Y (CCW) is atan2(dx, dy).
            double targetAngleRad_CCW_fromY = atan2(dx, dy);
            double targetHeadingDeg_CW_fromY = (targetAngleRad_CCW_fromY * RAD_TO_DEG) + headingOffset; // Convert to degrees

            // Send Command to Robot
            robot.moveRobot(calculatedLinSpeed, targetHeadingDeg_CW_fromY, yawCorrection);

            // Print Status Periodically
            if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
            {
                lastPrintTime = currentTime;
                Serial.print("Pose:(");
                Serial.print(currentPose.x, 1);
                Serial.print(",");
                Serial.print(currentPose.y, 1);
                Serial.print(") ");
                Serial.print(" dX:");
                Serial.print(dx, 1);
                Serial.print(" dY:");
                Serial.print(dy, 1);
                Serial.print(" Dist:");
                Serial.print(distanceToTarget, 1);
                Serial.print(" | Head:");
                Serial.print(targetHeadingDeg_CW_fromY, 1);
                Serial.print(" Spd:");
                Serial.print(calculatedLinSpeed, 0);
                Serial.print(" YawErr:");
                Serial.print(currentRelativeYaw, 1);
                Serial.print(" YawCorr:");
                Serial.println(yawCorrection, 0);
            }
        } // End target not reached block
    }
    else
    {
        // No valid pose data from Zigbee
        robot.stop(); // Stop the robot if positioning is lost
        if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
        {
            lastPrintTime = currentTime;
            Serial.println("Waiting for valid Zigbee pose data... Robot stopped.");
        }
    }

    // Main loop delay
    delay(20);
}