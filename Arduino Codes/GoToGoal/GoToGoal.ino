#include <PIDController.h>
#include <SensorSuite.h> // Requires SensorSuite (ZigbeeAruco, IMU)
#include <RobotBase.h>   // Requires RobotBase
#include <math.h>        // For sqrt, atan2, fabsf, isnan

// Robot Hardware
const int pwmPins[3] = {4, 5, 6};
const int dirPins[3] = {49, 47, 45};
const bool reverseFlags[3] = {true, false, true};
RobotBase robot(pwmPins, dirPins, reverseFlags);

// Sensors
IMU imuSensor;
const int zigbeeLedPin = 37; // Pin for Zigbee LED

// Define which Serial port is connected to the Zigbee module
// For Mega: Serial1 (pins 19/18), Serial2 (17/16), Serial3 (15/14)
#define ZIGBEE_SERIAL Serial1
#define ZIGBEE_BAUD_RATE 115200

ZigbeeAruco zigbee(ZIGBEE_SERIAL);

// Target Coordinates (Units should match Zigbee output, i.e., cm)
#define GOAL_X1 55
#define GOAL_Y1 30

#define GOAL_X2 55
#define GOAL_Y2 210

#define TOL_CM 2.0                 // Tolerance radius in cm to consider target reached
#define MAX_ANGULAR_CORRECTION 1.0 // Max PWM output scaling for yaw correction
#define MAX_LINEAR_SPEED 180.0     // Max PWM output for linear correction

#define XY_Kp 15
#define XY_Ki 0.01
#define XY_Kd 0.15

#define YAW_Kp -16
#define YAW_Ki 0.1
#define YAW_Kd 0.5

// PID Controllers
const double xyGains[3] = {XY_Kp, XY_Ki, XY_Kd};
// Using accumulation factors suitable for slower navigation tasks
PIDController pidXY(xyGains, PIDMode::PID, 1.0, 0.98);

// Gains for Yaw Hold
const double yawGains[3] = {YAW_Kp, YAW_Ki, YAW_Kd};
PIDController straightLineYaw(yawGains, PIDMode::PID);

// Timing
unsigned long lastPrintTime = 0;

// Initial Pose and heading offset
bool initialPose = false;
double headingOffset = 0.0;

float TARGET_X = GOAL_X2;
float TARGET_Y = GOAL_Y2;

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
    zigbee.begin(); // Clears buffers

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

    pinMode(zigbeeLedPin, OUTPUT);
    digitalWrite(zigbeeLedPin, LOW); // Turn off LED initially
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

        digitalWrite(zigbeeLedPin, LOW); // Turn on LED when valid pose is received

        if (!initialPose)
        {
            initialPose = true; // Set initial pose flag

            // Set initial heading offset based on the initial robot pose
            if (currentPose.y > 120)
            {
                headingOffset = 180.0;
                TARGET_X = GOAL_X1;
                TARGET_Y = GOAL_Y1;
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
            robot.moveRobot(180, 180, 0); // Move backward for strike
            delay(1500);

            robot.stop();
            delay(500);
            robot.moveRobot(200, 0, 0); // Move forward for strike

            delay(2000);
            robot.stop();

            // Halt execution for this example once target is reached
            while (1)
                delay(100);
        }
        else
        {
            // Target Not Reached - Calculate PID Corrections

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

            double targetAngleRad_CCW_fromY = atan2(dx, dy);
            double targetHeadingDeg_CW_fromY = (targetAngleRad_CCW_fromY * RAD_TO_DEG) + headingOffset; // Convert to degrees
            robot.moveRobot(calculatedLinSpeed, targetHeadingDeg_CW_fromY, yawCorrection);
        }
    }
    else
    {
        // No valid pose data from Zigbee
        robot.stop();                     // Stop the robot if positioning is lost
        digitalWrite(zigbeeLedPin, HIGH); // Turn on error LED
    }

    // Main loop delay
    delay(20);
}