#include <PIDController.h>
#include <SensorSuite.h> // Requires SensorSuite library (Pixy2, Ultrasonic)
#include <RobotBase.h>   // Requires RobotBase library
#include <math.h>        // For fabsf

#define Block_Kp 4
#define Block_Ki 0.01
#define Block_Kd 0.05

#define ORANGE_SIG 1               // Target block signature (adjust to match your PixyMon setup)
#define SEARCH_ROT_SPEED 195.0     // PWM speed for searching
#define MAX_ALIGN_ROT_SPEED 1.0    // Max PWM speed scaling during PID alignment
#define PIXEL_TOLERANCE 2          // Allowable pixel error from center
#define ALIGN_STABLE_MS 3000       // Must be aligned for this duration (ms)
#define BLOCK_LOST_TIMEOUT_MS 5000 // Timeout before reverting to search (ms)

// Configuration
const int pwmPins[3] = {4, 5, 6};
const int dirPins[3] = {49, 47, 45};
const bool reverseFlags[3] = {true, false, true};
RobotBase robot(pwmPins, dirPins, reverseFlags);

const int puckSecureLEDPin = 35;
const int redLEDPin = 37;

// Sensors
Pixy2Handler pixy;

// PID Controller for Block Centering
const double blockGains[3] = {Block_Kp, Block_Ki, Block_Kd};
PIDController blockOrientationController(blockGains, PIDMode::PID, 1.0, 0.99);

IMU imuSensor;

#define ZIGBEE_SERIAL Serial1
#define ZIGBEE_BAUD_RATE 115200

ZigbeeAruco zigbee(ZIGBEE_SERIAL);

#define GOAL_X1 55
#define GOAL_Y1 25

#define GOAL_X2 55
#define GOAL_Y2 210

#define TOL_CM 4.0                 // Tolerance radius in cm to consider target reached
#define MAX_ANGULAR_CORRECTION 1.0 // Max PWM output for yaw correction
#define MAX_LINEAR_SPEED 200.0     // Max PWM output for linear correction

#define XY_Kp 15
#define XY_Ki 0.01
#define XY_Kd 0.15

#define YAW_Kp -16
#define YAW_Ki 0.1
#define YAW_Kd 0.5

const double xyGains[3] = {XY_Kp, XY_Ki, XY_Kd};
PIDController pidXY(xyGains, PIDMode::PID, 1.0, 0.98);

const double yawGains[3] = {YAW_Kp, YAW_Ki, YAW_Kd};
PIDController straightLineYaw(yawGains, PIDMode::PID);

// Initial Pose and heading offset
bool initialPose = false;
double headingOffset = 0.0;

float TARGET_X = GOAL_X2;
float TARGET_Y = GOAL_Y2;

// State Machine
enum class State
{
    SEARCHING,
    ALIGNING,
    HEADING
};
State currentState = State::SEARCHING;

// Timing Variables
unsigned long alignStableStartTime = 0; // Time when alignment tolerance was first met
unsigned long blockLostStartTime = 0;   // Time when the target block was last seen

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

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

    Serial.println("Initializing Pixy2 Camera...");
    if (pixy.begin() != 0)
    {
        Serial.println("FATAL: Pixy2 failed to initialize. Check wiring/interface.");
        while (1)
            delay(100); // Halt
    }

    pinMode(puckSecureLEDPin, OUTPUT);
    digitalWrite(puckSecureLEDPin, LOW);

    pinMode(redLEDPin, OUTPUT);
    digitalWrite(redLEDPin, LOW);

    Serial.println("Initialization Complete. Starting Search...");
    currentState = State::SEARCHING;
}

void loop()
{
    unsigned long currentTime = millis();
    bool poseUpdated = zigbee.updatePoseData();

    if (!zigbee.hasValidPose())
    {
        // No valid pose data from Zigbee
        robot.stop();                  // Stop the robot if positioning is lost
        digitalWrite(redLEDPin, HIGH); // Turn on error LED
        return;
    }

    digitalWrite(redLEDPin, LOW); // Turn off error LED
    PoseData currentPose = zigbee.getPose();

    if (!initialPose)
    {
        initialPose = true; // Set initial pose flag

        // Set initial heading offset based on initial robot pose
        if (currentPose.y > 120)
        {
            headingOffset = 180.0;
            TARGET_X = GOAL_X1;
            TARGET_Y = GOAL_Y1;
        }
    }

    int8_t blocksFound = pixy.updateBlocks();
    PixyBlock largestBlock;
    bool targetVisible = false;

    if (blocksFound > 0 && pixy.getBlock(largestBlock) && largestBlock.signature == ORANGE_SIG)
    {
        targetVisible = true;
        blockLostStartTime = 0; // Reset the lost timer if target is seen
    }

    // State Machine Logic
    switch (currentState)
    {
    case State::SEARCHING:

        robot.moveRobot(0, 0, SEARCH_ROT_SPEED);
        if (targetVisible)
        {
            robot.stop();
            blockOrientationController.resetIntegral();
            currentState = State::ALIGNING;
            alignStableStartTime = 0;
            delay(100);
        }
        break;

    case State::ALIGNING:

        if (!targetVisible)
        {
            if (blockLostStartTime == 0)
            {
                blockLostStartTime = currentTime;
                robot.stop();
            }
            else if (currentTime - blockLostStartTime > BLOCK_LOST_TIMEOUT_MS)
            {
                currentState = State::SEARCHING;
            }
        }
        else
        {
            blockLostStartTime = 0;

            double errorX = largestBlock.x - (Pixy2Handler::FRAME_WIDTH / 2.0);
            double rotationalCorrection = blockOrientationController.getCorrection(
                0.0,
                errorX,
                MAX_ALIGN_ROT_SPEED);

            robot.moveRobot(180, 0, rotationalCorrection);

            if (fabsf(errorX) < PIXEL_TOLERANCE)
            {
                robot.stop();
            }

            if (pixy.getBlock(largestBlock) && largestBlock.width >= 270)
            {
                digitalWrite(puckSecureLEDPin, HIGH);
                robot.stop();
                Serial.println("Block Secured");
                currentState = State::HEADING;
                break;
            }
        }
        break;
    case State::HEADING:

        digitalWrite(puckSecureLEDPin, HIGH);
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

        yawCorrection = constrain(yawCorrection, 0, 180);
        if (fabsf(currentRelativeYaw) < 2.0)
        {
            robot.stop();

            // Start Pursuit to goal
            startGoal();
        }
        else
        {
            robot.moveRobot(150, 90, yawCorrection);
        }

        break;
    }

    // Main loop delay
    delay(20);
}

void startGoal()
{
    while (1)
    {
        bool poseUpdated = zigbee.updatePoseData();
        if (!zigbee.hasValidPose())
        {
            Serial.println("Zigbee pose update failed.");
            digitalWrite(redLEDPin, HIGH); // Turn on error LED
            robot.stop();
            continue;
        }

        digitalWrite(redLEDPin, LOW); // Turn off error LED
        PoseData currentPose = zigbee.getPose();

        // Calculate vector to target
        double dx = TARGET_X - currentPose.x; // Error in X
        double dy = TARGET_Y - currentPose.y; // Error in Y
        double distanceToTarget = sqrt(dx * dx + dy * dy);

        Serial.print("Distance to Target: ");
        Serial.println(distanceToTarget);
        Serial.print("Current Pose: ");
        Serial.print(currentPose.x);
        Serial.print(", ");
        Serial.print(currentPose.y);

        // Check if Target Reached
        if (distanceToTarget < TOL_CM)
        {
            robot.stop();
            robot.moveRobot(180, 180, 0); // Move backward for strike
            delay(1000);

            robot.stop();
            delay(500);
            robot.moveRobot(220, 0, 0); // Move forward for strike

            delay(1250);
            robot.stop();

            // Halt execution once target is reached
            while (1)
            {
                digitalWrite(puckSecureLEDPin, HIGH);
                delay(500);
                digitalWrite(puckSecureLEDPin, LOW);
                delay(500);
                Serial.println("Target Reached");
            }
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

            // Send Command to Robot
            robot.moveRobot(calculatedLinSpeed, targetHeadingDeg_CW_fromY, yawCorrection);
        }
        delay(20);
    }
}