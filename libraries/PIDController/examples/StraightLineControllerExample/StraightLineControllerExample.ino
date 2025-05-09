// PIDController/examples/StraightLineControllerExample/StraightLineControllerExample.ino

#include <PIDController.h>
#include <SensorSuite.h> // Requires SensorSuite library installed
#include <RobotBase.h>   // Requires RobotBase library installed
#include <math.h>        // For isnan

#define Kp -16
#define Ki 0.1
#define Kd 0.5

#define LEG_DURATION_S 1.5                                       // Duration of each leg in seconds
#define LEG_DURATION_MS (unsigned long)(LEG_DURATION_S * 1000.0) // Duration in milliseconds
#define LINEAR_SPEED 180                                         // Speed along the leg
#define MAX_ANGULAR_CORRECTION 1.0                               // Scale PID output to PWM range
#define PRINT_INTERVAL_MS 200                                    // Print status every 200 ms

// Configuration - Robot Hardware Pins
const int pwmPins[3] = {4, 5, 6};
const int dirPins[3] = {49, 47, 45};
const bool reverseFlags[3] = {true, false, true};
RobotBase robot(pwmPins, dirPins, reverseFlags);

// IMU Sensor
IMU imuSensor;

// PID Controller for Heading Hold
const double headingGains[3] = {Kp, Ki, Kd};
PIDController straightLineController(headingGains, PIDMode::PID);

// Octagon Pattern Parameters
const double HEADINGS_DEG[] = {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 0.0}; // 8 headings
const int NUM_HEADINGS = sizeof(HEADINGS_DEG) / sizeof(HEADINGS_DEG[0]);

// Timing and State
int currentHeadingIndex = 0;
unsigned long legStartTime = 0;
unsigned long lastPrintTime = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial connection

    Serial.println("Straight Line Controller Example - Octagon Trace");

    Serial.println("Initializing Robot Base...");
    robot.begin();

    Serial.println("Initializing IMU...");
    if (!imuSensor.begin())
    {
        Serial.println("FATAL: IMU failed to initialize. Check wiring/address.");
        while (1)
            delay(100); // Halt
    }
    Serial.println("Resetting IMU Yaw...");
    imuSensor.resetYaw(); // Set the current orientation as the zero reference

    delay(10000);

    Serial.println("Initialization Complete. Starting Octagon Trace...");
    legStartTime = millis(); // Start the first leg timer
    Serial.print("Starting Leg 1/8: Heading ");
    Serial.println(HEADINGS_DEG[currentHeadingIndex]);
}

void loop()
{
    unsigned long currentTime = millis();

    // Check if the current leg duration has elapsed
    if (currentTime - legStartTime >= LEG_DURATION_MS)
    {
        Serial.println("Leg complete. Stopping briefly.");
        robot.stop();
        delay(500);                             // Short stop before next leg
        straightLineController.resetIntegral(); // Reset PID state for the next leg

        currentHeadingIndex++; // Move to the next heading

        if (currentHeadingIndex >= NUM_HEADINGS)
        {
            Serial.println("Octagon Trace Complete!");
            // Stop permanently
            while (1)
                delay(100); // Halt after one octagon
        }

        Serial.print("Starting Leg ");
        Serial.print(currentHeadingIndex + 1);
        Serial.print("/");
        Serial.print(NUM_HEADINGS);
        Serial.print(": Heading ");
        Serial.println(HEADINGS_DEG[currentHeadingIndex]);

        // Reset IMU yaw reference before starting the next leg
        imuSensor.resetYaw();
        delay(500); // Short pause between legs

        legStartTime = millis(); // Reset the timer for the new leg
    }

    // PID Control for Heading Maintenance
    double currentRelativeYaw = imuSensor.getRelativeYaw(); // Get yaw relative to start of current leg

    if (!isnan(currentRelativeYaw))
    {
        // Setpoint is 0 degrees relative yaw (maintain initial heading of the leg)
        double angularCorrection = straightLineController.getCorrection(
            0.0,                   // Setpoint (target relative yaw)
            currentRelativeYaw,    // Measured value (current relative yaw)
            MAX_ANGULAR_CORRECTION // Scale factor for output
        );

        // Drive in a straight line along the target heading for this leg,
        // using the PID output to correct any rotational deviation.
        robot.moveRobot(LINEAR_SPEED, HEADINGS_DEG[currentHeadingIndex], angularCorrection);

        // Print status periodically
        if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
        {
            lastPrintTime = currentTime;
            Serial.print(" Target Hdg: ");
            Serial.print(HEADINGS_DEG[currentHeadingIndex]);
            Serial.print(" | Rel Yaw: ");
            Serial.print(currentRelativeYaw, 1);
            Serial.print(" | PID Out: ");
            Serial.println(angularCorrection, 1);
        }
    }
    else
    {
        // Handle IMU error - stop the robot
        robot.stop();
        if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
        {
            lastPrintTime = currentTime;
            Serial.println("WARNING: Invalid IMU reading. Robot stopped.");
        }
    }

    // Small delay
    delay(10);
}