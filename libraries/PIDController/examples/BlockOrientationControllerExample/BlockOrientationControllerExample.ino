// PIDController/examples/BlockOrientationControllerExample/BlockOrientationControllerExample.ino

#include <PIDController.h>
#include <SensorSuite.h> // Requires SensorSuite library (Pixy2, Ultrasonic)
#include <RobotBase.h>   // Requires RobotBase library
#include <math.h>        // For fabsf

#define Kp 4
#define Ki 0.01
#define Kd 0.05

#define ORANGE_SIG 1               // Target block signature
#define SEARCH_ROT_SPEED 180.0     // PWM speed for searching
#define MAX_ALIGN_ROT_SPEED 1.0    // Max PWM speed scaling during PID alignment
#define PIXEL_TOLERANCE 2          // Allowable pixel error from center
#define ALIGN_STABLE_MS 10000      // Must be aligned for this duration (ms)
#define BLOCK_LOST_TIMEOUT_MS 1000 // Timeout before reverting to search (ms)

// Configuration - Robot Hardware
const int pwmPins[3] = {4, 5, 6};
const int dirPins[3] = {49, 47, 45};
const bool reverseFlags[3] = {true, false, true};
RobotBase robot(pwmPins, dirPins, reverseFlags);

// Sensors
Pixy2Handler pixy;

const int ultrasonicTriggerPin = 23;
UltrasonicSensor ultrasonic(ultrasonicTriggerPin);

// PID Controller for Block Centering
const double blockGains[3] = {Kp, Ki, Kd};
PIDController blockOrientationController(blockGains, PIDMode::PID, 1.0, 0.99); // Using faster integral decay

// State Machine
enum class State
{
    SEARCHING,
    ALIGNING,
    ALIGNED
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

    Serial.println("Block Orientation Controller Example");

    Serial.println("Initializing Robot Base...");
    robot.begin();

    Serial.println("Initializing Pixy2 Camera...");
    if (pixy.begin() != 0)
    {
        Serial.println("FATAL: Pixy2 failed to initialize. Check wiring/interface.");
        while (1)
            delay(100); // Halt
    }

    Serial.println("Initializing Ultrasonic Sensor...");
    ultrasonic.begin(); // Initializes pin (though mode is set dynamically)

    Serial.println("Initialization Complete. Starting Search...");
    currentState = State::SEARCHING;
}

void loop()
{
    unsigned long currentTime = millis();

    // Update Sensor Data
    int8_t blocksFound = pixy.updateBlocks(); // Get fresh block data
    PixyBlock largestBlock;
    bool targetVisible = false;

    // Check if the largest detected block matches our target signature
    if (blocksFound > 0 && pixy.getBlock(largestBlock) && largestBlock.signature == ORANGE_SIG)
    {
        targetVisible = true;
        blockLostStartTime = 0; // Reset the lost timer if target is seen
        Serial.print("Target Seen: X=");
        Serial.println(largestBlock.x);
    }

    // State Machine Logic
    switch (currentState)
    {
    case State::SEARCHING:
        // Action: Spin slowly clockwise to scan for the block
        robot.moveRobot(0, 0, SEARCH_ROT_SPEED); // 0 linear, 0 heading, positive angular speed

        // Transition: If target block becomes visible, switch to ALIGNING
        if (targetVisible)
        {
            Serial.println("Target Found! -> Switching to ALIGNING");
            robot.stop();                               // Stop the search spin
            blockOrientationController.resetIntegral(); // Reset PID state
            currentState = State::ALIGNING;
            alignStableStartTime = 0; // Reset alignment timer
            delay(100);               // Small delay to allow motion to stop
        }
        break; // End SEARCHING case

    case State::ALIGNING:
        // Check if block is lost
        if (!targetVisible)
        {
            if (blockLostStartTime == 0)
            {
                // Start the lost timer
                blockLostStartTime = currentTime;
                Serial.println("Target lost, starting timeout...");
                robot.stop(); // Stop alignment motion while waiting
            }
            else if (currentTime - blockLostStartTime > BLOCK_LOST_TIMEOUT_MS)
            {
                // Timeout exceeded, revert to searching
                Serial.println("Block lost timeout! -> Switching back to SEARCHING");
                currentState = State::SEARCHING;
                // PID reset is handled when entering ALIGNING again
            }
        }
        else
        {
            // Block is visible, perform PID alignment
            blockLostStartTime = 0; // Reset lost timer

            // Calculate error: difference between block center X and frame center X
            double errorX = largestBlock.x - (Pixy2Handler::FRAME_WIDTH / 2.0);

            // Calculate PID correction for rotation
            // Setpoint is 0 error (block centered). Scale output to max rotation speed.
            double rotationalCorrection = blockOrientationController.getCorrection(
                0.0,                // Setpoint (target error)
                errorX,             // Measured value (current error)
                MAX_ALIGN_ROT_SPEED // Scale factor
            );

            // Apply rotation-only movement
            robot.moveRobot(0, 0, rotationalCorrection);

            // Print status
            Serial.print("ALIGNING - ErrorX: ");
            Serial.print(errorX, 1);
            Serial.print(" | PID Out: ");
            Serial.println(rotationalCorrection, 1);

            // Check for stable alignment
            if (fabsf(errorX) < PIXEL_TOLERANCE)
            {
                if (alignStableStartTime == 0)
                {
                    // First time tolerance is met, start the timer
                    alignStableStartTime = currentTime;
                }
                else if (currentTime - alignStableStartTime >= ALIGN_STABLE_MS)
                {
                    // Tolerance met consistently for the required duration
                    Serial.println("Alignment Stable! -> Switching to ALIGNED");
                    robot.stop();
                    currentState = State::ALIGNED;
                }
            }
            else
            {
                // Error is outside tolerance, reset the stable timer
                alignStableStartTime = 0;
            }
        }
        break; // End ALIGNING case

    case State::ALIGNED:
        // Action: Stay stopped and report distance
        robot.stop(); // Ensure robot remains stopped

        // Read distance from ultrasonic sensor
        float distanceCm = ultrasonic.readDistanceCm();

        Serial.print("State: ALIGNED. Distance: ");
        if (!isnan(distanceCm))
        {
            Serial.print(distanceCm, 1);
            Serial.println(" cm");
        }
        else
        {
            Serial.println("Ultrasonic Sensor Timeout/Error");
        }

        // Transition back to aligning
        delay(2000); // Wait for 2 seconds
        Serial.println("-> Returning to ALIGNING state");
        currentState = State::ALIGNING;
        break; // End ALIGNED case
    }

    // Main loop delay
    delay(20);
}