#include <PIDController.h>
#include <SensorSuite.h> // Requires SensorSuite library (Pixy2, Ultrasonic)
#include <RobotBase.h>   // Requires RobotBase library
#include <math.h>        // For fabsf

#define Kp 4
#define Ki 0.01
#define Kd 0.05

#define ORANGE_SIG 1               // Target block signature (adjust to match your PixyMon setup)
#define SEARCH_ROT_SPEED 195.0     // PWM speed for searching
#define MAX_ALIGN_ROT_SPEED 1.0    // Max PWM speed scaling during PID alignment
#define PIXEL_TOLERANCE 2          // Allowable pixel error from center
#define ALIGN_STABLE_MS 3000       // Must be aligned for this duration (ms)
#define BLOCK_LOST_TIMEOUT_MS 1000 // Timeout before reverting to search (ms)

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
const double blockGains[3] = {Kp, Ki, Kd};
PIDController blockOrientationController(blockGains, PIDMode::PID, 1.0, 0.99);

// State Machine
enum class State
{
    SEARCHING,
    ALIGNING
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

    Serial.println("Initializing Robot Base...");
    robot.begin();

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

            robot.moveRobot(150, 0, rotationalCorrection);

            if (fabsf(errorX) < PIXEL_TOLERANCE)
            {
                robot.stop();
            }

            // Block Width Check: If the block is too wide, stop i.e. the puck is secured
            if (pixy.getBlock(largestBlock) && largestBlock.width >= 270)
            {
                digitalWrite(puckSecureLEDPin, HIGH);
                digitalWrite(redLEDPin, HIGH);
                robot.stop();
                while (1)
                    ;
            }

            digitalWrite(puckSecureLEDPin, LOW);
            digitalWrite(redLEDPin, LOW);
        }
        break;
    }

    // Main loop delay
    delay(20);
}