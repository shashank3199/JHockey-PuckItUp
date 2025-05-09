// RobotBase/examples/BasicDrive/BasicDrive.ino

#include <RobotBase.h> // Include the RobotBase

// Create a RobotBase instance
int pwmPins[3] = {4, 5, 6};
int dirPins[3] = {49, 47, 45};
bool reverseFlags[3] = {true, false, true};
RobotBase robot(pwmPins, dirPins, reverseFlags);

// Variables to store received command values
float linearSpeed = 0.0;
float angularSpeed = 0.0;
float thetaDegrees = 0.0; // Angle is clockwise from +Y as per moveRobot comment

bool performInitailTest = true; // Flag to perform initial test

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial connection

    Serial.println("RobotBase BasicDrive Example (C-Array Version)");
    Serial.println("Initializing Robot Base...");
    robot.begin(); // Initialize motor pins for the 'robot' instance

    // Perform initial test if needed
    if (performInitailTest)
    {
        Serial.println("Performing initial motor tests...");
        for (int i = 0; i < 3; ++i)
        {
            robot.testMotor(i); // Test each motor
        }
        Serial.println("Initial test complete.");
    }

    Serial.println("Ready to receive commands over Serial.");
    Serial.println("Format: P<int>W<int>A<int>M<int>K");
    Serial.println("  P<linear_speed_magnitude>");
    Serial.println("  W<angular_speed_ccw_positive>");
    Serial.println("  A<theta_degrees_clockwise_from_forward>");
    Serial.println("  M<delay_units_100ms>");
    Serial.println("  K (terminates M command sequence & executes move/delay/stop)");
    Serial.println("Example: P100W50A30M10K  (Speed 100, Angular 50, Angle 30deg, Delay 1000ms)");
    Serial.println("Send just 'K' to stop motors immediately.");
}

void loop()
{
    while (Serial.available())
    {
        char commandChar = Serial.read();
        switch (commandChar)
        {
        case 'P':
        case 'p':
            if (Serial.peek() != '\n' && Serial.peek() != '\r')
            {
                linearSpeed = Serial.parseInt();
                Serial.print("Linear Speed set: ");
                Serial.println(linearSpeed);
            }
            break;
        case 'W':
        case 'w':
            if (Serial.peek() != '\n' && Serial.peek() != '\r')
            {
                angularSpeed = Serial.parseInt();
                Serial.print("Angular Speed set: ");
                Serial.println(angularSpeed);
            }
            break;
        case 'A':
        case 'a':
            if (Serial.peek() != '\n' && Serial.peek() != '\r')
            {
                thetaDegrees = Serial.parseInt();
                Serial.print("Theta Degrees set: ");
                Serial.println(thetaDegrees);
            }
            break;
        case 'M':
        case 'm':
            if (Serial.peek() != '\n' && Serial.peek() != '\r')
            {
                // Read delay in 100ms units
                int delayUnits = Serial.parseInt();
                int delayMs = delayUnits * 100;
                Serial.print("Delay set: ");
                Serial.print(delayMs);
                Serial.println(" ms");

                Serial.println("Executing Move-Delay-Stop sequence...");
                robot.moveRobot(linearSpeed, thetaDegrees, angularSpeed);
                delay(delayMs);
                robot.stop();
                Serial.println("Sequence complete.");
            }
            break;
        case 'K':
        case 'k':
            Serial.println("Immediate Stop Command Received: K");
            robot.stop();
            // Reset potentially partially entered command values
            linearSpeed = 0.0;
            angularSpeed = 0.0;
            thetaDegrees = 0.0;
            break;
        default:
            // Ignore other characters
            Serial.print("Ignoring character: ");
            Serial.println(commandChar);
            break;
        }
    }
}