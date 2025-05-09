// SensorSuite/examples/ZigbeeExample/ZigbeeExample.ino

#include <SensorSuite.h> // Includes ZigbeeAruco class and PoseData struct

// Configuration
// Define the Serial port to which the Zigbee module is connected
// For Arduino Mega, Serial1 is typically pins 19 (RX) and 18 (TX)
#define ZIGBEE_SERIAL Serial1 // Using Serial1 on a Mega

const unsigned long ZIGBEE_BAUD_RATE = 115200; // Baud rate for Zigbee communication

// Create a ZigbeeAruco instance, passing the chosen Serial port
ZigbeeAruco zigbeePose(ZIGBEE_SERIAL);

unsigned long lastRequestTime = 0;
const unsigned long REQUEST_INTERVAL_MS = 200; // Request pose data every 200 ms

void setup()
{
    Serial.begin(115200); // Serial monitor output
    while (!Serial)
        ;

    Serial.println("Zigbee/Aruco Pose Example");

    // Initialize the serial port for Zigbee communication
    ZIGBEE_SERIAL.begin(ZIGBEE_BAUD_RATE);
    while (!ZIGBEE_SERIAL)
        ; // Wait for Zigbee serial port to be ready

    // Initialize the ZigbeeAruco handler (clears buffers)
    zigbeePose.begin();

    Serial.println("Zigbee communication initialized.");
    Serial.println("Requesting pose data periodically...");
}

void loop()
{
    unsigned long currentTime = millis();

    // Request pose data periodically
    if (currentTime - lastRequestTime >= REQUEST_INTERVAL_MS)
    {
        lastRequestTime = currentTime;
        Serial.println("Sending pose request ('?')...");

        // Check for incoming data and parse it if a full message arrives
        if (zigbeePose.updatePoseData())
        {
            // updatePoseData returns true if a complete message was received and parsed
            Serial.println("Received and parsed a response:");

            PoseData currentPose = zigbeePose.getPose();

            Serial.print("  Timestamp: ");
            Serial.println(currentPose.timestamp, 2);
            Serial.print("  X Position: ");
            Serial.println(currentPose.x, 2);
            Serial.print("  Y Position: ");
            Serial.println(currentPose.y, 2);
            Serial.print("  Valid Data Flag: ");
            Serial.println(currentPose.valid ? "True" : "False");
            Serial.println("--------------------");
        }
        else
        {
            // No new message received or parsing failed
            Serial.println("No new message or parsing error.");
        }
    }

    // Small delay
    delay(10);
}