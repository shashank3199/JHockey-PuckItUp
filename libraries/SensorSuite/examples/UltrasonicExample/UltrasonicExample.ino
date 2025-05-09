// SensorSuite/examples/UltrasonicExample/UltrasonicExample.ino

#include <SensorSuite.h> // Includes UltrasonicSensor class

// Create an UltrasonicSensor instance using the default pin (12)
uint8_t ultrasonicPin = 23;                       // Default pin for the ultrasonic sensor
UltrasonicSensor ultrasonicSensor(ultrasonicPin); // Create an instance of the UltrasonicSensor class

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL_MS = 100; // Read every 100 ms

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial connection

    Serial.println("Ultrasonic Sensor Example");

    // Initialization
    ultrasonicSensor.begin();

    Serial.println("Starting distance measurements every 100 ms:");
}

void loop()
{
    unsigned long currentTime = millis();

    // Read distance at the specified interval
    if (currentTime - lastReadTime >= READ_INTERVAL_MS)
    {
        lastReadTime = currentTime;

        // Read distance in centimeters
        float distance = ultrasonicSensor.readDistanceCm();

        Serial.print("Time: ");
        Serial.print(currentTime / 1000.0, 2);
        Serial.print("s | ");

        if (distance >= 0)
        { // Check if reading was successful (readDistanceCm returns < 0 on error/timeout)
            Serial.print("Distance: ");
            Serial.print(distance, 2); // Print with 2 decimal places
            Serial.println(" cm");
        }
        else
        {
            Serial.println("Distance: Error/Timeout reading sensor.");
        }
    }

    // Add a small delay, allows other tasks to run if needed
    delay(10);
}