// SensorSuite/examples/IMUExample/IMUExample.ino

#include <SensorSuite.h> // Includes IMU class

// Create an IMU instance using the default I2C address and Wire interface
IMU imuSensor;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // 10 Hz

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial connection

    Serial.println("IMU Example");
    Serial.println("Initializing BNO055 IMU...");
    if (!imuSensor.begin())
    {
        Serial.println("Failed to initialize BNO055. Check wiring/address.");
        Serial.println("Stopping execution.");
        while (1)
            delay(10); // Halt execution
    }

    Serial.println("IMU Initialized Successfully.");
    Serial.println("Resetting relative yaw origin...");
    imuSensor.resetYaw(); // Set current orientation as the reference for relative yaw
    Serial.println("Yaw origin set.");

    Serial.println("Starting orientation output (Yaw/Pitch/Roll) at ~10 Hz:");
}

void loop()
{
    unsigned long currentTime = millis();

    // Print data at the specified interval (approximately 10 Hz)
    if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
    {
        lastPrintTime = currentTime;

        // Get orientation data
        double relativeYaw = imuSensor.getRelativeYaw();
        double pitch = imuSensor.getPitch();
        double roll = imuSensor.getRoll();
        double absoluteYaw = imuSensor.getAbsoluteYaw();

        // Check if data is valid (not NAN)
        if (!isnan(relativeYaw) && !isnan(pitch) && !isnan(roll) && !isnan(absoluteYaw))
        {
            Serial.print("Time: ");
            Serial.print(currentTime / 1000.0, 2);
            Serial.print("s | ");
            Serial.print("Yaw (Rel): ");
            Serial.print(relativeYaw, 1);
            Serial.print(" | ");
            Serial.print("Pitch: ");
            Serial.print(pitch, 1);
            Serial.print(" | ");
            Serial.print("Roll: ");
            Serial.print(roll, 1);
            Serial.print(" | Yaw (Abs): ");
            Serial.print(absoluteYaw, 1);
            Serial.println();
        }
        else
        {
            Serial.println("Failed to read valid IMU data.");
        }
    }

    // Add a small delay to prevent busy-waiting
    delay(5);
}