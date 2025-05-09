// SensorSuite/examples/PixyExample/PixyExample.ino

#include <SensorSuite.h> // Includes Pixy2Handler class and PixyBlock struct

// Create a Pixy2Handler instance
Pixy2Handler pixyHandler;

// Structure to hold the data of the largest block found
PixyBlock largestBlock;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL_MS = 100; // Update and print at 10 Hz

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial connection

    Serial.println("Pixy2 Camera Example");

    Serial.println("Initializing Pixy2 Camera...");
    int8_t initResult = pixyHandler.begin();

    if (initResult != 0)
    {
        Serial.println("Stopping execution.");
        while (1)
            delay(10); // Halt
    }

    Serial.println("Pixy2 Initialized Successfully.");
    Serial.println("Starting block detection (reporting largest block) at ~10 Hz:");
}

void loop()
{
    unsigned long currentTime = millis();

    // Update and print data at the specified interval
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS)
    {
        lastUpdateTime = currentTime;

        // Ask the handler to update block data from the camera
        int8_t blocksFound = pixyHandler.updateBlocks();

        Serial.print("Time: ");
        Serial.print(currentTime / 1000.0, 2);
        Serial.print("s | ");

        if (blocksFound < 0)
        {
            Serial.print("Error getting blocks: ");
            Serial.println(blocksFound);
        }
        else if (blocksFound == 0)
        {
            Serial.println("No blocks detected.");
        }
        else
        {
            // Try to get the largest block found during the update
            if (pixyHandler.getBlock(largestBlock))
            {
                Serial.print("Largest Block: Sig=");
                Serial.print(largestBlock.signature);
                Serial.print(" X=");
                Serial.print(largestBlock.x);
                Serial.print(" Y=");
                Serial.print(largestBlock.y);
                Serial.print(" W=");
                Serial.print(largestBlock.width);
                Serial.print(" H=");
                Serial.print(largestBlock.height);
                Serial.print(" Age=");
                Serial.print(largestBlock.age);
                Serial.print(" Area=");
                Serial.println(largestBlock.area);
            }
            else
            {
                // This case should ideally not happen if blocksFound > 0, but good for testing
                Serial.println("Detected blocks, but failed to get largest block info (should not happen).");
            }
        }
    }

    // Small delay
    delay(5);
}