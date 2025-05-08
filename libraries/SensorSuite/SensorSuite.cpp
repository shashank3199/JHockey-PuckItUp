// SensorSuite/SensorSuite.cpp

#include "SensorSuite.h"
#include <Wire.h> // For I2C communication
#include <math.h> // For NAN

// IMU Implementation

IMU::IMU()
    : _isInitialized(false)
{
    _bno = Adafruit_BNO055(55, 0x28, &Wire);
}

bool IMU::begin()
{
    if (!_bno.begin())
    {
        // Error message printed by the BNO library itself if Serial is available
        _isInitialized = false;
        Serial.println("BNO055 not found or failed to initialize.");
        return false;
    }

    // Short delay to allow sensor to stabilize after init
    delay(100);

    _isInitialized = true;
    _initialYaw = getAbsoluteYaw();

    Serial.println("BNO055 initialization status: " + String(_isInitialized ? "Success" : "Failed"));
    return _isInitialized;
}

double IMU::getAbsoluteYaw() const
{
    if (!_isInitialized)
    {
        Serial.println("IMU not initialized. Cannot get absolute yaw.");
        return NAN; // Use NAN to indicate invalid data
    }

    // BNO055 Euler angles: x = heading (yaw), y = roll, z = pitch
    sensors_event_t _orientationData;
    _bno.getEvent(&_orientationData, Adafruit_BNO055::VECTOR_EULER);
    return static_cast<double>(_orientationData.orientation.x);
}

double IMU::getRelativeYaw() const
{
    if (!_isInitialized || isnan(_initialYaw))
    {
        Serial.println("IMU not initialized or initial yaw not set. Cannot get relative yaw.");
        return NAN;
    }

    // Get the current absolute yaw
    sensors_event_t _orientationData;
    _bno.getEvent(&_orientationData, Adafruit_BNO055::VECTOR_EULER);
    double currentAbsoluteYaw = static_cast<double>(_orientationData.orientation.x);

    // Normalize the relative yaw to the range (-180, 180]
    double relativeYaw = currentAbsoluteYaw - _initialYaw;
    while (relativeYaw <= -180.0)
    {
        relativeYaw += 360.0;
    }
    while (relativeYaw > 180.0)
    {
        relativeYaw -= 360.0;
    }

    return relativeYaw;
}

double IMU::getPitch() const
{
    if (!_isInitialized)
    {
        Serial.println("IMU not initialized. Cannot get pitch.");
        return NAN;
    }

    // BNO055 Euler angles: z = pitch
    sensors_event_t _orientationData;
    _bno.getEvent(&_orientationData, Adafruit_BNO055::VECTOR_EULER);
    return static_cast<double>(_orientationData.orientation.z);
}

double IMU::getRoll() const
{
    if (!_isInitialized)
    {
        Serial.println("IMU not initialized. Cannot get roll.");
        return NAN;
    }

    // BNO055 Euler angles: y = roll
    sensors_event_t _orientationData;
    _bno.getEvent(&_orientationData, Adafruit_BNO055::VECTOR_EULER);
    return static_cast<double>(_orientationData.orientation.y);
}

bool IMU::resetYaw()
{
    if (!_isInitialized)
    {
        Serial.println("IMU not initialized. Cannot reset yaw.");
        return false;
    }

    double currentAbsYaw = getAbsoluteYaw();
    if (!isnan(currentAbsYaw))
    {
        _initialYaw = currentAbsYaw;
        return true;
    }

    // Failed to update or get absolute yaw
    return false;
}

// UltrasonicSensor Implementation

UltrasonicSensor::UltrasonicSensor(uint8_t trigEchoPin)
    : _trigEchoPin(trigEchoPin)
{
}

void UltrasonicSensor::begin()
{
    // Pin mode is set dynamically in readDistanceCm,
    // but setting it low initially is good practice.
    pinMode(_trigEchoPin, OUTPUT);
    digitalWrite(_trigEchoPin, LOW);
}

float UltrasonicSensor::readDistanceCm()
{
    // Send trigger pulse
    pinMode(_trigEchoPin, OUTPUT);

    digitalWrite(_trigEchoPin, LOW);
    delayMicroseconds(5); // Ensure clean low state

    digitalWrite(_trigEchoPin, HIGH);
    delayMicroseconds(5); // Trigger pulse width
    digitalWrite(_trigEchoPin, LOW);

    // Switch pin to input to read the echo
    pinMode(_trigEchoPin, INPUT);

    // Measure the duration of the echo pulse
    // pulseIn returns duration in microseconds, or 0 on timeout
    unsigned long duration_us = pulseIn(_trigEchoPin, HIGH, PULSE_TIMEOUT_US);

    // Calculate distance
    // Distance (cm) = (Duration (us) * Speed of Sound (cm/us)) / 2
    // The division by 2 accounts for the pulse traveling to the object and back.
    if (duration_us == 0)
    {
        // Timeout or no echo detected
        return NAN; // Indicate error/timeout
    }
    else
    {
        float distance = (static_cast<float>(duration_us) * SPEED_OF_SOUND_CM_US) / 2.0f;
        return distance;
    }
}

// Pixy2Handler Implementation

Pixy2Handler::Pixy2Handler()
    : _isInitialized(false),
      _largestBlockIndex(-1)
{
}

int8_t Pixy2Handler::begin()
{
    int8_t result = _pixy.init();

    if (result == 0)
    {
        _isInitialized = true;

        // Optional: We can set camera parameters if needed (e.g., brightness, mode)
        // _pixy.setLamp(1, 1); // Turn on LEDs
        // _pixy.setServos(pan, tilt); // If using servos
    }
    else
    {
        _isInitialized = false;
        Serial.print("Pixy2 init failed with error: ");

        /*
        PIXY_RESULT_OK                       0
        PIXY_RESULT_ERROR                    -1
        PIXY_RESULT_BUSY                     -2
        PIXY_RESULT_CHECKSUM_ERROR           -3
        PIXY_RESULT_TIMEOUT                  -4
        PIXY_RESULT_BUTTON_OVERRIDE          -5
        PIXY_RESULT_PROG_CHANGING            -6
        */

        switch (result)
        {
        case PIXY_RESULT_ERROR:
            Serial.println("Error");
            break;
        case PIXY_RESULT_BUSY:
            Serial.println("Busy");
            break;
        case PIXY_RESULT_CHECKSUM_ERROR:
            Serial.println("Checksum Error");
            break;
        case PIXY_RESULT_TIMEOUT:
            Serial.println("Timeout");
            break;
        case PIXY_RESULT_BUTTON_OVERRIDE:
            Serial.println("Button Override");
            break;
        case PIXY_RESULT_PROG_CHANGING:
            Serial.println("Program Changing");
            break;
        default:
            Serial.println("Unknown Error");
            break;
        }
    }
    return result;
}

int8_t Pixy2Handler::updateBlocks()
{
    if (!_isInitialized)
    {
        Serial.println("Pixy2 not initialized. Cannot update blocks.");
        return -1;
    }

    // Get block data from Pixy2
    int8_t numBlocks = _pixy.ccc.getBlocks();

    // Reset largest block tracking for this frame
    _largestBlockIndex = -1;
    double _largestBlockArea = 0.0; // Reset area for comparison

    if (numBlocks > 0)
    {
        // Find the largest block among the returned blocks
        for (int i = 0; i < numBlocks; ++i)
        {
            uint32_t area = (uint32_t)_pixy.ccc.blocks[i].m_width * (uint32_t)_pixy.ccc.blocks[i].m_height;
            if (area > _largestBlockArea)
            {
                _largestBlockIndex = i; // Store the index within pixy.ccc.blocks
                _largestBlock.signature = _pixy.ccc.blocks[i].m_signature;
                _largestBlock.x = _pixy.ccc.blocks[i].m_x;
                _largestBlock.y = _pixy.ccc.blocks[i].m_y;
                _largestBlock.width = _pixy.ccc.blocks[i].m_width;
                _largestBlock.height = _pixy.ccc.blocks[i].m_height;
                _largestBlock.age = _pixy.ccc.blocks[i].m_age;
                _largestBlock.area = area; // Store the area for comparison

                // Update the largest block area
                _largestBlockArea = area;
            }
        }
    }

    return numBlocks; // Return the number of blocks found (or error code)
}

uint8_t Pixy2Handler::getNumBlocks() const
{
    if (!_isInitialized || _pixy.ccc.numBlocks < 0)
        return 0;
    return _pixy.ccc.numBlocks;
}

bool Pixy2Handler::getBlock(PixyBlock &block, uint8_t index = -1) const
{

    if (!_isInitialized || _pixy.ccc.numBlocks <= 0)
    {
        return false; // Invalid index or no blocks available
    }

    if (index == -1)
    {
        // If no index is provided, use the largest block index
        index = _largestBlockIndex;
    }

    // Copy data from the Pixy2 largest block
    block = _largestBlock;
    return true;
}

// ZigbeeAruco Implementation

ZigbeeAruco::ZigbeeAruco(Stream &communicationStream)
    : _stream(communicationStream),
      _bufferIndex(0)
{
    _currentPose.valid = false;                        // Initially invalid
    memset(_receiveBuffer, 0, sizeof(_receiveBuffer)); // Clear buffer
}

void ZigbeeAruco::begin()
{
    // We assume the user configures the stream (e.g., Serial1.begin()) outside.

    // We just clear the buffer here.
    while (_stream.available())
    {
        // Clear any old data
        _stream.read();
    }

    _bufferIndex = 0; // Reset buffer index
    _currentPose.valid = false;
}

void ZigbeeAruco::requestPose()
{
    _stream.write(REQUEST_CHAR);
}

bool ZigbeeAruco::updatePoseData()
{
    // Request pose data from the Zigbee device
    requestPose();

    bool newMessageReceived = false;
    while (_stream.available() > 0)
    {
        if (_bufferIndex < RESPONSE_LENGTH)
        {
            char c = _stream.read();
            _receiveBuffer[_bufferIndex++] = c;

            // Check if we have received the expected number of characters
            if (_bufferIndex == RESPONSE_LENGTH)
            {
                _receiveBuffer[_bufferIndex] = '\0'; // Null-terminate the string
                parseResponse();                     // Attempt to parse the complete message
                newMessageReceived = true;
                _bufferIndex = 0; // Reset buffer for the next message
                break;
            }
        }
        else
        {
            // Buffer overflow, discard character and reset buffer
            _stream.read();                                                 // Consume the character
            _bufferIndex = 0;                                               // Reset - potentially lost a message
            Serial.println("Zigbee Buffer Overflow: Discarding character"); // Log error
            _currentPose.valid = false;                                     // Mark data as invalid due to error
        }
    }
    return newMessageReceived; // Indicates if a *potential* message was fully received
}

void ZigbeeAruco::parseResponse()
{
    // Example parsing for "M,TTTT,XXX,YYY" (14 chars)
    // Indices: M=0, ,=1, T=2-5, ,=6, X=7-9, ,=10, Y=11-13

    // Check for delimiters roughly in expected places
    if (_receiveBuffer[1] != DELIMITER_CHAR || _receiveBuffer[6] != DELIMITER_CHAR || _receiveBuffer[10] != DELIMITER_CHAR)
    {
        // Malformed message format
        _currentPose.valid = false;
        Serial.println("Zigbee Parse Error: Delimiters not found");
        return;
    }

    // Extract parts
    char matchChar = _receiveBuffer[0];

    // Create temporary null-terminated strings for conversion
    char timeStr[5]; // TTTT + null
    strncpy(timeStr, _receiveBuffer + 2, 4);
    timeStr[4] = '\0';

    char xposStr[4]; // XXX + null
    strncpy(xposStr, _receiveBuffer + 7, 3);
    xposStr[3] = '\0';

    char yposStr[4]; // YYY + null
    strncpy(yposStr, _receiveBuffer + 11, 3);
    yposStr[3] = '\0';

    /*
    // Check for valid characters in the strings
    // Example parsing for "M,TTTT,XXX,YYY" (14 chars)
    // M,TTTT,---,--- If your bot’s information is not sent across by the Tx (camera has not detected it).
    // ?,????,---,--- If the XBee module is not receiving any information from the Tx.
    // /,////,---,--- If the checksums are not matching.
    */

    // Check for valid characters in the strings
    if (strcmp(xposStr, "---") == 0 || strcmp(yposStr, "---") == 0)
    {
        // Invalid data received
        _currentPose.valid = false;
        Serial.println("Zigbee Parse Error: Aruco not detected");
        return;
    }

    // Check for valid match character
    if (matchChar != '1' && matchChar != '0')
    {
        // Invalid match character
        _currentPose.valid = false;
        Serial.println("Zigbee Parse Error: Invalid match character");
        return;
    }

    // Convert to numbers: Use atof for float/double
    double timestamp = atof(timeStr);
    double xpos = atof(xposStr);
    double ypos = atof(yposStr);
    bool matchValid = (matchChar == '1');

    // Update internal state if match is valid
    _currentPose.valid = matchValid; // Set validity based on the 'M' character
    _currentPose.timestamp = timestamp;
    _currentPose.x = xpos;
    _currentPose.y = ypos;
}

PoseData ZigbeeAruco::getPose() const
{
    return _currentPose;
}

bool ZigbeeAruco::hasValidPose() const
{
    return _currentPose.valid;
}

double ZigbeeAruco::getX() const
{
    return _currentPose.valid ? _currentPose.x : 0.0; // Return 0 if not valid
}

double ZigbeeAruco::getY() const
{
    return _currentPose.valid ? _currentPose.y : 0.0; // Return 0 if not valid
}

double ZigbeeAruco::getTimestamp() const
{
    return _currentPose.valid ? _currentPose.timestamp : 0.0; // Return 0 if not valid
}
