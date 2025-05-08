// SensorSuite/SensorSuite.h

#pragma once

// Include common Arduino types and functions
#include <Arduino.h>

// IMU Class
#include <Wire.h> // Requires I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Pixy2 Class
#include <Pixy2.h>

// Zigbee/Aruco Positioning Class
#include <Stream.h> // For Serial/HardwareSerial communication

// Forward declarations for sensor classes
class IMU;
class UltrasonicSensor;
class Pixy2Handler;
class ZigbeeAruco;

// IMU Class
/**
 * @brief Interface for the Adafruit BNO055 9-DOF IMU.
 *
 * Provides methods to read orientation data (yaw, pitch, roll) and reset the yaw origin.
 * Assumes I2C communication.
 */
class IMU
{
public:
    /**
     * @brief Constructor for the IMU class.
     */
    IMU();

    /**
     * @brief Initializes the BNO055 sensor.
     * Call this in your setup() function.
     * @return True if initialization was successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Gets the absolute yaw (heading) in degrees.
     * This value is relative to the sensor's magnetic north or initial calibration.
     * Range: [0, 360) degrees.
     * @return Absolute yaw in degrees, or NAN if no data is available.
     */
    double getAbsoluteYaw() const;

    /**
     * @brief Gets the yaw (heading) relative to the initial yaw set at startup or last reset.
     * Range: (-180, 180] degrees. Counter-clockwise is positive.
     * @return Relative yaw in degrees, or NAN if no data is available.
     */
    double getRelativeYaw() const;

    /**
     * @brief Gets the pitch angle in degrees.
     * Range: Typically [-90, 90] degrees.
     * @return Pitch in degrees, or NAN if no data is available.
     */
    double getPitch() const;

    /**
     * @brief Gets the roll angle in degrees.
     * Range: Typically [-180, 180) degrees.
     * @return Roll in degrees, or NAN if no data is available.
     */
    double getRoll() const;

    /**
     * @brief Resets the relative yaw origin to the current absolute yaw.
     * @return True if reset was successful, false otherwise (e.g., sensor not read yet).
     */
    bool resetYaw();

private:
    Adafruit_BNO055 _bno;
    double _initialYaw = 0.0; // Stores the yaw at initialization or last reset
    bool _isInitialized = false;
};

// Ultrasonic Sensor Class
/**
 * @brief Interface for single-pin ultrasonic distance sensors PING))).
 *
 * Measures distance by sending a ping and listening for the echo on the same pin.
 */
class UltrasonicSensor
{
public:
    /**
     * @brief Default Arduino pin for the ultrasonic sensor trigger/echo.
     */
    static constexpr uint8_t DEFAULT_ULTRASONIC_PIN = 23;

    /**
     * @brief Speed of sound in cm per microsecond (at approx 20°C).
     * Used for distance calculation: distance = (duration * speedOfSound) / 2.
     */
    static constexpr float SPEED_OF_SOUND_CM_US = 0.0343;

    /**
     * @brief Timeout for pulseIn() in microseconds. Prevents blocking indefinitely.
     * Based on expected maximum range and sensor specs. 15ms ~ 2.5m range.
     */
    static constexpr unsigned long PULSE_TIMEOUT_US = 15000UL; // 15 milliseconds

    /**
     * @brief Constructor for the UltrasonicSensor class.
     * @param trigEchoPin The Arduino pin connected to the sensor's trigger/echo pin.
     */
    UltrasonicSensor(uint8_t trigEchoPin = DEFAULT_ULTRASONIC_PIN);

    /**
     * @brief Initializes the sensor pin. Call this in your setup() function.
     * Note: Pin mode is set dynamically during measurement.
     */
    void begin();

    /**
     * @brief Reads the distance from the ultrasonic sensor.
     * Sends a trigger pulse and measures the echo pulse duration.
     * @return Measured distance in centimeters (cm). Returns 0 or a large value on timeout/error.
     * Check sensor datasheet for valid range and error indications.
     */
    float readDistanceCm();

private:
    uint8_t _trigEchoPin;
};

// Pixy2 Camera Handler Class

// Define a structure to hold block information
struct PixyBlock
{
    uint16_t signature{-1}; // The signature number of the detected block.
    uint16_t x{-1};         // X coordinate of the center of the block in pixels (0-315).
    uint16_t y{-1};         // Y coordinate of the center of the block in pixels (0-207).
    uint16_t width{-1};     // Width of the block in pixels.
    uint16_t height{-1};    // Height of the block in pixels.
    uint8_t age{-1};        // The number of frames the object has been tracked.
    uint32_t area{0};       // Calculated area (width * height).
};

/**
 * @brief Handler for interacting with the Pixy2 camera.
 *
 * Initializes the camera and provides methods to retrieve information about detected blocks,
 * focusing on the largest detected block. Assumes CCC (Color Connected Components) mode.
 */
class Pixy2Handler
{
public:
    /**
     * @brief Horizontal resolution of the Pixy2 camera frame.
     */
    static constexpr int FRAME_WIDTH = 316;

    /**
     * @brief Vertical resolution of the Pixy2 camera frame.
     */
    static constexpr int FRAME_HEIGHT = 208;

    /**
     * @brief Constructor for the Pixy2Handler class.
     */
    Pixy2Handler();

    /**
     * @brief Initializes the Pixy2 camera.
     * Assumes default communication interface (e.g., I2C or SPI selected via PixyMon).
     * @return 0 if initialization was successful, negative error code otherwise.
     */
    int8_t begin();

    /**
     * @brief Updates the block data from the Pixy2 camera.
     * This method should be called periodically in the main loop to get fresh data.
     * @param sigmap Optional bitmask of signatures to look for. 0 means all signatures.
     * @return The number of blocks detected in the latest frame, or a negative error code.
     */
    int8_t updateBlocks();

    /**
     * @brief Gets the number of blocks detected in the last call to updateBlocks().
     * @return The number of detected blocks.
     */
    uint8_t getNumBlocks() const;

    /**
     * @brief Gets information about a specific block by its index.
     * Requires updateBlocks() to have been called.
     * @param index The index of the block (0 to getNumBlocks() - 1).
     * @param block Output parameter where the block data will be stored.
     * @return True if the index is valid and data was retrieved, false otherwise.
     */
    bool getBlock(PixyBlock &block, uint8_t index = -1) const;

private:
    Pixy2 _pixy;                    // The Pixy2 object
    bool _isInitialized = false;    // True if the Pixy2 is initialized
    int8_t _largestBlockIndex = -1; // Index within the Pixy2's internal array for the largest block
    PixyBlock _largestBlock;        // The largest block data
};

// Define a structure for Pose data
struct PoseData
{
    bool valid = false;     // True if the data in this struct is valid from the last request.
    double timestamp = 0.0; // Timestamp from the positioning system (if provided).
    double x = 0.0;         // X position (cm).
    double y = 0.0;         // Y position (cm).
};

/**
 * @brief Handles communication with a Zigbee/Serial device providing Aruco marker pose.
 *
 * Assumes a simple request/response protocol over a Stream interface (e.g., Serial1).
 * Protocol:
 * - Request: Send '?' character.
 * - Response: A fixed-length string (e.g., 14 bytes) like "M,TTTT,XXX,YYY"
 * - M: '1' if pose is valid/matched, '0' otherwise.
 * - TTTT: Timestamp (e.g., milliseconds as string).
 * - XXX: X position (e.g., as string).
 * - YYY: Y position (e.g., as string).
 * - ',' are delimiters.
 */
class ZigbeeAruco
{
public:
    /**
     * @brief Expected length of the response message from the Zigbee device.
     * Example: "1,1234,100,200" = 14 characters
     */
    static constexpr size_t RESPONSE_LENGTH = 14;

    /**
     * @brief Timeout in milliseconds for waiting for a response.
     */
    static constexpr unsigned long RESPONSE_TIMEOUT_MS = 5000;

    /**
     * @brief Character to send to request a pose update.
     */
    static constexpr char REQUEST_CHAR = '?';

    /**
     * @brief Delimiter character in the response message.
     * This is used to separate the fields in the response string.
     */
    static constexpr char DELIMITER_CHAR = ',';

    /**
     * @brief Constructor for the ZigbeeAruco class.
     * @param communicationStream Reference to the Stream object (e.g., Serial1) for communication.
     */
    ZigbeeAruco(Stream &communicationStream);

    /**
     * @brief Initializes the communication stream and clears the receive buffer.
     * Call this in setup() if needed.
     */
    void begin();

    /**
     * @brief Sends a request for pose data to the Zigbee device.
     * This function is non-blocking; it just sends the request character.
     */
    void requestPose();

    /**
     * @brief Checks for and parses an incoming pose response from the Zigbee device.
     * This function should be called periodically in the loop after requestPose().
     * It reads available data, attempts to parse it if a full message is received,
     * and updates the internal pose data.
     * @return True if a new, complete, and validly formatted message was received and parsed, false otherwise.
     */
    bool updatePoseData();

    /**
     * @brief Parses the received message buffer.
     * Assumes _receiveBuffer contains a null-terminated string of RESPONSE_LENGTH.
     * Updates _currentPose based on parsing result.
     */
    void parseResponse();

    /**
     * @brief Gets the most recently received pose data.
     * Check the 'valid' field of the returned struct to see if the data is current and trustworthy.
     * @return A PoseData struct containing the latest information.
     */
    PoseData getPose() const;

    /**
     * @brief Checks if the last received pose data was marked as valid by the external system.
     * @return True if the last parsed message indicated a valid match ('1'), false otherwise ('0' or no data).
     */
    bool hasValidPose() const;

    /**
     * @brief Gets the X position from the last valid pose update.
     * @return X position, or 0.0 if no valid data.
     */
    double getX() const;

    /**
     * @brief Gets the Y position from the last valid pose update.
     * @return Y position, or 0.0 if no valid data.
     */
    double getY() const;

    /**
     * @brief Gets the timestamp from the last valid pose update.
     * @return Timestamp, or 0.0 if no valid data.
     */
    double getTimestamp() const;

private:
    Stream &_stream;
    PoseData _currentPose;
    char _receiveBuffer[RESPONSE_LENGTH + 1]; // Buffer to hold incoming message + null terminator
    size_t _bufferIndex = 0;                  // Current position in the buffer
};