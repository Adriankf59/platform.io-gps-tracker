// ImuManager.h - MPU6050 IMU Management for ESP32 GPS Tracker
#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// IMU States
enum ImuState {
  IMU_STATE_UNINITIALIZED,
  IMU_STATE_INITIALIZING,
  IMU_STATE_CALIBRATING,
  IMU_STATE_READY,
  IMU_STATE_ACTIVE,
  IMU_STATE_ERROR
};

// Position Source
enum PositionSource {
  POS_SOURCE_GPS,
  POS_SOURCE_IMU,
  POS_SOURCE_FUSION
};

// IMU Data Structure
struct ImuData {
  // Raw sensor data
  float accelX, accelY, accelZ;    // Accelerometer (g)
  float gyroX, gyroY, gyroZ;       // Gyroscope (deg/s)
  float temperature;               // Temperature (C)
  
  // Processed data
  float totalAccel;                // Total acceleration magnitude
  float tiltAngle;                 // Tilt angle from vertical
  float heading;                   // Current heading (degrees)
  float headingRate;               // Rate of heading change
  bool isMoving;                   // Movement detection
  bool isStationary;               // Stationary detection
  
  // Calibration offsets
  float accelXOffset, accelYOffset, accelZOffset;
  float gyroXOffset, gyroYOffset, gyroZOffset;
  
  // Timestamp
  unsigned long timestamp;
  
  void reset() {
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;
    temperature = 0;
    totalAccel = 0;
    tiltAngle = 0;
    heading = 0;
    headingRate = 0;
    isMoving = false;
    isStationary = true;
    timestamp = 0;
  }
};

// Dead Reckoning Data
struct DeadReckoningData {
  // Last known GPS position
  double lastGpsLat;
  double lastGpsLon;
  float lastGpsHeading;
  unsigned long lastGpsTime;
  
  // Current estimated position
  double estimatedLat;
  double estimatedLon;
  float estimatedHeading;
  float estimatedSpeed;
  float distanceTraveled;
  
  // Error estimation
  float positionError;        // Estimated error in meters
  float confidenceLevel;      // 0.0 to 1.0
  
  // State
  bool active;
  unsigned long startTime;
  unsigned long lastUpdateTime;
  
  void reset() {
    lastGpsLat = lastGpsLon = 0;
    lastGpsHeading = 0;
    lastGpsTime = 0;
    estimatedLat = estimatedLon = 0;
    estimatedHeading = estimatedSpeed = 0;
    distanceTraveled = 0;
    positionError = 0;
    confidenceLevel = 1.0;
    active = false;
    startTime = lastUpdateTime = 0;
  }
};

// Basement Detection Data
struct BasementDetectionData {
  bool inBasement;
  bool possibleEntry;
  bool possibleExit;
  unsigned long entryTime;
  unsigned long exitTime;
  float entryZAccel;
  float exitZAccel;
  int gpsLossCount;
  
  void reset() {
    inBasement = false;
    possibleEntry = false;
    possibleExit = false;
    entryTime = exitTime = 0;
    entryZAccel = exitZAccel = 0;
    gpsLossCount = 0;
  }
};

class ImuManager {
private:
  MPU6050 mpu;
  ImuState currentState;
  ImuData currentData;
  DeadReckoningData deadReckoning;
  BasementDetectionData basementDetection;
  
  // Configuration
  bool isEnabled;
  uint8_t i2cAddress;
  int sampleRate;
  
  // Calibration
  bool isCalibrated;
  int calibrationSamples;
  
  // Movement detection
  float movementThreshold;
  float stationaryThreshold;
  unsigned long lastMovementTime;
  
  // Data buffer for averaging
  static const int BUFFER_SIZE = 10;
  float accelBuffer[BUFFER_SIZE];
  int bufferIndex;
  
  // Performance tracking
  unsigned long lastSampleTime;
  unsigned long totalSamples;
  unsigned long errorCount;
  
  // Helper functions
  bool initializeSensor();
  void performCalibration();
  void updateMovementDetection();
  void updateDeadReckoning(float deltaTime);
  void updateBasementDetection();
  float calculateTotalAccel();
  float calculateTiltAngle();
  void applyCalibrationOffsets();
  double calculateDistance(float accel, float deltaTime);
  void updatePositionEstimate(float distance, float heading);
  
public:
  ImuManager();
  
  // Initialization
  bool begin();
  void end();
  bool isReady() const { return currentState == IMU_STATE_READY || currentState == IMU_STATE_ACTIVE; }
  
  // Configuration
  void enable() { isEnabled = true; }
  void disable() { isEnabled = false; }
  bool isEnabledStatus() const { return isEnabled; }
  void setSampleRate(int rate) { sampleRate = rate; }
  
  // Calibration
  bool calibrate();
  bool isCalibrationComplete() const { return isCalibrated; }
  void resetCalibration();
  
  // Data acquisition
  bool update();
  bool readSensor();
  ImuData getData() const { return currentData; }
  
  // Dead reckoning
  void startDeadReckoning(double lat, double lon, float heading);
  void stopDeadReckoning();
  bool isDeadReckoningActive() const { return deadReckoning.active; }
  DeadReckoningData getDeadReckoningData() const { return deadReckoning; }
  void updateGpsPosition(double lat, double lon, float heading);
  
  // Basement detection
  bool isInBasement() const { return basementDetection.inBasement; }
  BasementDetectionData getBasementStatus() const { return basementDetection; }
  void notifyGpsLoss();
  void notifyGpsRecovered();
  
  // Movement detection
  bool isMoving() const { return currentData.isMoving; }
  bool isStationary() const { return currentData.isStationary; }
  float getCurrentHeading() const { return currentData.heading; }
  float getEstimatedSpeed() const { return deadReckoning.estimatedSpeed; }
  
  // Position estimation
  double getEstimatedLatitude() const { return deadReckoning.estimatedLat; }
  double getEstimatedLongitude() const { return deadReckoning.estimatedLon; }
  float getPositionError() const { return deadReckoning.positionError; }
  float getConfidenceLevel() const { return deadReckoning.confidenceLevel; }
  PositionSource getPositionSource() const;
  
  // Diagnostics
  ImuState getState() const { return currentState; }
  const char* getStateString() const;
  void printStatus();
  void printSensorData();
  void printDeadReckoningInfo();
  String getJsonData();
  String getDiagnosticInfo();
  
  // Statistics
  unsigned long getTotalSamples() const { return totalSamples; }
  unsigned long getErrorCount() const { return errorCount; }
  float getErrorRate() const { return totalSamples > 0 ? (errorCount * 100.0 / totalSamples) : 0; }
};

// Utility functions
namespace ImuUtils {
  // Convert between coordinate systems
  double metersToLatitude(double meters, double currentLat);
  double metersToLongitude(double meters, double currentLat);
  double calculateBearing(double lat1, double lon1, double lat2, double lon2);
  double calculateDistance(double lat1, double lon1, double lat2, double lon2);
  
  // Sensor data validation
  bool isValidAcceleration(float x, float y, float z);
  bool isValidGyroscope(float x, float y, float z);
  
  // Movement analysis
  String getMovementDescription(float totalAccel);
  bool detectSuddenStop(float currentAccel, float previousAccel);
  bool detectTurn(float gyroZ, float threshold = 15.0);
}

#endif // IMU_MANAGER_H