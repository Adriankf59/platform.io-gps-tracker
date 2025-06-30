// RealtimeGpsManager.h - Enhanced GPS Manager for Smooth Tracking
#ifndef REALTIME_GPS_MANAGER_H
#define REALTIME_GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// Struktur untuk menyimpan posisi GPS dengan timestamp
struct GpsPosition {
  double latitude;
  double longitude;
  float speed;
  float course;
  unsigned long timestamp;
  bool valid;
};

// Struktur untuk interpolasi
struct InterpolatedPosition {
  double latitude;
  double longitude;
  float speed;
  float course;
  float accuracy;
};

class RealtimeGpsManager {
private:
  TinyGPSPlus& gps;
  HardwareSerial& serialGPS;
  
  // Position history untuk smoothing
  static const int POSITION_BUFFER_SIZE = 10;
  GpsPosition positionBuffer[POSITION_BUFFER_SIZE];
  int bufferIndex = 0;
  
  // Kalman filter state
  struct KalmanState {
    double lat;
    double lon;
    double latVelocity;
    double lonVelocity;
    double latAccel;
    double lonAccel;
    unsigned long lastUpdate;
    bool initialized;
  } kalman;
  
  // Prediction parameters
  float minSpeedThreshold = 0.5; // km/h - below this, consider stationary
  float maxAcceleration = 4.0;   // m/s² - max realistic acceleration for vehicle
  
  // GPS quality metrics
  float hdop = 0.0;
  int fixQuality = 0;
  
  // Timing
  unsigned long lastGpsRead = 0;
  unsigned long lastValidFix = 0;
  unsigned long lastPositionSent = 0;
  
  // Internal methods
  void updateKalmanFilter(double lat, double lon, unsigned long currentTime);
  InterpolatedPosition predictPosition(unsigned long targetTime);
  double calculateDistance(double lat1, double lon1, double lat2, double lon2);
  float calculateBearing(double lat1, double lon1, double lat2, double lon2);
  void addToBuffer(const GpsPosition& pos);
  GpsPosition getSmoothedPosition();
  bool isPositionRealistic(double lat, double lon, float speed);
  
public:
  RealtimeGpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial);
  
  void begin();
  void update();
  
  // Get current position with prediction
  InterpolatedPosition getCurrentPosition();
  
  // Get predicted position at future time
  InterpolatedPosition getPredictedPosition(unsigned long futureTimeMs);
  
  // Raw GPS data access
  bool hasValidFix() const { return gps.location.isValid() && gps.location.age() < 2000; }
  double getRawLatitude() const { return gps.location.lat(); }
  double getRawLongitude() const { return gps.location.lng(); }
  float getRawSpeed() const { return gps.speed.kmph(); }
  float getCourse() const { return gps.course.deg(); }
  int getSatellites() const { return gps.satellites.value(); }
  float getHDOP() const { return hdop; }
  
  // Position quality
  float getPositionAccuracy(); // in meters
  bool isMoving() const { return getRawSpeed() > minSpeedThreshold; }
  
  // Timing info
  unsigned long getTimeSinceLastFix() const { return millis() - lastValidFix; }
  void getTimestamp(char* timestampStr, size_t maxLen);
  
  // Configuration
  void setMinSpeedThreshold(float threshold) { minSpeedThreshold = threshold; }
  void setMaxAcceleration(float accel) { maxAcceleration = accel; }
};

#endif // REALTIME_GPS_MANAGER_H