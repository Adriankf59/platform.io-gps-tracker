// GpsManager.h - Enhanced for Real-time Tracking

#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// GPS configuration commands for higher update rates
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_BAUDRATE_115200 "$PMTK251,115200*1F"

class GpsManager {
private:
  TinyGPSPlus& gps;
  HardwareSerial& serialGPS;
  unsigned long lastBufferClearTime;
  unsigned long lastFixTime;
  unsigned long fixLostTime;
  bool lastFixValid;
  bool newFixFlag;
  
  // Enhanced tracking data
  float lastLatitude;
  float lastLongitude;
  unsigned long lastPositionTime;
  float distanceFromLastPosition;
  
  // Position filtering
  float positionFilter[3][2]; // 3 samples of lat/lon
  int filterIndex;
  bool filterReady;
  
  // HDOP tracking
  float currentHDOP;
  
  void sendCommand(const char* cmd);
  void configureHighRate();
  void applyPositionFilter();
  
public:
  GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial);
  
  void begin();
  void update();
  void clearBuffer();
  void getTimestamp(char* timestampStr, size_t maxLen);
  void enableHighUpdateRate(int hz = 10);
  
  // Status getters
  bool isValid() const { return gps.location.isValid() && currentHDOP < 2.0; } // Better validation
  bool hasNewFix() { 
    bool result = newFixFlag;
    newFixFlag = false;
    return result;
  }
  
  // Enhanced position getters with filtering
  float getLatitude() const;
  float getLongitude() const;
  float getRawLatitude() const { return gps.location.lat(); }
  float getRawLongitude() const { return gps.location.lng(); }
  
  // Motion getters
  float getSpeed() const { return gps.speed.kmph(); }
  float getHeading() const { return gps.course.isValid() ? gps.course.deg() : 0.0; }
  float getAltitude() const { return gps.altitude.isValid() ? gps.altitude.meters() : 0.0; }
  bool isSpeedValid() const { return gps.speed.isValid(); }
  bool isHeadingValid() const { return gps.course.isValid(); }
  
  // Quality indicators
  int getSatellites() const;
  float getHDOP() const { return currentHDOP; }
  bool isHighAccuracy() const { return currentHDOP < 1.0 && getSatellites() >= 8; }
  
  // Movement detection
  float getDistanceFromLastPosition() const { return distanceFromLastPosition; }
  unsigned long getTimeSinceLastMovement() const;
  
  // GPS time functions
  bool hasValidTime() const { return gps.date.isValid() && gps.time.isValid(); }
  int getGpsHour() const { return gps.time.hour(); }
  int getGpsMinute() const { return gps.time.minute(); }
  int getGpsSecond() const { return gps.time.second(); }
  
  // For unit testing
  friend class GpsManagerTest;
};

#endif // GPS_MANAGER_H