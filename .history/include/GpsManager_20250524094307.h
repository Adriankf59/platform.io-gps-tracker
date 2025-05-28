// GpsManager.h
#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

class GpsManager {
private:
  TinyGPSPlus& gps;
  HardwareSerial& serialGPS;
  unsigned long lastBufferClearTime;
  unsigned long lastFixTime;
  unsigned long fixLostTime;
  bool lastFixValid;
  
public:
  GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial);
  
  void begin();
  void update();
  void clearBuffer();
  void getTimestamp(char* timestampStr, size_t maxLen);
  
  // Status getters
  bool isValid() const { return gps.location.isValid(); }
  bool hasNewFix() const { return isValid() && !lastFixValid; }
  float getLatitude() const { return gps.location.lat(); }
  float getLongitude() const { return gps.location.lng(); }
  int getSatellites() const;
  
  // For unit testing
  friend class GpsManagerTest;
};

#endif // GPS_MANAGER_H