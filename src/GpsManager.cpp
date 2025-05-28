// GpsManager.cpp
#include "GpsManager.h"

GpsManager::GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial) 
  : gps(gpsInstance), 
    serialGPS(serial),
    lastBufferClearTime(0),
    lastFixTime(0),
    fixLostTime(0),
    lastFixValid(false),
    newFixFlag(false) {
}

void GpsManager::begin() {
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  LOG_INFO(MODULE_GPS, "GPS module initialized on pins RX:%d TX:%d", GPS_RX_PIN, GPS_TX_PIN);
}

void GpsManager::update() {
  // Read GPS data
  while (serialGPS.available() > 0) {
    gps.encode(serialGPS.read());
  }
  
  // Track GPS fix changes
  bool currentFixValid = gps.location.isValid();
  unsigned long currentTime = millis();
  
  // Detect GPS fix acquisition
  if (currentFixValid && !lastFixValid) {
    LOG_INFO(MODULE_GPS, "GPS fix acquired! Lat: %.6f, Lon: %.6f", 
             getLatitude(), getLongitude());
    lastFixTime = currentTime;
    fixLostTime = 0;
    newFixFlag = true; // Set flag for new fix
  }
  
  // Detect GPS fix loss
  if (!currentFixValid && lastFixValid) {
    LOG_WARN(MODULE_GPS, "GPS fix lost!");
    fixLostTime = currentTime;
  }
  
  lastFixValid = currentFixValid;
  
  // Clear GPS buffer periodically
  if (currentTime - lastBufferClearTime >= GPS_BUFFER_CLEAR_INTERVAL) {
    clearBuffer();
    lastBufferClearTime = currentTime;
  }
}

void GpsManager::clearBuffer() {
  int clearedBytes = Utils::clearSerialBuffer(serialGPS);
  
  if (clearedBytes > 0) {
    LOG_DEBUG(MODULE_GPS, "Cleared GPS buffer: %d bytes", clearedBytes);
  }
}

void GpsManager::getTimestamp(char* timestampStr, size_t maxLen) {
  // Default timestamp if GPS is not valid
  strcpy(timestampStr, "2025-01-15T21:09:00+07:00");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    Utils::formatISO8601(timestampStr, maxLen,
                        gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second(),
                        UTC_OFFSET);
    
    LOG_DEBUG(MODULE_GPS, "GPS Time (WIB): %s", timestampStr);
  } else {
    LOG_DEBUG(MODULE_GPS, "GPS time data invalid, using default");
  }
}

int GpsManager::getSatellites() const {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}