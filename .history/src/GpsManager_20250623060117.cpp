// GpsManager.cpp - Enhanced for Real-time Tracking

#include "GpsManager.h"
#include <math.h>

GpsManager::GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial) 
  : gps(gpsInstance), 
    serialGPS(serial),
    lastBufferClearTime(0),
    lastFixTime(0),
    fixLostTime(0),
    lastFixValid(false),
    newFixFlag(false),
    lastLatitude(0),
    lastLongitude(0),
    lastPositionTime(0),
    distanceFromLastPosition(0),
    filterIndex(0),
    filterReady(false),
    currentHDOP(99.9) {
  
  // Initialize position filter
  memset(positionFilter, 0, sizeof(positionFilter));
}

void GpsManager::begin() {
  // Start with standard baud rate
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  LOG_INFO(MODULE_GPS, "GPS module initialized on pins RX:%d TX:%d", GPS_RX_PIN, GPS_TX_PIN);
  
  delay(1000);
  
  // Try to configure for higher update rate
  #ifdef GPS_UPDATE_RATE
    enableHighUpdateRate(GPS_UPDATE_RATE);
  #endif
}

void GpsManager::sendCommand(const char* cmd) {
  serialGPS.println(cmd);
  LOG_DEBUG(MODULE_GPS, "Sent GPS command: %s", cmd);
  delay(100);
}

void GpsManager::configureHighRate() {
  // Send configuration commands
  #ifdef GPS_BAUD_RATE
    if (GPS_BAUD_RATE == 115200) {
      // Try to set GPS module to 115200 baud
      sendCommand(PMTK_SET_NMEA_BAUDRATE_115200);
      delay(500);
      
      // Switch ESP32 UART to 115200
      serialGPS.end();
      delay(100);
      serialGPS.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
      LOG_INFO(MODULE_GPS, "GPS baud rate set to 115200");
      delay(500);
    }
  #endif
}

void GpsManager::enableHighUpdateRate(int hz) {
  const char* command = nullptr;
  
  switch(hz) {
    case 1:
      command = PMTK_SET_NMEA_UPDATE_1HZ;
      break;
    case 5:
      command = PMTK_SET_NMEA_UPDATE_5HZ;
      break;
    case 10:
      command = PMTK_SET_NMEA_UPDATE_10HZ;
      break;
    default:
      LOG_WARN(MODULE_GPS, "Unsupported GPS update rate: %d Hz", hz);
      return;
  }
  
  if (command) {
    sendCommand(command);
    LOG_INFO(MODULE_GPS, "GPS update rate set to %d Hz", hz);
  }
}

void GpsManager::update() {
  // Read all available GPS data
  while (serialGPS.available() > 0) {
    char c = serialGPS.read();
    gps.encode(c);
    
    // Check if we got a complete position update
    if (gps.location.isUpdated()) {
      applyPositionFilter();
    }
  }
  
  // Update HDOP if available
  if (gps.hdop.isValid()) {
    currentHDOP = gps.hdop.hdop();
  }
  
  // Track GPS fix changes
  bool currentFixValid = gps.location.isValid() && currentHDOP < 3.0; // Stricter validation
  unsigned long currentTime = millis();
  
  // Detect GPS fix acquisition
  if (currentFixValid && !lastFixValid) {
    LOG_INFO(MODULE_GPS, "GPS fix acquired! Lat: %.6f, Lon: %.6f, HDOP: %.1f", 
             getLatitude(), getLongitude(), currentHDOP);
    lastFixTime = currentTime;
    fixLostTime = 0;
    newFixFlag = true;
  }
  
  // Detect GPS fix loss
  if (!currentFixValid && lastFixValid) {
    LOG_WARN(MODULE_GPS, "GPS fix lost! HDOP: %.1f", currentHDOP);
    fixLostTime = currentTime;
  }
  
  // Calculate distance from last position
  if (currentFixValid && lastLatitude != 0 && lastLongitude != 0) {
    float currentLat = getLatitude();
    float currentLon = getLongitude();
    
    // Simple distance calculation for small distances
    float dLat = currentLat - lastLatitude;
    float dLon = currentLon - lastLongitude;
    distanceFromLastPosition = sqrt(dLat*dLat + dLon*dLon) * 111320; // Convert to meters
    
    // Update last position if moved significantly (> 1 meter)
    if (distanceFromLastPosition > 1.0) {
      lastLatitude = currentLat;
      lastLongitude = currentLon;
      lastPositionTime = currentTime;
    }
  } else if (currentFixValid) {
    // Initialize last position
    lastLatitude = getLatitude();
    lastLongitude = getLongitude();
    lastPositionTime = currentTime;
  }
  
  lastFixValid = currentFixValid;
  
  // Clear GPS buffer periodically (less frequently)
  if (currentTime - lastBufferClearTime >= GPS_BUFFER_CLEAR_INTERVAL) {
    clearBuffer();
    lastBufferClearTime = currentTime;
  }
}

void GpsManager::applyPositionFilter() {
  if (!gps.location.isValid()) return;
  
  // Add new position to filter
  positionFilter[filterIndex][0] = gps.location.lat();
  positionFilter[filterIndex][1] = gps.location.lng();
  filterIndex = (filterIndex + 1) % 3;
  
  // Mark filter as ready after filling all samples
  if (filterIndex == 0 && !filterReady) {
    filterReady = true;
  }
}

float GpsManager::getLatitude() const {
  if (!filterReady || !gps.location.isValid()) {
    return gps.location.lat();
  }
  
  // Return median filtered position
  float lats[3];
  for (int i = 0; i < 3; i++) {
    lats[i] = positionFilter[i][0];
  }
  
  // Simple bubble sort for 3 elements
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (lats[j] > lats[j + 1]) {
        float temp = lats[j];
        lats[j] = lats[j + 1];
        lats[j + 1] = temp;
      }
    }
  }
  
  return lats[1]; // Return median
}

float GpsManager::getLongitude() const {
  if (!filterReady || !gps.location.isValid()) {
    return gps.location.lng();
  }
  
  // Return median filtered position
  float lons[3];
  for (int i = 0; i < 3; i++) {
    lons[i] = positionFilter[i][1];
  }
  
  // Simple bubble sort for 3 elements
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (lons[j] > lons[j + 1]) {
        float temp = lons[j];
        lons[j] = lons[j + 1];
        lons[j + 1] = temp;
      }
    }
  }
  
  return lons[1]; // Return median
}

void GpsManager::clearBuffer() {
  int clearedBytes = Utils::clearSerialBuffer(serialGPS);
  
  if (clearedBytes > 100) { // Only log if significant data cleared
    LOG_DEBUG(MODULE_GPS, "Cleared GPS buffer: %d bytes", clearedBytes);
  }
}

void GpsManager::getTimestamp(char* timestampStr, size_t maxLen) {
  // Default timestamp in UTC
  strcpy(timestampStr, "2025-01-15T21:09:00Z");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    // Always use UTC (offset 0)
    Utils::formatISO8601(timestampStr, maxLen,
                        gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second(),
                        0); // UTC offset = 0
    
    LOG_TRACE(MODULE_GPS, "GPS Time (UTC): %s", timestampStr);
  } else {
    LOG_TRACE(MODULE_GPS, "GPS time invalid, using default");
  }
}

int GpsManager::getSatellites() const {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

unsigned long GpsManager::getTimeSinceLastMovement() const {
  if (lastPositionTime == 0) return 0;
  return millis() - lastPositionTime;
}