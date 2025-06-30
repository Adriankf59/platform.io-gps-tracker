// GpsManager.cpp - Implementasi Manajer GPS (Enhanced Version)
#include "GpsManager.h"
#include <math.h>

// Konstruktor
GpsManager::GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial) 
  : gps(gpsInstance), 
    serialGPS(serial),
    lastBufferClearTime(0),
    lastFixTime(0),
    fixLostTime(0),
    lastFixValid(false),
    newFixFlag(false),
    firstFixAcquired(false),
    lastLatitude(0),
    lastLongitude(0),
    lastPositionTime(0),
    distanceFromLastPosition(0),
    filterIndex(0),
    filterReady(false),
    currentHDOP(99.9),
    currentFixStatus(GPS_FIX_NONE),
    moduleStartTime(0),
    firstFixTime(0),
    totalSentences(0),
    validSentences(0),
    checksumErrors(0),
    highUpdateRateEnabled(false),
    currentUpdateRate(1),
    sbasEnabled(false) {
  
  // Inisialisasi filter posisi
  memset(positionFilter, 0, sizeof(positionFilter));
  
  // Inisialisasi quality indicators
  quality.satellites = 0;
  quality.hdop = 99.9;
  quality.pdop = 99.9;
  quality.vdop = 99.9;
  quality.fixQuality = 0;
  quality.hasAltitude = false;
  quality.lastUpdateTime = 0;
}

// Inisialisasi modul GPS
void GpsManager::begin() {
  moduleStartTime = millis();
  
  // Start serial dengan buffer lebih besar untuk performa lebih baik
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  serialGPS.setRxBufferSize(2048); // Increase RX buffer
  serialGPS.setTxBufferSize(512);  // Increase TX buffer
  
  LOG_INFO(MODULE_GPS, "Modul GPS diinisialisasi pada pin RX:%d TX:%d", 
           GPS_RX_PIN, GPS_TX_PIN);
  
  delay(1000); // Give module time to start
  
  // Clear any garbage in buffer
  clearBuffer();
  
  // Send initialization sequence for optimal performance
  LOG_INFO(MODULE_GPS, "Mengirim konfigurasi awal GPS...");
  
  // 1. Set NMEA output to RMC and GGA only (faster parsing)
  sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(100);
  
  // 2. Set update rate to 1Hz initially (can be increased later)
  sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(100);
  
  // 3. Enable SBAS for better accuracy
  sendCommand(PMTK_ENABLE_SBAS);
  delay(100);
  sbasEnabled = true;
  
  // 4. Enable WAAS (for US) / EGNOS (for Europe) / MSAS (for Japan)
  sendCommand(PMTK_ENABLE_WAAS);
  delay(100);
  
  // 5. Request firmware version
  sendCommand("$PMTK605*31");
  delay(100);
  
  // 6. Try hot start first (fastest if ephemeris data is valid)
  sendCommand(PMTK_CMD_HOT_START);
  delay(100);
  
  LOG_INFO(MODULE_GPS, "GPS initialization commands sent");
  LOG_INFO(MODULE_GPS, "Waiting for GPS fix...");
}

// Kirim command ke modul GPS
void GpsManager::sendCommand(const char* cmd) {
  serialGPS.println(cmd);
  LOG_DEBUG(MODULE_GPS, "GPS CMD: %s", cmd);
  
  // Small delay to ensure command is processed
  delay(10);
}

// Update data GPS
void GpsManager::update() {
  bool dataUpdated = false;
  
  // Process all available GPS data
  while (serialGPS.available() > 0) {
    char c = serialGPS.read();
    
    if (gps.encode(c)) {
      dataUpdated = true;
      validSentences++;
    }
    
    totalSentences++;
    
    // Check for checksum errors
    if (gps.failedChecksum() > checksumErrors) {
      checksumErrors = gps.failedChecksum();
      LOG_DEBUG(MODULE_GPS, "GPS checksum error detected (total: %lu)", checksumErrors);
    }
  }
  
  // Update quality indicators if data was processed
  if (dataUpdated) {
    updateQualityIndicators();
    
    // Apply position filter if location updated
    if (gps.location.isUpdated()) {
      applyPositionFilter();
    }
  }
  
  // Track fix status changes
  bool currentFixValid = gps.location.isValid() && currentHDOP < GPS_MAX_HDOP;
  unsigned long currentTime = millis();
  
  // Detect GPS fix acquired
  if (currentFixValid && !lastFixValid) {
    LOG_INFO(MODULE_GPS, "✅ GPS FIX ACQUIRED!");
    LOG_INFO(MODULE_GPS, "📍 Position: %.6f, %.6f", getLatitude(), getLongitude());
    LOG_INFO(MODULE_GPS, "🛰️ Satellites: %d, HDOP: %.1f", getSatellites(), currentHDOP);
    LOG_INFO(MODULE_GPS, "📊 Fix Quality: %s", getFixStatusString());
    
    lastFixTime = currentTime;
    fixLostTime = 0;
    newFixFlag = true;
    
    // Record first fix time
    if (!firstFixAcquired) {
      firstFixAcquired = true;
      firstFixTime = currentTime;
      unsigned long timeToFix = (firstFixTime - moduleStartTime) / 1000;
      LOG_INFO(MODULE_GPS, "⏱️ Time to first fix: %lu seconds", timeToFix);
    }
  }
  
  // Detect GPS fix lost
  if (!currentFixValid && lastFixValid) {
    LOG_WARN(MODULE_GPS, "⚠️ GPS FIX LOST!");
    LOG_WARN(MODULE_GPS, "Last known position: %.6f, %.6f", lastLatitude, lastLongitude);
    LOG_WARN(MODULE_GPS, "Satellites: %d, HDOP: %.1f", getSatellites(), currentHDOP);
    fixLostTime = currentTime;
  }
  
  // Update movement tracking
  if (currentFixValid && lastLatitude != 0 && lastLongitude != 0) {
    float currentLat = getLatitude();
    float currentLon = getLongitude();
    
    // Calculate distance from last position
    distanceFromLastPosition = calculateDistance(
      lastLatitude, lastLongitude, currentLat, currentLon
    );
    
    // Update last position if moved significantly (> 5 meters)
    if (distanceFromLastPosition > 5.0) {
      lastLatitude = currentLat;
      lastLongitude = currentLon;
      lastPositionTime = currentTime;
      
      LOG_DEBUG(MODULE_GPS, "Position updated, moved %.1f meters", distanceFromLastPosition);
    }
  } else if (currentFixValid && lastLatitude == 0) {
    // Initialize last position
    lastLatitude = getLatitude();
    lastLongitude = getLongitude();
    lastPositionTime = currentTime;
  }
  
  lastFixValid = currentFixValid;
  
  // Clear buffer periodically to prevent overflow
  if (currentTime - lastBufferClearTime >= GPS_BUFFER_CLEAR_INTERVAL) {
    clearBuffer();
    lastBufferClearTime = currentTime;
  }
  
  // Log GPS health periodically (every 30 seconds)
  static unsigned long lastHealthLog = 0;
  if (currentTime - lastHealthLog > 30000 && currentFixValid) {
    LOG_DEBUG(MODULE_GPS, "GPS Health: Sats=%d, HDOP=%.1f, Success Rate=%.1f%%",
              getSatellites(), currentHDOP, getSentenceSuccessRate());
    lastHealthLog = currentTime;
  }
}

// Update quality indicators
void GpsManager::updateQualityIndicators() {
  // Update basic quality metrics
  if (gps.satellites.isValid()) {
    quality.satellites = gps.satellites.value();
  }
  
  if (gps.hdop.isValid()) {
    currentHDOP = gps.hdop.hdop();
    quality.hdop = currentHDOP;
  }
  
  // Determine fix type based on available data
  if (!gps.location.isValid()) {
    currentFixStatus = GPS_FIX_NONE;
  } else if (gps.altitude.isValid() && quality.satellites >= 4) {
    currentFixStatus = GPS_FIX_3D;
    quality.hasAltitude = true;
  } else if (quality.satellites >= 3) {
    currentFixStatus = GPS_FIX_2D;
    quality.hasAltitude = false;
  } else {
    currentFixStatus = GPS_FIX_NONE;
  }
  
  // Check for DGPS (if HDOP is very good and satellites > 8)
  if (currentFixStatus != GPS_FIX_NONE && quality.hdop < 1.0 && quality.satellites >= 8) {
    currentFixStatus = GPS_FIX_DGPS;
  }
  
  quality.lastUpdateTime = millis();
}

// Apply position filter
void GpsManager::applyPositionFilter() {
  if (!gps.location.isValid()) return;
  
  // Add new position to filter
  positionFilter[filterIndex][0] = gps.location.lat();
  positionFilter[filterIndex][1] = gps.location.lng();
  filterIndex = (filterIndex + 1) % 3;
  
  // Mark filter ready after buffer is full
  if (filterIndex == 0 && !filterReady) {
    filterReady = true;
    LOG_DEBUG(MODULE_GPS, "Position filter ready");
  }
}

// Calculate distance between two coordinates (Haversine formula)
float GpsManager::calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000; // Earth radius in meters
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c;
}

// Check if GPS fix is valid
bool GpsManager::isValid() const {
  return gps.location.isValid() && 
         currentHDOP < GPS_MAX_HDOP && 
         quality.satellites >= GPS_MIN_SATELLITES;
}

// Check and reset new fix flag
bool GpsManager::hasNewFix() {
  bool result = newFixFlag;
  newFixFlag = false;
  return result;
}

// Get filtered latitude
float GpsManager::getLatitude() const {
  if (!filterReady || !gps.location.isValid()) {
    return gps.location.lat();
  }
  
  // Return median of 3 samples
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

// Get filtered longitude
float GpsManager::getLongitude() const {
  if (!filterReady || !gps.location.isValid()) {
    return gps.location.lng();
  }
  
  // Return median of 3 samples
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

// Clear serial buffer
void GpsManager::clearBuffer() {
  int clearedBytes = Utils::clearSerialBuffer(serialGPS);
  
  if (clearedBytes > 100) { // Only log if significant data
    LOG_DEBUG(MODULE_GPS, "GPS buffer cleared: %d bytes", clearedBytes);
  }
}

// Get timestamp in ISO8601 format
void GpsManager::getTimestamp(char* timestampStr, size_t maxLen) {
  // Default timestamp
  strcpy(timestampStr, "2025-01-01T00:00:00Z");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    // Format timestamp in UTC
    Utils::formatISO8601(timestampStr, maxLen,
                        gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second(),
                        0); // UTC offset = 0
    
    LOG_TRACE(MODULE_GPS, "GPS Time (UTC): %s", timestampStr);
  } else {
    LOG_TRACE(MODULE_GPS, "GPS time not valid, using default");
  }
}

// Get GPS time as Unix timestamp
unsigned long GpsManager::getGpsTime() const {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return 0;
  }
  
  // Simple conversion (not accounting for leap seconds)
  // This is approximate but sufficient for most uses
  struct tm t;
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min = gps.time.minute();
  t.tm_sec = gps.time.second();
  
  return mktime(&t);
}

// Get satellites count
int GpsManager::getSatellites() const {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

// Get time since last movement
unsigned long GpsManager::getTimeSinceLastMovement() const {
  if (lastPositionTime == 0) return 0;
  return millis() - lastPositionTime;
}

// Get time to first fix
unsigned long GpsManager::getTimeToFirstFix() const {
  if (!firstFixAcquired) return 0;
  return (firstFixTime - moduleStartTime) / 1000; // Return in seconds
}

// Get time since last fix
unsigned long GpsManager::getTimeSinceLastFix() const {
  if (lastFixTime == 0) return 0;
  return (millis() - lastFixTime) / 1000; // Return in seconds
}

// Get fix status string
const char* GpsManager::getFixStatusString() const {
  switch (currentFixStatus) {
    case GPS_FIX_NONE: return "No Fix";
    case GPS_FIX_2D: return "2D Fix";
    case GPS_FIX_3D: return "3D Fix";
    case GPS_FIX_DGPS: return "DGPS Fix";
    default: return "Unknown";
  }
}

// Enable high update rate
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
      LOG_WARN(MODULE_GPS, "Update rate %d Hz not supported", hz);
      return;
  }
  
  if (command) {
    sendCommand(command);
    currentUpdateRate = hz;
    highUpdateRateEnabled = (hz > 1);
    LOG_INFO(MODULE_GPS, "GPS update rate set to %d Hz", hz);
  }
}

// Set update rate (generic)
void GpsManager::setUpdateRate(int hz) {
  enableHighUpdateRate(hz);
}

// Enable/disable SBAS
void GpsManager::enableSBAS(bool enable) {
  if (enable) {
    sendCommand(PMTK_ENABLE_SBAS);
    sbasEnabled = true;
    LOG_INFO(MODULE_GPS, "SBAS enabled");
  } else {
    sendCommand("$PMTK301,0*2C"); // Disable SBAS
    sbasEnabled = false;
    LOG_INFO(MODULE_GPS, "SBAS disabled");
  }
}

// Enable/disable WAAS
void GpsManager::enableWAAS(bool enable) {
  if (enable) {
    sendCommand(PMTK_ENABLE_WAAS);
    LOG_INFO(MODULE_GPS, "WAAS enabled");
  } else {
    sendCommand("$PMTK313,0*2F"); // Disable WAAS
    LOG_INFO(MODULE_GPS, "WAAS disabled");
  }
}

// Perform cold start
void GpsManager::performColdStart() {
  LOG_INFO(MODULE_GPS, "Performing GPS cold start...");
  sendCommand(PMTK_CMD_COLD_START);
  
  // Reset tracking variables
  firstFixAcquired = false;
  firstFixTime = 0;
  lastFixValid = false;
  newFixFlag = false;
  filterReady = false;
  filterIndex = 0;
  
  delay(100);
}

// Perform warm start
void GpsManager::performWarmStart() {
  LOG_INFO(MODULE_GPS, "Performing GPS warm start...");
  sendCommand(PMTK_CMD_WARM_START);
  delay(100);
}

// Perform hot start
void GpsManager::performHotStart() {
  LOG_INFO(MODULE_GPS, "Performing GPS hot start...");
  sendCommand(PMTK_CMD_HOT_START);
  delay(100);
}

// Factory reset
void GpsManager::factoryReset() {
  LOG_WARN(MODULE_GPS, "Performing GPS factory reset...");
  sendCommand("$PMTK104*37"); // Full cold start and reset
  delay(1000);
  
  // Re-initialize after reset
  begin();
}

// Set custom NMEA output
void GpsManager::setNMEAOutput(const char* config) {
  sendCommand(config);
  LOG_INFO(MODULE_GPS, "NMEA output configured");
}

// Enter standby mode
void GpsManager::enterStandbyMode() {
  LOG_INFO(MODULE_GPS, "GPS entering standby mode");
  sendCommand(PMTK_STANDBY_MODE);
}

// Wake up from standby
void GpsManager::wakeUp() {
  LOG_INFO(MODULE_GPS, "GPS waking up");
  sendCommand(PMTK_AWAKE);
  delay(100);
}

// Print diagnostics
void GpsManager::printDiagnostics() {
  LOG_INFO(MODULE_GPS, "=== GPS DIAGNOSTICS ===");
  LOG_INFO(MODULE_GPS, "Status: %s", isValid() ? "VALID" : "INVALID");
  LOG_INFO(MODULE_GPS, "Fix Type: %s", getFixStatusString());
  LOG_INFO(MODULE_GPS, "Satellites: %d", getSatellites());
  LOG_INFO(MODULE_GPS, "HDOP: %.1f", currentHDOP);
  LOG_INFO(MODULE_GPS, "Update Rate: %d Hz", currentUpdateRate);
  LOG_INFO(MODULE_GPS, "SBAS: %s", sbasEnabled ? "Enabled" : "Disabled");
  
  if (firstFixAcquired) {
    LOG_INFO(MODULE_GPS, "Time to First Fix: %lu seconds", getTimeToFirstFix());
  } else {
    LOG_INFO(MODULE_GPS, "First Fix: NOT ACQUIRED");
  }
  
  LOG_INFO(MODULE_GPS, "Total Sentences: %lu", totalSentences);
  LOG_INFO(MODULE_GPS, "Valid Sentences: %lu", validSentences);
  LOG_INFO(MODULE_GPS, "Checksum Errors: %lu", checksumErrors);
  LOG_INFO(MODULE_GPS, "Success Rate: %.1f%%", getSentenceSuccessRate());
  
  if (isValid()) {
    LOG_INFO(MODULE_GPS, "Position: %.6f, %.6f", getLatitude(), getLongitude());
    LOG_INFO(MODULE_GPS, "Altitude: %.1f m", getAltitude());
    LOG_INFO(MODULE_GPS, "Speed: %.1f km/h", getSpeed());
    LOG_INFO(MODULE_GPS, "Heading: %.1f°", getHeading());
  }
  
  LOG_INFO(MODULE_GPS, "======================");
}

// Wait for fix (blocking)
bool GpsManager::waitForFix(unsigned long timeout) {
  unsigned long startTime = millis();
  
  LOG_INFO(MODULE_GPS, "Waiting for GPS fix (timeout: %lu ms)...", timeout);
  
  while (millis() - startTime < timeout) {
    update();
    
    if (isValid()) {
      LOG_INFO(MODULE_GPS, "GPS fix acquired in %lu ms", millis() - startTime);
      return true;
    }
    
    // Log progress every 5 seconds
    if ((millis() - startTime) % 5000 < 100) {
      LOG_DEBUG(MODULE_GPS, "Waiting for fix... Sats: %d, HDOP: %.1f", 
                getSatellites(), currentHDOP);
    }
    
    Utils::feedWatchdog();
    delay(100);
  }
  
  LOG_WARN(MODULE_GPS, "GPS fix timeout after %lu ms", timeout);
  return false;
}

// Optimize for cold start
void GpsManager::optimizeForColdStart() {
  LOG_INFO(MODULE_GPS, "Optimizing GPS for cold start...");
  
  // 1. Enable all NMEA sentences for faster acquisition
  sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  delay(100);
  
  // 2. Set higher update rate for faster fix
  enableHighUpdateRate(5);
  delay(100);
  
  // 3. Enable SBAS and WAAS
  enableSBAS(true);
  enableWAAS(true);
  delay(100);
  
  // 4. Perform full cold start
  performColdStart();
}

// Optimize for moving vehicle
void GpsManager::optimizeForMoving() {
  LOG_INFO(MODULE_GPS, "Optimizing GPS for moving vehicle...");
  
  // 1. Increase update rate for better tracking
  enableHighUpdateRate(10);
  delay(100);
  
  // 2. Optimize NMEA output for essential data only
  sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(100);
  
  // 3. Ensure SBAS is enabled for accuracy
  enableSBAS(true);
}

// Optimize for stationary use
void GpsManager::optimizeForStationary() {
  LOG_INFO(MODULE_GPS, "Optimizing GPS for stationary use...");
  
  // 1. Reduce update rate to save power
  enableHighUpdateRate(1);
  delay(100);
  
  // 2. Enable all accuracy features
  enableSBAS(true);
  enableWAAS(true);
}

// ===== GPS Utility Functions Implementation =====
namespace GpsUtils {
  
  // Format coordinate as Degrees Minutes Seconds
  String formatDMS(float coordinate, bool isLatitude) {
    char direction;
    if (isLatitude) {
      direction = coordinate >= 0 ? 'N' : 'S';
    } else {
      direction = coordinate >= 0 ? 'E' : 'W';
    }
    
    coordinate = abs(coordinate);
    int degrees = (int)coordinate;
    float minutesFloat = (coordinate - degrees) * 60;
    int minutes = (int)minutesFloat;
    float seconds = (minutesFloat - minutes) * 60;
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%d°%d'%.2f\"%c", 
             degrees, minutes, seconds, direction);
    
    return String(buffer);
  }
  
  // Convert DMS to decimal degrees
  float DMStoDegrees(int degrees, int minutes, float seconds) {
    return degrees + (minutes / 60.0) + (seconds / 3600.0);
  }
  
  // Get compass direction from heading
  String getCompassDirection(float heading) {
    const char* directions[] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };
    
    int index = (int)((heading + 11.25) / 22.5) % 16;
    return String(directions[index]);
  }
  
  // Calculate bearing between two points
  float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 = lat1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    
    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    float bearing = atan2(y, x) * RAD_TO_DEG;
    return fmod((bearing + 360), 360);
  }
}