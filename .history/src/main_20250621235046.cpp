// ========================================
// Main.cpp - Enhanced for Aggressive GPS Transmission
// ========================================

/**
 * ESP32 Vehicle Tracking with Aggressive WebSocket Updates
 * - Real-time GPS data transmission with minimal delays
 * - Enhanced movement detection with acceleration and turning
 * - Position interpolation and smoothing
 * - Offline data buffering
 */

// ----- ARDUINO FRAMEWORK -----
#include <Arduino.h>

// ----- DEFINITIONS FOR MODEM -----
#define SerialMon Serial
#define SerialAT Serial1

// ----- INCLUDES -----
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// Custom modules
#include "Config.h"
#include "Logger.h"
#include "Utils.h"
#include "GpsManager.h"
#include "ModemManager.h"
#include "HttpClient.h"
#include "WebSocketManager.h"

// ----- SYSTEM STATES -----
enum SystemState {
  STATE_INIT,
  STATE_IDLE,
  STATE_OPERATIONAL,
  STATE_MODEM_RESET,
  STATE_CONNECTION_RECOVERY,
  STATE_ERROR
};

// ----- ENHANCED MOVEMENT STATES -----
enum MovementState {
  MOVEMENT_UNKNOWN,
  MOVEMENT_STATIC,
  MOVEMENT_MOVING,
  MOVEMENT_ACCELERATING,
  MOVEMENT_TURNING
};

// ----- GPS DATA STRUCTURE -----
struct GpsDataPoint {
  float latitude;
  float longitude;
  float speed;
  float heading;
  int satellites;
  unsigned long timestamp;
  bool valid;
};

// ----- GLOBAL OBJECTS -----
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
GpsManager gpsManager(gps, SerialGPS);
ModemManager modemManager(modem, SerialAT);
HttpClientWrapper httpClient(gsmClient);
WebSocketManager wsManager(&gsmClient);

// ----- STATE VARIABLES -----
SystemState currentState = STATE_INIT;
unsigned long lastGpsSendTime = 0;
unsigned long lastSuccessfulOperation = 0;
bool relayState = true;

// ----- ENHANCED MOVEMENT DETECTION VARIABLES -----
MovementState currentMovementState = MOVEMENT_UNKNOWN;
float speedSamples[MOVEMENT_DETECTION_SAMPLES];
float headingSamples[MOVEMENT_DETECTION_SAMPLES];
int speedSampleIndex = 0;
bool speedSamplesReady = false;
unsigned long lastMovementLogTime = 0;
float lastSpeed = 0.0;
float lastHeading = 0.0;
unsigned long lastSpeedCheckTime = 0;

// ----- GPS DATA BUFFERING -----
GpsDataPoint gpsBuffer[GPS_DATA_BUFFER_SIZE];
int gpsBufferIndex = 0;
int gpsBufferCount = 0;
GpsDataPoint lastValidPosition;
GpsDataPoint smoothedPosition;

// ----- CONNECTION TRACKING -----
int consecutiveConnectionFailures = 0;
bool isOfflineMode = false;
unsigned long offlineModeStartTime = 0;

// ----- FUNCTION PROTOTYPES -----
void handleInitState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSerialCommands();
void printStatus();
void printHelp();
bool sendVehicleDataViaWebSocket();
bool sendBufferedData();
void onRelayUpdate(bool newState);

// Enhanced movement detection functions
void updateMovementDetection();
MovementState detectMovementState();
float getAverageSpeed();
float getSpeedChange();
float getHeadingChange();
unsigned long getCurrentGpsInterval();
const char* getMovementStateString(MovementState state);

// GPS data processing
void updateGpsData();
bool isValidGpsData();
void smoothGpsPosition();
void bufferGpsData();
float calculateDistance(float lat1, float lon1, float lat2, float lon2);

// ----- WEBSOCKET CALLBACK -----
void onRelayUpdate(bool newState) {
  if (newState != relayState) {
    LOG_INFO(MODULE_RELAY, "🔄 Relay update from WebSocket: %s → %s", 
             relayState ? "ON" : "OFF", newState ? "ON" : "OFF");
    
    digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
    relayState = newState;
    
    LOG_INFO(MODULE_RELAY, "✅ Physical relay updated to: %s", newState ? "ON" : "OFF");
  }
}

// ----- SETUP -----
void setup() {
  SerialMon.begin(115200);
  delay(100);
  
  Logger::init(&SerialMon, LOG_DEBUG); // Set to DEBUG for more info
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 Vehicle Tracking System ===");
  LOG_INFO(MODULE_MAIN, "Version: 4.0 (Aggressive Tracking)");
  LOG_INFO(MODULE_MAIN, "WebSocket URL: %s", WS_URL);
  LOG_INFO(MODULE_MAIN, "GPS ID: %s", GPS_ID);
  
  // Initialize arrays
  for (int i = 0; i < MOVEMENT_DETECTION_SAMPLES; i++) {
    speedSamples[i] = 0.0;
    headingSamples[i] = 0.0;
  }
  
  // Initialize GPS data structures
  lastValidPosition = {0, 0, 0, 0, 0, 0, false};
  smoothedPosition = {0, 0, 0, 0, 0, 0, false};
  
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  
  gpsManager.begin();
  modemManager.begin();
  
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  printHelp();
  
  currentState = STATE_INIT;
  lastSuccessfulOperation = millis();
}

// ----- MAIN LOOP -----
void loop() {
  Utils::feedWatchdog();
  
  // High frequency GPS update
  static unsigned long lastGpsUpdate = 0;
  if (millis() - lastGpsUpdate >= GPS_UPDATE_RATE) {
    gpsManager.update();
    updateGpsData();
    updateMovementDetection();
    lastGpsUpdate = millis();
  }
  
  // Update WebSocket connection
  if (currentState == STATE_OPERATIONAL) {
    wsManager.update();
  }
  
  handleSerialCommands();
  
  // State machine
  switch (currentState) {
    case STATE_INIT:
      handleInitState();
      break;
      
    case STATE_IDLE:
    case STATE_OPERATIONAL:
      handleOperationalState();
      break;
      
    case STATE_MODEM_RESET:
      handleModemResetState();
      break;
      
    case STATE_CONNECTION_RECOVERY:
      handleConnectionRecoveryState();
      break;
      
    case STATE_ERROR:
      LOG_ERROR(MODULE_SYS, "System in error state, attempting recovery...");
      modemManager.startReset();
      currentState = STATE_MODEM_RESET;
      break;
  }
  
  // Check for system stuck condition
  if (currentState == STATE_OPERATIONAL) {
    unsigned long timeSinceSuccess = millis() - lastSuccessfulOperation;
    if (timeSinceSuccess > SYSTEM_STUCK_TIMEOUT) {
      LOG_WARN(MODULE_SYS, "System appears stuck, initiating recovery");
      currentState = STATE_CONNECTION_RECOVERY;
    }
  }
  
  delay(5); // Reduced delay for more responsive loop
}

// ----- STATE HANDLERS -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "Initializing system...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "System initialization successful");
    
    if (wsManager.connect()) {
      LOG_INFO(MODULE_SYS, "WebSocket connected successfully");
      isOfflineMode = false;
    } else {
      LOG_WARN(MODULE_SYS, "WebSocket connection failed, starting in offline mode");
      isOfflineMode = true;
      offlineModeStartTime = millis();
    }
    
    currentState = STATE_OPERATIONAL;
    lastSuccessfulOperation = millis();
  } else {
    LOG_ERROR(MODULE_SYS, "Initialization failed, starting modem reset");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleOperationalState() {
  unsigned long currentTime = millis();
  unsigned long currentGpsInterval = getCurrentGpsInterval();
  
  // Force send if we have a significant position change
  bool forceSend = false;
  if (lastValidPosition.valid && smoothedPosition.valid) {
    float distance = calculateDistance(
      lastValidPosition.latitude, lastValidPosition.longitude,
      smoothedPosition.latitude, smoothedPosition.longitude
    );
    
    // Force send if moved more than 5 meters
    if (distance > 5.0) {
      forceSend = true;
      LOG_DEBUG(MODULE_GPS, "🚀 Force send: moved %.1f meters", distance);
    }
  }
  
  // Check if time to send vehicle data
  if (forceSend || currentTime - lastGpsSendTime >= currentGpsInterval || gpsManager.hasNewFix()) {
    
    if (isOfflineMode && ENABLE_OFFLINE_BUFFERING) {
      // Buffer data when offline
      bufferGpsData();
      LOG_DEBUG(MODULE_GPS, "📦 Buffered GPS data (offline mode, buffer: %d/%d)", 
                gpsBufferCount, GPS_DATA_BUFFER_SIZE);
      
      // Try to reconnect periodically
      if (currentTime - offlineModeStartTime > 10000) {
        if (wsManager.connect()) {
          LOG_INFO(MODULE_SYS, "🔄 Reconnected! Sending buffered data...");
          isOfflineMode = false;
          sendBufferedData();
        } else {
          offlineModeStartTime = currentTime;
        }
      }
    } else {
      // Normal online transmission
      if (sendVehicleDataViaWebSocket()) {
        lastGpsSendTime = currentTime;
        lastSuccessfulOperation = currentTime;
        consecutiveConnectionFailures = 0;
        
        // Update last valid position after successful send
        if (smoothedPosition.valid) {
          lastValidPosition = smoothedPosition;
        }
      } else {
        consecutiveConnectionFailures++;
        if (consecutiveConnectionFailures >= 3) {
          LOG_WARN(MODULE_SYS, "Multiple connection failures, entering offline mode");
          isOfflineMode = true;
          offlineModeStartTime = currentTime;
        }
      }
    }
  }
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
    if (modemManager.setup()) {
      LOG_INFO(MODULE_SYS, "Modem reset successful");
      currentState = STATE_OPERATIONAL;
      consecutiveConnectionFailures = 0;
    } else {
      LOG_ERROR(MODULE_SYS, "Modem reset failed");
      currentState = STATE_ERROR;
    }
  }
}

void handleConnectionRecoveryState() {
  LOG_INFO(MODULE_SYS, "Attempting connection recovery...");
  
  wsManager.disconnect();
  modemManager.disconnectGprs();
  Utils::safeDelay(2000);
  
  if (modemManager.connectGprs()) {
    LOG_INFO(MODULE_SYS, "GPRS recovery successful");
    
    if (wsManager.connect()) {
      LOG_INFO(MODULE_SYS, "WebSocket reconnected");
      currentState = STATE_OPERATIONAL;
      lastSuccessfulOperation = millis();
      isOfflineMode = false;
      
      // Send any buffered data
      if (gpsBufferCount > 0) {
        sendBufferedData();
      }
    } else {
      LOG_ERROR(MODULE_SYS, "WebSocket reconnection failed");
      currentState = STATE_MODEM_RESET;
    }
  } else {
    LOG_ERROR(MODULE_SYS, "Connection recovery failed, trying modem reset");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

// ----- GPS DATA PROCESSING -----
void updateGpsData() {
  if (!gpsManager.isValid()) return;
  
  GpsDataPoint currentPoint = {
    gpsManager.getLatitude(),
    gpsManager.getLongitude(),
    gpsManager.isSpeedValid() ? gpsManager.getSpeed() : 0.0,
    gps.course.isValid() ? gps.course.deg() : lastValidPosition.heading,
    gpsManager.getSatellites(),
    millis(),
    true
  };
  
  // Apply position smoothing if enabled
  if (ENABLE_POSITION_INTERPOLATION && smoothedPosition.valid) {
    smoothGpsPosition();
  } else {
    smoothedPosition = currentPoint;
  }
}

void smoothGpsPosition() {
  if (!gpsManager.isValid()) return;
  
  float newLat = gpsManager.getLatitude();
  float newLon = gpsManager.getLongitude();
  
  // Check for GPS jumps
  if (smoothedPosition.valid) {
    float distance = calculateDistance(
      smoothedPosition.latitude, smoothedPosition.longitude,
      newLat, newLon
    );
    
    // Reject jumps larger than threshold
    if (distance > MAX_POSITION_JUMP_METERS) {
      LOG_WARN(MODULE_GPS, "GPS jump detected: %.1f meters, rejecting", distance);
      return;
    }
  }
  
  // Apply exponential smoothing
  if (smoothedPosition.valid) {
    smoothedPosition.latitude = smoothedPosition.latitude * (1.0 - POSITION_SMOOTHING_FACTOR) + 
                               newLat * POSITION_SMOOTHING_FACTOR;
    smoothedPosition.longitude = smoothedPosition.longitude * (1.0 - POSITION_SMOOTHING_FACTOR) + 
                                newLon * POSITION_SMOOTHING_FACTOR;
  } else {
    smoothedPosition.latitude = newLat;
    smoothedPosition.longitude = newLon;
  }
  
  smoothedPosition.speed = gpsManager.getSpeed();
  smoothedPosition.heading = gps.course.isValid() ? gps.course.deg() : smoothedPosition.heading;
  smoothedPosition.satellites = gpsManager.getSatellites();
  smoothedPosition.timestamp = millis();
  smoothedPosition.valid = true;
}

bool isValidGpsData() {
  if (!gpsManager.isValid()) return false;
  
  // Check satellite count
  if (gpsManager.getSatellites() < MIN_SATELLITES_FOR_SEND) {
    LOG_DEBUG(MODULE_GPS, "Not enough satellites: %d < %d", 
              gpsManager.getSatellites(), MIN_SATELLITES_FOR_SEND);
    return false;
  }
  
  // Check HDOP
  if (gps.hdop.isValid() && gps.hdop.hdop() > MIN_HDOP_FOR_SEND) {
    LOG_DEBUG(MODULE_GPS, "HDOP too high: %.1f > %.1f", 
              gps.hdop.hdop(), MIN_HDOP_FOR_SEND);
    return false;
  }
  
  return true;
}

void bufferGpsData() {
  if (!isValidGpsData()) return;
  
  gpsBuffer[gpsBufferIndex] = smoothedPosition;
  gpsBufferIndex = (gpsBufferIndex + 1) % GPS_DATA_BUFFER_SIZE;
  
  if (gpsBufferCount < GPS_DATA_BUFFER_SIZE) {
    gpsBufferCount++;
  }
}

bool sendBufferedData() {
  if (gpsBufferCount == 0) return true;
  
  LOG_INFO(MODULE_GPS, "📤 Sending %d buffered GPS points...", gpsBufferCount);
  
  int sent = 0;
  int startIndex = (gpsBufferIndex - gpsBufferCount + GPS_DATA_BUFFER_SIZE) % GPS_DATA_BUFFER_SIZE;
  
  for (int i = 0; i < gpsBufferCount; i++) {
    int idx = (startIndex + i) % GPS_DATA_BUFFER_SIZE;
    GpsDataPoint& point = gpsBuffer[idx];
    
    if (point.valid) {
      // Create timestamp from point
      char timestamp[30];
      gpsManager.getTimestamp(timestamp, sizeof(timestamp));
      
      if (wsManager.sendVehicleData(
        point.latitude,
        point.longitude,
        point.speed,
        point.satellites,
        String(timestamp)
      )) {
        sent++;
        delay(100); // Small delay between sends
      } else {
        LOG_WARN(MODULE_GPS, "Failed to send buffered point %d", i);
        break;
      }
    }
  }
  
  LOG_INFO(MODULE_GPS, "✅ Sent %d/%d buffered points", sent, gpsBufferCount);
  
  // Clear buffer if all sent
  if (sent == gpsBufferCount) {
    gpsBufferCount = 0;
    gpsBufferIndex = 0;
    return true;
  }
  
  return false;
}

// ----- VEHICLE DATA VIA WEBSOCKET -----
bool sendVehicleDataViaWebSocket() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No GPRS connection");
    return false;
  }
  
  if (!wsManager.isReady()) {
    LOG_WARN(MODULE_GPS, "WebSocket not ready, attempting to connect...");
    if (!wsManager.connect()) {
      return false;
    }
  }
  
  if (!isValidGpsData()) {
    LOG_DEBUG(MODULE_GPS, "GPS data not valid for sending");
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "📡 Sending GPS data [%s mode, interval: %lums]", 
           getMovementStateString(currentMovementState), getCurrentGpsInterval());
  
  // Prepare timestamp
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  
  // Convert to ISO format
  String isoTimestamp = String(timestamp);
  if (isoTimestamp.endsWith("Z")) {
    isoTimestamp.replace("Z", ".000Z");
  } else {
    int tzIndex = isoTimestamp.lastIndexOf('+');
    if (tzIndex == -1) tzIndex = isoTimestamp.lastIndexOf('-', isoTimestamp.length() - 6);
    if (tzIndex > 0) {
      isoTimestamp = isoTimestamp.substring(0, tzIndex) + ".000Z";
    } else {
      isoTimestamp += ".000Z";
    }
  }
  
  // Send smoothed position
  bool success = wsManager.sendVehicleData(
    smoothedPosition.latitude,
    smoothedPosition.longitude,
    smoothedPosition.speed,
    smoothedPosition.satellites,
    isoTimestamp
  );
  
  if (success) {
    LOG_INFO(MODULE_GPS, "✅ GPS sent: %.6f,%.6f speed:%.1f sats:%d hdop:%.1f",
             smoothedPosition.latitude, smoothedPosition.longitude,
             smoothedPosition.speed, smoothedPosition.satellites,
             gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
  }
  
  return success;
}

// ----- ENHANCED MOVEMENT DETECTION -----
void updateMovementDetection() {
  if (!gpsManager.isValid()) return;
  
  float currentSpeed = gpsManager.isSpeedValid() ? gpsManager.getSpeed() : 0.0;
  float currentHeading = gps.course.isValid() ? gps.course.deg() : lastHeading;
  
  speedSamples[speedSampleIndex] = currentSpeed;
  headingSamples[speedSampleIndex] = currentHeading;
  speedSampleIndex = (speedSampleIndex + 1) % MOVEMENT_DETECTION_SAMPLES;
  
  if (speedSampleIndex == 0 && !speedSamplesReady) {
    speedSamplesReady = true;
    LOG_INFO(MODULE_GPS, "Movement detection ready");
  }
  
  if (speedSamplesReady) {
    MovementState newState = detectMovementState();
    
    if (newState != currentMovementState) {
      LOG_INFO(MODULE_GPS, "🚗 Movement: %s → %s (speed:%.1f km/h, Δspeed:%.1f, Δheading:%.1f°)", 
               getMovementStateString(currentMovementState),
               getMovementStateString(newState),
               getAverageSpeed(),
               getSpeedChange(),
               getHeadingChange());
      currentMovementState = newState;
      lastMovementLogTime = millis();
    }
    
    // Periodic status log
    unsigned long currentTime = millis();
    if (currentTime - lastMovementLogTime >= 30000) {
      LOG_DEBUG(MODULE_GPS, "📊 Status: %s, Speed:%.1f, Interval:%lums", 
                getMovementStateString(currentMovementState),
                getAverageSpeed(),
                getCurrentGpsInterval());
      lastMovementLogTime = currentTime;
    }
  }
  
  lastSpeed = currentSpeed;
  lastHeading = currentHeading;
  lastSpeedCheckTime = millis();
}

MovementState detectMovementState() {
  if (!speedSamplesReady) return MOVEMENT_UNKNOWN;
  
  float avgSpeed = getAverageSpeed();
  float speedChange = getSpeedChange();
  float headingChange = getHeadingChange();
  
  // Deteksi berbelok
  if (avgSpeed > MOVEMENT_SPEED_THRESHOLD && 
      fabs(headingChange) > MOVEMENT_HEADING_CHANGE_THRESHOLD) {
    return MOVEMENT_TURNING;
  }
  
  // Deteksi akselerasi/deselerasi
  if (fabs(speedChange) > MOVEMENT_ACCELERATION_THRESHOLD) {
    return MOVEMENT_ACCELERATING;
  }
  
  // Deteksi bergerak normal atau diam
  if (avgSpeed >= MOVEMENT_SPEED_THRESHOLD) {
    return MOVEMENT_MOVING;
  } else {
    return MOVEMENT_STATIC;
  }
}

float getAverageSpeed() {
  if (!speedSamplesReady) return 0.0;
  
  float sum = 0.0;
  for (int i = 0; i < MOVEMENT_DETECTION_SAMPLES; i++) {
    sum += speedSamples[i];
  }
  return sum / MOVEMENT_DETECTION_SAMPLES;
}

float getSpeedChange() {
  if (!speedSamplesReady || MOVEMENT_DETECTION_SAMPLES < 2) return 0.0;
  
  // Calculate speed change rate (km/h per second)
  int newestIdx = (speedSampleIndex - 1 + MOVEMENT_DETECTION_SAMPLES) % MOVEMENT_DETECTION_SAMPLES;
  int oldestIdx = speedSampleIndex;
  
  float speedDiff = speedSamples[newestIdx] - speedSamples[oldestIdx];
  float timeDiff = (float)MOVEMENT_DETECTION_SAMPLES * (GPS_UPDATE_RATE / 1000.0);
  
  return speedDiff / timeDiff;
}

float getHeadingChange() {
  if (!speedSamplesReady || MOVEMENT_DETECTION_SAMPLES < 2) return 0.0;
  
  int newestIdx = (speedSampleIndex - 1 + MOVEMENT_DETECTION_SAMPLES) % MOVEMENT_DETECTION_SAMPLES;
  int oldestIdx = speedSampleIndex;
  
  float headingDiff = headingSamples[newestIdx] - headingSamples[oldestIdx];
  
  // Normalize heading difference to -180 to 180
  if (headingDiff > 180) headingDiff -= 360;
  if (headingDiff < -180) headingDiff += 360;
  
  return headingDiff;
}

unsigned long getCurrentGpsInterval() {
  switch (currentMovementState) {
    case MOVEMENT_TURNING:
      return GPS_SEND_INTERVAL_TURNING;
    case MOVEMENT_ACCELERATING:
      return GPS_SEND_INTERVAL_ACCELERATING;
    case MOVEMENT_MOVING:
      return GPS_SEND_INTERVAL_MOVING;
    case MOVEMENT_STATIC:
      return GPS_SEND_INTERVAL_STATIC;
    case MOVEMENT_UNKNOWN:
    default:
      return GPS_SEND_INTERVAL_MOVING;
  }
}

const char* getMovementStateString(MovementState state) {
  switch (state) {
    case MOVEMENT_MOVING: return "MOVING";
    case MOVEMENT_STATIC: return "STATIC";
    case MOVEMENT_TURNING: return "TURNING";
    case MOVEMENT_ACCELERATING: return "ACCEL";
    case MOVEMENT_UNKNOWN: return "UNKNOWN";
    default: return "INVALID";
  }
}

// ----- UTILITY FUNCTIONS -----
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula for distance in meters
  const float R = 6371000; // Earth radius in meters
  float dLat = (lat2 - lat1) * PI / 180.0;
  float dLon = (lon2 - lon1) * PI / 180.0;
  
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
            sin(dLon/2) * sin(dLon/2);
  
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// ----- SERIAL COMMANDS -----
void handleSerialCommands() {
  if (SerialMon.available()) {
    String cmd = SerialMon.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "help") {
      printHelp();
    } else if (cmd == "status") {
      printStatus();
    } else if (cmd == "on") {
      onRelayUpdate(true);
    } else if (cmd == "off") {
      onRelayUpdate(false);
    } else if (cmd == "gps") {
      sendVehicleDataViaWebSocket();
    } else if (cmd == "ws") {
      LOG_INFO(MODULE_MAIN, "=== WEBSOCKET STATUS ===");
      LOG_INFO(MODULE_MAIN, "State: %s", wsManager.getStateString());
      LOG_INFO(MODULE_MAIN, "URL: %s", WS_URL);
      LOG_INFO(MODULE_MAIN, "Offline Mode: %s", isOfflineMode ? "YES" : "NO");
      LOG_INFO(MODULE_MAIN, "Buffer: %d/%d", gpsBufferCount, GPS_DATA_BUFFER_SIZE);
    } else if (cmd == "connect") {
      wsManager.connect();
    } else if (cmd == "disconnect") {
      wsManager.disconnect();
    } else if (cmd == "buffer") {
      LOG_INFO(MODULE_MAIN, "=== GPS BUFFER STATUS ===");
      LOG_INFO(MODULE_MAIN, "Count: %d/%d", gpsBufferCount, GPS_DATA_BUFFER_SIZE);
      LOG_INFO(MODULE_MAIN, "Offline Mode: %s", isOfflineMode ? "YES" : "NO");
      if (gpsBufferCount > 0) {
        LOG_INFO(MODULE_MAIN, "Oldest data: %lu ms ago", 
                 millis() - gpsBuffer[(gpsBufferIndex - gpsBufferCount + GPS_DATA_BUFFER_SIZE) % GPS_DATA_BUFFER_SIZE].timestamp);
      }
    } else if (cmd == "reset") {
      ESP.restart();
    } else {
      LOG_WARN(MODULE_MAIN, "Unknown command: %s", cmd.c_str());
    }
  }
}

void printStatus() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM STATUS ===");
  LOG_INFO(MODULE_MAIN, "State: %d", currentState);
  LOG_INFO(MODULE_MAIN, "Uptime: %s", Utils::formatUptime(millis()).c_str());
  LOG_INFO(MODULE_MAIN, "WebSocket: %s", wsManager.getStateString());
  LOG_INFO(MODULE_MAIN, "Offline Mode: %s", isOfflineMode ? "YES" : "NO");
  
  // GPS Status
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f) Speed: %.1f km/h, Sats: %d, HDOP: %.1f",
             smoothedPosition.latitude, smoothedPosition.longitude,
             smoothedPosition.speed, smoothedPosition.satellites,
             gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix (Sats: %d)", gpsManager.getSatellites());
  }
  
  // Movement Status
  LOG_INFO(MODULE_MAIN, "Movement: %s (avg: %.1f km/h, interval: %lums)", 
           getMovementStateString(currentMovementState), 
           getAverageSpeed(), 
           getCurrentGpsInterval());
  
  // Relay Status
  LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
  
  // Modem Status
  if (modemManager.isGprsConnected()) {
    int csq = modemManager.getSignalQuality();
    LOG_INFO(MODULE_MAIN, "GPRS: Connected, Signal: %d (%s)", 
             csq, Utils::getSignalQualityString(csq));
  } else {
    LOG_INFO(MODULE_MAIN, "GPRS: Not connected");
  }
  
  // Buffer Status
  LOG_INFO(MODULE_MAIN, "GPS Buffer: %d/%d points", gpsBufferCount, GPS_DATA_BUFFER_SIZE);
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "help       - Show this help");
  LOG_INFO(MODULE_MAIN, "status     - Show system status");
  LOG_INFO(MODULE_MAIN, "on/off     - Manual relay control");
  LOG_INFO(MODULE_MAIN, "gps        - Send GPS data now");
  LOG_INFO(MODULE_MAIN, "ws         - Show WebSocket status");
  LOG_INFO(MODULE_MAIN, "buffer     - Show GPS buffer status");
  LOG_INFO(MODULE_MAIN, "connect    - Connect WebSocket");
  LOG_INFO(MODULE_MAIN, "disconnect - Disconnect WebSocket");
  LOG_INFO(MODULE_MAIN, "reset      - Restart system");
}