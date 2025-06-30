// ========================================
// Main.cpp - Enhanced Real-time Vehicle Tracking
// ========================================

/**
 * ESP32 Vehicle Tracking with Enhanced Real-time Features
 * - Adaptive GPS intervals based on speed ranges
 * - Position change detection
 * - Heading change detection
 * - Message buffering for network issues
 * - Predictive tracking
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
#include <CircularBuffer.h> // For message buffering

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

// ----- MOVEMENT DETECTION ENHANCED -----
enum MovementState {
  MOVEMENT_UNKNOWN,
  MOVEMENT_STATIC,
  MOVEMENT_CREEPING,    // Very slow movement (< 3 km/h)
  MOVEMENT_SLOW,        // Slow movement (3-20 km/h)
  MOVEMENT_MEDIUM,      // Medium speed (20-60 km/h)
  MOVEMENT_FAST         // High speed (> 60 km/h)
};

// ----- GPS DATA STRUCTURE -----
struct GpsData {
  float latitude;
  float longitude;
  float speed;
  float heading;
  int satellites;
  unsigned long timestamp;
  bool valid;
};

// ----- MESSAGE BUFFER -----
CircularBuffer<String, WS_MESSAGE_QUEUE_SIZE> messageQueue;

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

// ----- ENHANCED TRACKING VARIABLES -----
MovementState currentMovementState = MOVEMENT_UNKNOWN;
GpsData lastGpsData = {0};
GpsData currentGpsData = {0};
float speedHistory[5] = {0};  // Rolling speed average
int speedHistoryIndex = 0;
unsigned long lastPositionChangeTime = 0;
float totalDistanceTraveled = 0;

// Acceleration detection
float lastSpeed = 0;
unsigned long lastSpeedTime = 0;
float currentAcceleration = 0;

// ----- FUNCTION PROTOTYPES -----
void handleInitState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSerialCommands();
void printStatus();
void printHelp();
bool sendVehicleDataViaWebSocket();
void onRelayUpdate(bool newState);

// Enhanced tracking functions
void updateEnhancedTracking();
MovementState detectMovementState();
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateHeadingChange(float h1, float h2);
bool shouldSendUpdate();
unsigned long getAdaptiveGpsInterval();
void processMessageQueue();
float getRollingAverageSpeed();
void detectAcceleration();
bool isSignificantPositionChange();
bool isSignificantHeadingChange();
bool isSignificantSpeedChange();
const char* getMovementStateString(MovementState state);

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
  
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 Enhanced Real-time Vehicle Tracking ===");
  LOG_INFO(MODULE_MAIN, "Version: 4.0 (Real-time Optimization)");
  LOG_INFO(MODULE_MAIN, "WebSocket URL: %s", WS_URL);
  LOG_INFO(MODULE_MAIN, "GPS ID: %s", GPS_ID);
  
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  
  // Initialize with higher baud rate if supported
  #ifdef GPS_BAUD_RATE
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    LOG_INFO(MODULE_GPS, "GPS initialized at %d baud", GPS_BAUD_RATE);
  #else
    gpsManager.begin();
  #endif
  
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
  
  // High-frequency GPS update
  gpsManager.update();
  
  // Update enhanced tracking
  updateEnhancedTracking();
  
  // Process WebSocket messages
  if (currentState == STATE_OPERATIONAL) {
    wsManager.update();
    processMessageQueue();
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
  
  // Faster stuck detection
  if (currentState == STATE_OPERATIONAL) {
    unsigned long timeSinceSuccess = millis() - lastSuccessfulOperation;
    if (timeSinceSuccess > SYSTEM_STUCK_TIMEOUT) {
      LOG_WARN(MODULE_SYS, "System appears stuck, initiating recovery");
      currentState = STATE_CONNECTION_RECOVERY;
    }
  }
  
  delay(5); // Reduced delay for higher update rate
}

// ----- ENHANCED TRACKING IMPLEMENTATION -----
void updateEnhancedTracking() {
  if (!gpsManager.isValid()) return;
  
  // Update current GPS data
  currentGpsData.latitude = gpsManager.getLatitude();
  currentGpsData.longitude = gpsManager.getLongitude();
  currentGpsData.speed = gpsManager.isSpeedValid() ? gpsManager.getSpeed() : 0.0;
  currentGpsData.heading = gps.course.isValid() ? gps.course.deg() : 0.0;
  currentGpsData.satellites = gpsManager.getSatellites();
  currentGpsData.timestamp = millis();
  currentGpsData.valid = true;
  
  // Update speed history
  speedHistory[speedHistoryIndex] = currentGpsData.speed;
  speedHistoryIndex = (speedHistoryIndex + 1) % 5;
  
  // Detect acceleration/deceleration
  detectAcceleration();
  
  // Update movement state
  MovementState newState = detectMovementState();
  if (newState != currentMovementState) {
    LOG_INFO(MODULE_GPS, "🚗 Movement state: %s → %s (speed: %.1f km/h, accel: %.1f m/s²)", 
             getMovementStateString(currentMovementState),
             getMovementStateString(newState),
             currentGpsData.speed,
             currentAcceleration);
    currentMovementState = newState;
  }
  
  // Calculate distance traveled
  if (lastGpsData.valid) {
    float distance = calculateDistance(
      lastGpsData.latitude, lastGpsData.longitude,
      currentGpsData.latitude, currentGpsData.longitude
    );
    totalDistanceTraveled += distance;
  }
}

MovementState detectMovementState() {
  float avgSpeed = getRollingAverageSpeed();
  
  // Multi-level movement detection
  if (avgSpeed < SPEED_THRESHOLD_CREEP) {
    return MOVEMENT_STATIC;
  } else if (avgSpeed < SPEED_THRESHOLD_LOW) {
    return MOVEMENT_CREEPING;
  } else if (avgSpeed < SPEED_THRESHOLD_MEDIUM) {
    return MOVEMENT_SLOW;
  } else if (avgSpeed < SPEED_THRESHOLD_HIGH) {
    return MOVEMENT_MEDIUM;
  } else {
    return MOVEMENT_FAST;
  }
}

bool shouldSendUpdate() {
  unsigned long currentTime = millis();
  unsigned long timeSinceLastSend = currentTime - lastGpsSendTime;
  unsigned long adaptiveInterval = getAdaptiveGpsInterval();
  
  // 1. Time-based update
  if (timeSinceLastSend >= adaptiveInterval) {
    return true;
  }
  
  // 2. Significant position change
  if (isSignificantPositionChange()) {
    LOG_DEBUG(MODULE_GPS, "Significant position change detected");
    return true;
  }
  
  // 3. Significant heading change (for turns)
  if (isSignificantHeadingChange() && currentGpsData.speed > SPEED_THRESHOLD_LOW) {
    LOG_DEBUG(MODULE_GPS, "Significant heading change detected");
    return true;
  }
  
  // 4. Significant speed change (acceleration/braking)
  if (isSignificantSpeedChange()) {
    LOG_DEBUG(MODULE_GPS, "Significant speed change detected");
    return true;
  }
  
  // 5. GPS fix just acquired
  if (gpsManager.hasNewFix()) {
    LOG_DEBUG(MODULE_GPS, "New GPS fix acquired");
    return true;
  }
  
  return false;
}

unsigned long getAdaptiveGpsInterval() {
  // Adaptive interval based on movement state and conditions
  switch (currentMovementState) {
    case MOVEMENT_FAST:
      return GPS_INTERVAL_HIGH_SPEED;
      
    case MOVEMENT_MEDIUM:
      // Faster updates during acceleration/deceleration
      if (abs(currentAcceleration) > 2.0) {
        return GPS_INTERVAL_HIGH_SPEED;
      }
      return GPS_INTERVAL_MEDIUM_SPEED;
      
    case MOVEMENT_SLOW:
      // Faster updates in turns
      if (isSignificantHeadingChange()) {
        return GPS_INTERVAL_MEDIUM_SPEED;
      }
      return GPS_INTERVAL_LOW_SPEED;
      
    case MOVEMENT_CREEPING:
      return GPS_INTERVAL_LOW_SPEED * 2; // 2 seconds for very slow movement
      
    case MOVEMENT_STATIC:
    case MOVEMENT_UNKNOWN:
    default:
      return GPS_INTERVAL_STATIC;
  }
}

void handleOperationalState() {
  if (shouldSendUpdate()) {
    if (sendVehicleDataViaWebSocket()) {
      lastGpsSendTime = millis();
      lastSuccessfulOperation = millis();
      
      // Update last GPS data after successful send
      memcpy(&lastGpsData, &currentGpsData, sizeof(GpsData));
    }
  }
}

bool sendVehicleDataViaWebSocket() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No GPRS connection");
    return false;
  }
  
  // If WebSocket not ready, queue the message
  if (!wsManager.isReady()) {
    LOG_WARN(MODULE_GPS, "WebSocket not ready, queueing message...");
    
    // Create and queue the message
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    String isoTimestamp = String(timestamp);
    if (isoTimestamp.endsWith("Z")) {
      isoTimestamp.replace("Z", ".000Z");
    }
    
    StaticJsonDocument<384> doc;
    doc["type"] = "items";
    doc["collection"] = "vehicle_datas";
    doc["action"] = "create";
    
    JsonObject data = doc.createNestedObject("data");
    data["gps_id"] = GPS_ID;
    data["latitude"] = String(currentGpsData.latitude, 6);  // Higher precision
    data["longitude"] = String(currentGpsData.longitude, 6);
    data["speed"] = (int)currentGpsData.speed;
    data["heading"] = (int)currentGpsData.heading;
    data["satellites_used"] = currentGpsData.satellites;
    data["timestamp"] = isoTimestamp;
    data["battery_level"] = 12.5;
    
    String message;
    serializeJson(doc, message);
    
    if (!messageQueue.isFull()) {
      messageQueue.push(message);
      LOG_INFO(MODULE_GPS, "Message queued (%d/%d)", messageQueue.size(), WS_MESSAGE_QUEUE_SIZE);
    } else {
      LOG_WARN(MODULE_GPS, "Message queue full, dropping oldest");
      messageQueue.shift(); // Remove oldest
      messageQueue.push(message);
    }
    
    // Try to reconnect WebSocket
    wsManager.connect();
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "📡 Sending GPS data [%s, %.1f km/h, interval: %lums]", 
           getMovementStateString(currentMovementState), 
           currentGpsData.speed,
           getAdaptiveGpsInterval());
  
  // Prepare timestamp
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  String isoTimestamp = String(timestamp);
  if (isoTimestamp.endsWith("Z")) {
    isoTimestamp.replace("Z", ".000Z");
  }
  
  bool success = false;
  if (gpsManager.isValid()) {
    success = wsManager.sendVehicleData(
      currentGpsData.latitude,
      currentGpsData.longitude,
      currentGpsData.speed,
      currentGpsData.satellites,
      isoTimestamp
    );
    
    if (success) {
      LOG_INFO(MODULE_GPS, "✅ GPS data sent successfully");
      LOG_DEBUG(MODULE_GPS, "📍 lat=%.6f, lon=%.6f, spd=%.1f, hdg=%.0f°, sats=%d",
               currentGpsData.latitude, currentGpsData.longitude,
               currentGpsData.speed, currentGpsData.heading,
               currentGpsData.satellites);
    }
  }
  
  return success;
}

void processMessageQueue() {
  // Process queued messages when WebSocket is ready
  if (wsManager.isReady() && !messageQueue.isEmpty()) {
    String message = messageQueue.shift();
    wsManager.sendText(message);
    LOG_INFO(MODULE_GPS, "Sent queued message (%d remaining)", messageQueue.size());
    
    // Small delay between queued messages
    delay(100);
  }
}

// ----- HELPER FUNCTIONS -----
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula for small distances
  const float R = 6371000; // Earth radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

float calculateHeadingChange(float h1, float h2) {
  float diff = abs(h2 - h1);
  if (diff > 180) diff = 360 - diff;
  return diff;
}

float getRollingAverageSpeed() {
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += speedHistory[i];
  }
  return sum / 5.0;
}

void detectAcceleration() {
  unsigned long currentTime = millis();
  if (lastSpeedTime > 0) {
    float timeDiff = (currentTime - lastSpeedTime) / 1000.0; // seconds
    float speedDiff = currentGpsData.speed - lastSpeed; // km/h
    currentAcceleration = (speedDiff * 0.2778) / timeDiff; // m/s²
  }
  lastSpeed = currentGpsData.speed;
  lastSpeedTime = currentTime;
}

bool isSignificantPositionChange() {
  if (!lastGpsData.valid) return false;
  
  float distance = calculateDistance(
    lastGpsData.latitude, lastGpsData.longitude,
    currentGpsData.latitude, currentGpsData.longitude
  );
  
  // Dynamic threshold based on speed
  float threshold = 5.0; // 5 meters default
  if (currentMovementState == MOVEMENT_FAST) {
    threshold = 10.0; // 10 meters for high speed
  } else if (currentMovementState == MOVEMENT_MEDIUM) {
    threshold = 7.0; // 7 meters for medium speed
  }
  
  return distance > threshold;
}

bool isSignificantHeadingChange() {
  if (!lastGpsData.valid) return false;
  
  float headingChange = calculateHeadingChange(
    lastGpsData.heading, currentGpsData.heading
  );
  
  return headingChange > HEADING_CHANGE_THRESHOLD;
}

bool isSignificantSpeedChange() {
  if (!lastGpsData.valid) return false;
  
  float speedChange = abs(currentGpsData.speed - lastGpsData.speed);
  
  // Dynamic threshold
  float threshold = 5.0; // 5 km/h default
  if (currentMovementState == MOVEMENT_FAST) {
    threshold = 10.0; // 10 km/h for high speed
  }
  
  // Also check acceleration
  return speedChange > threshold || abs(currentAcceleration) > 2.0; // 2 m/s²
}

const char* getMovementStateString(MovementState state) {
  switch (state) {
    case MOVEMENT_FAST: return "FAST";
    case MOVEMENT_MEDIUM: return "MEDIUM";
    case MOVEMENT_SLOW: return "SLOW";
    case MOVEMENT_CREEPING: return "CREEPING";
    case MOVEMENT_STATIC: return "STATIC";
    case MOVEMENT_UNKNOWN: return "UNKNOWN";
    default: return "INVALID";
  }
}

// ----- STATE HANDLERS (same as before) -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "Initializing system...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "System initialization successful");
    
    if (wsManager.connect()) {
      LOG_INFO(MODULE_SYS, "WebSocket connected successfully");
    } else {
      LOG_WARN(MODULE_SYS, "WebSocket connection failed, will retry...");
    }
    
    currentState = STATE_OPERATIONAL;
    lastSuccessfulOperation = millis();
  } else {
    LOG_ERROR(MODULE_SYS, "Initialization failed, starting modem reset");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
    if (modemManager.setup()) {
      LOG_INFO(MODULE_SYS, "Modem reset successful");
      currentState = STATE_OPERATIONAL;
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
      LOG_INFO(MODULE_MAIN, "Queue: %d messages", messageQueue.size());
    } else if (cmd == "tracking") {
      LOG_INFO(MODULE_MAIN, "=== TRACKING STATUS ===");
      LOG_INFO(MODULE_MAIN, "Movement: %s", getMovementStateString(currentMovementState));
      LOG_INFO(MODULE_MAIN, "Speed: %.1f km/h (avg: %.1f)", currentGpsData.speed, getRollingAverageSpeed());
      LOG_INFO(MODULE_MAIN, "Acceleration: %.2f m/s²", currentAcceleration);
      LOG_INFO(MODULE_MAIN, "Distance traveled: %.2f km", totalDistanceTraveled / 1000.0);
      LOG_INFO(MODULE_MAIN, "Update interval: %lu ms", getAdaptiveGpsInterval());
    } else if (cmd == "connect") {
      wsManager.connect();
    } else if (cmd == "disconnect") {
      wsManager.disconnect();
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
  
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f) Speed: %.1f km/h, Heading: %.0f°, Sats: %d",
             currentGpsData.latitude, currentGpsData.longitude,
             currentGpsData.speed, currentGpsData.heading,
             currentGpsData.satellites);
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix");
  }
  
  LOG_INFO(MODULE_MAIN, "Movement: %s (interval: %lums)", 
           getMovementStateString(currentMovementState), 
           getAdaptiveGpsInterval());
  
  LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
  
  if (modemManager.isGprsConnected()) {
    int csq = modemManager.getSignalQuality();
    LOG_INFO(MODULE_MAIN, "GPRS: Connected, Signal: %d (%s)", 
             csq, Utils::getSignalQualityString(csq));
  } else {
    LOG_INFO(MODULE_MAIN, "GPRS: Not connected");
  }
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "help       - Show this help");
  LOG_INFO(MODULE_MAIN, "status     - Show system status");
  LOG_INFO(MODULE_MAIN, "tracking   - Show tracking details");
  LOG_INFO(MODULE_MAIN, "on/off     - Manual relay control");
  LOG_INFO(MODULE_MAIN, "gps        - Send GPS data now");
  LOG_INFO(MODULE_MAIN, "ws         - Show WebSocket status");
  LOG_INFO(MODULE_MAIN, "connect    - Connect WebSocket");
  LOG_INFO(MODULE_MAIN, "disconnect - Disconnect WebSocket");
  LOG_INFO(MODULE_MAIN, "reset      - Restart system");
}