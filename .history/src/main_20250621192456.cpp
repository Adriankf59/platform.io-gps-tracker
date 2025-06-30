// ========================================
// Main.cpp - Updated for Your Config.h
// ========================================

/**
 * ESP32 Vehicle Tracking with Simplified Relay Success Notification
 * - Real-time GPS data transmission via WebSocket
 * - Instant relay status updates from Directus
 * - Send relay command success status back to vehicle table only
 * - Adaptive monitoring based on movement
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

// ----- MOVEMENT DETECTION -----
enum MovementState {
  MOVEMENT_UNKNOWN,
  MOVEMENT_STATIC,
  MOVEMENT_MOVING
};

// ----- RELAY COMMAND TRACKING (SIMPLIFIED) -----
struct RelayCommand {
  String vehicleId;
  String commandType;
  bool isWaitingForConfirmation;
  unsigned long commandTime;
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
bool relayState = true; // Default relay state is ON

// ----- SIMPLIFIED RELAY COMMAND TRACKING -----
RelayCommand pendingCommand = {"", "", false, 0};

// ----- MOVEMENT DETECTION VARIABLES -----
MovementState currentMovementState = MOVEMENT_UNKNOWN;
float speedSamples[MOVEMENT_DETECTION_SAMPLES];
int speedSampleIndex = 0;
bool speedSamplesReady = false;
unsigned long lastMovementLogTime = 0;

// Track consecutive connection failures
int consecutiveConnectionFailures = 0;

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
bool sendRelayCommandStatus(const String& status);
String getCurrentVehicleId();
String getCurrentTimestamp();

// Movement detection functions
void updateMovementDetection();
MovementState detectMovementState();
float getAverageSpeed();
unsigned long getCurrentGpsInterval();
const char* getMovementStateString(MovementState state);

// ----- WEBSOCKET CALLBACK (SIMPLIFIED) -----
void onRelayUpdate(bool newState) {
  LOG_INFO(MODULE_RELAY, "🔄 Relay command received: %s → %s", 
           relayState ? "ON" : "OFF", newState ? "ON" : "OFF");
  
  // Set waiting for confirmation BEFORE updating relay
  pendingCommand.vehicleId = getCurrentVehicleId();
  pendingCommand.commandType = newState ? "RELAY_ON" : "RELAY_OFF";
  pendingCommand.isWaitingForConfirmation = true;
  pendingCommand.commandTime = millis();
  
  // Update physical relay
  bool oldState = relayState;
  digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
  relayState = newState;
  
  LOG_INFO(MODULE_RELAY, "✅ Physical relay updated: %s → %s", 
           oldState ? "ON" : "OFF", newState ? "ON" : "OFF");
  
  // Small delay to ensure relay is properly set
  delay(100);
  
  // Verify relay state matches command
  bool actualRelayState = digitalRead(RELAY_PIN) == RELAY_ON;
  if (actualRelayState == newState) {
    LOG_INFO(MODULE_RELAY, "🎯 Relay state verified successfully");
    
    // Send success status to server
    if (sendRelayCommandStatus("success")) {
      LOG_INFO(MODULE_RELAY, "📤 Relay success status sent to server");
      pendingCommand.isWaitingForConfirmation = false;
    } else {
      LOG_ERROR(MODULE_RELAY, "❌ Failed to send relay success status");
      // Keep waiting for confirmation, will retry later
    }
  } else {
    LOG_ERROR(MODULE_RELAY, "❌ Relay state verification failed!");
    
    // Send failure status to server
    if (sendRelayCommandStatus("failed")) {
      LOG_INFO(MODULE_RELAY, "📤 Relay failure status sent to server");
      pendingCommand.isWaitingForConfirmation = false;
    }
  }
}

// ----- GET CURRENT VEHICLE ID FROM GPS_ID -----
String getCurrentVehicleId() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No GPRS connection for vehicle ID lookup");
    return "";
  }
  
  // Build URL using your config
  String url = "https://" + String(SERVER_HOST) + String(VEHICLE_ENDPOINT) + 
               "?filter[gps_id][_eq]=" + String(GPS_ID);
  String response;
  
  LOG_DEBUG(MODULE_RELAY, "Looking up vehicle ID for GPS_ID: %s", GPS_ID);
  
  int statusCode = httpClient.get(url, response);
  
  if (statusCode == 200) {
    // Parse JSON response
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error && doc["data"].is<JsonArray>()) {
      JsonArray vehicles = doc["data"];
      if (vehicles.size() > 0) {
        JsonObject vehicle = vehicles[0];
        int vehicleId = vehicle["vehicle_id"] | 0;
        if (vehicleId > 0) {
          LOG_DEBUG(MODULE_RELAY, "Found vehicle_id: %d for GPS_ID: %s", vehicleId, GPS_ID);
          return String(vehicleId);
        }
      }
    }
    
    LOG_ERROR(MODULE_RELAY, "Vehicle not found for GPS_ID: %s", GPS_ID);
  } else {
    LOG_ERROR(MODULE_RELAY, "Failed to lookup vehicle ID: HTTP %d", statusCode);
  }
  
  return "";
}

// ----- SEND RELAY COMMAND STATUS TO VEHICLE TABLE -----
bool sendRelayCommandStatus(const String& status) {
  LOG_INFO(MODULE_RELAY, "📡 Sending relay command status: %s", status.c_str());
  
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No GPRS connection for status update");
    return false;
  }
  
  String vehicleId = getCurrentVehicleId();
  if (vehicleId.isEmpty()) {
    LOG_ERROR(MODULE_RELAY, "Cannot get vehicle ID for status update");
    return false;
  }
  
  // Prepare JSON payload
  DynamicJsonDocument doc(256);
  doc["relay_command_status"] = status;
  doc["update_at"] = getCurrentTimestamp();
  
  String payload;
  serializeJson(doc, payload);
  
  // Build URL using your config
  String url = "https://" + String(SERVER_HOST) + String(VEHICLE_ENDPOINT) + "/" + vehicleId;
  
  LOG_DEBUG(MODULE_RELAY, "PATCH URL: %s", url.c_str());
  LOG_DEBUG(MODULE_RELAY, "Payload: %s", payload.c_str());
  
  // Send PATCH request
  String response;
  int statusCode = httpClient.patch(url, payload, response, "application/json");
  
  if (statusCode == 200) {
    LOG_INFO(MODULE_RELAY, "✅ Vehicle %s relay_command_status updated to %s", 
             vehicleId.c_str(), status.c_str());
    return true;
  } else {
    LOG_ERROR(MODULE_RELAY, "❌ Failed to update vehicle status: HTTP %d", statusCode);
    LOG_DEBUG(MODULE_RELAY, "Response: %s", response.c_str());
    return false;
  }
}

// ----- GET CURRENT TIMESTAMP -----
String getCurrentTimestamp() {
  // Use current system time with UTC offset from config
  unsigned long currentTime = millis();
  
  // Simple timestamp based on uptime (for demo purposes)
  // In production, you should sync with NTP server
  unsigned long seconds = currentTime / 1000;
  int hour = (seconds / 3600 + UTC_OFFSET) % 24;
  int minute = (seconds / 60) % 60;
  int sec = seconds % 60;
  
  char timestamp[30];
  snprintf(timestamp, sizeof(timestamp), "2024-06-21T%02d:%02d:%02d.000Z", hour, minute, sec);
  
  return String(timestamp);
}

// ----- SETUP -----
void setup() {
  // Initialize serial monitor
  SerialMon.begin(115200);
  delay(100);
  
  // Initialize logging
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 Vehicle Tracking System ===");
  LOG_INFO(MODULE_MAIN, "Version: 3.3 (Updated for Your Config)");
  LOG_INFO(MODULE_MAIN, "WebSocket URL: %s", WS_URL);
  LOG_INFO(MODULE_MAIN, "GPS ID: %s", GPS_ID);
  LOG_INFO(MODULE_MAIN, "Server Host: %s", SERVER_HOST);
  LOG_INFO(MODULE_MAIN, "Vehicle Endpoint: %s", VEHICLE_ENDPOINT);
  
  // Initialize movement detection arrays
  for (int i = 0; i < MOVEMENT_DETECTION_SAMPLES; i++) {
    speedSamples[i] = 0.0;
  }
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  // Initialize hardware - using your pin configuration
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON); // Default relay state is ON (HIGH in your config)
  
  LOG_INFO(MODULE_RELAY, "Relay initialized on pin %d (ON=%s, OFF=%s)", 
           RELAY_PIN, RELAY_ON ? "HIGH" : "LOW", RELAY_OFF ? "HIGH" : "LOW");
  
  // Initialize managers
  gpsManager.begin();
  modemManager.begin();
  
  // Initialize WebSocket manager
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  // Print help
  printHelp();
  
  // Set initial state
  currentState = STATE_INIT;
  lastSuccessfulOperation = millis();
}

// ----- MAIN LOOP -----
void loop() {
  Utils::feedWatchdog();
  
  // Update GPS
  gpsManager.update();
  
  // Update movement detection
  updateMovementDetection();
  
  // Update WebSocket connection
  if (currentState == STATE_OPERATIONAL) {
    wsManager.update();
  }
  
  // Check for pending relay command confirmation timeout
  if (pendingCommand.isWaitingForConfirmation) {
    unsigned long timeSinceCommand = millis() - pendingCommand.commandTime;
    if (timeSinceCommand > 10000) { // 10 second timeout
      LOG_WARN(MODULE_RELAY, "⏰ Relay command confirmation timeout, retrying...");
      
      // Retry sending status
      if (sendRelayCommandStatus("success")) {
        pendingCommand.isWaitingForConfirmation = false;
        LOG_INFO(MODULE_RELAY, "✅ Relay status sent on retry");
      } else {
        // Reset pending command to avoid infinite retry
        pendingCommand.isWaitingForConfirmation = false;
        LOG_ERROR(MODULE_RELAY, "❌ Failed to send relay status after retry");
      }
    }
  }
  
  // Handle serial commands
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
  
  delay(10);
}

// ----- STATE HANDLERS -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "Initializing system...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "System initialization successful");
    
    // Connect to WebSocket
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

void handleOperationalState() {
  unsigned long currentTime = millis();
  unsigned long currentGpsInterval = getCurrentGpsInterval();
  
  // Check if time to send vehicle data
  if (currentTime - lastGpsSendTime >= currentGpsInterval || gpsManager.hasNewFix()) {
    bool sendReason = (currentTime - lastGpsSendTime >= currentGpsInterval) ? false : true;
    
    if (sendVehicleDataViaWebSocket()) {
      if (sendReason) {
        LOG_DEBUG(MODULE_GPS, "✨ Data sent due to new GPS fix");
      } else {
        LOG_DEBUG(MODULE_GPS, "⏰ Data sent due to %s interval (%lums)", 
                  getMovementStateString(currentMovementState), currentGpsInterval);
      }
      lastGpsSendTime = currentTime;
      lastSuccessfulOperation = currentTime;
    }
  }
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
    // Reset complete
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
  
  // Disconnect WebSocket
  wsManager.disconnect();
  
  // Simple recovery: disconnect and reconnect GPRS
  modemManager.disconnectGprs();
  Utils::safeDelay(2000);
  
  if (modemManager.connectGprs()) {
    LOG_INFO(MODULE_SYS, "GPRS recovery successful");
    
    // Reconnect WebSocket
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

// ----- VEHICLE DATA VIA WEBSOCKET -----
bool sendVehicleDataViaWebSocket() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No GPRS connection");
    return false;
  }
  
  if (!wsManager.isReady()) {
    LOG_WARN(MODULE_GPS, "WebSocket not ready, attempting to connect...");
    wsManager.connect();
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "Sending vehicle data via WebSocket... [%s mode, interval: %lums]", 
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
  
  // Send data
  bool success = false;
  if (gpsManager.isValid()) {
    success = wsManager.sendVehicleData(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      gpsManager.isSpeedValid() ? gpsManager.getSpeed() : 0.0,
      gpsManager.getSatellites(),
      isoTimestamp
    );
    
    if (success) {
      LOG_INFO(MODULE_GPS, "✅ Vehicle data sent successfully via WebSocket");
      LOG_INFO(MODULE_GPS, "📍 GPS: lat=%.6f, lon=%.6f, speed=%.1f km/h, sats=%d",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               gpsManager.getSpeed(), gpsManager.getSatellites());
    }
  } else {
    LOG_WARN(MODULE_GPS, "GPS data not valid");
  }
  
  return success;
}

// ----- MOVEMENT DETECTION -----
void updateMovementDetection() {
  if (!gpsManager.isValid() || !gpsManager.isSpeedValid()) {
    return;
  }
  
  speedSamples[speedSampleIndex] = gpsManager.getSpeed();
  speedSampleIndex = (speedSampleIndex + 1) % MOVEMENT_DETECTION_SAMPLES;
  
  if (speedSampleIndex == 0 && !speedSamplesReady) {
    speedSamplesReady = true;
    LOG_INFO(MODULE_GPS, "Movement detection samples ready");
  }
  
  if (speedSamplesReady) {
    MovementState newState = detectMovementState();
    
    if (newState != currentMovementState) {
      LOG_INFO(MODULE_GPS, "🚗 Movement state changed: %s → %s (avg speed: %.1f km/h)", 
               getMovementStateString(currentMovementState),
               getMovementStateString(newState),
               getAverageSpeed());
      currentMovementState = newState;
      lastMovementLogTime = millis();
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastMovementLogTime >= 30000) {
      LOG_DEBUG(MODULE_GPS, "📊 Movement: %s, Speed: %.1f km/h, GPS Interval: %lums", 
                getMovementStateString(currentMovementState),
                getAverageSpeed(),
                getCurrentGpsInterval());
      lastMovementLogTime = currentTime;
    }
  }
}

MovementState detectMovementState() {
  if (!speedSamplesReady) {
    return MOVEMENT_UNKNOWN;
  }
  
  float avgSpeed = getAverageSpeed();
  
  if (currentMovementState == MOVEMENT_MOVING) {
    return (avgSpeed < (MOVEMENT_SPEED_THRESHOLD - 1.0)) ? MOVEMENT_STATIC : MOVEMENT_MOVING;
  } else {
    return (avgSpeed >= MOVEMENT_SPEED_THRESHOLD) ? MOVEMENT_MOVING : MOVEMENT_STATIC;
  }
}

float getAverageSpeed() {
  if (!speedSamplesReady) {
    return 0.0;
  }
  
  float sum = 0.0;
  for (int i = 0; i < MOVEMENT_DETECTION_SAMPLES; i++) {
    sum += speedSamples[i];
  }
  return sum / MOVEMENT_DETECTION_SAMPLES;
}

unsigned long getCurrentGpsInterval() {
  switch (currentMovementState) {
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
    case MOVEMENT_UNKNOWN: return "UNKNOWN";
    default: return "INVALID";
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
    } else if (cmd == "connect") {
      wsManager.connect();
    } else if (cmd == "disconnect") {
      wsManager.disconnect();
    } else if (cmd == "test") {
      // Test relay command status sending
      LOG_INFO(MODULE_MAIN, "Testing relay command status...");
      sendRelayCommandStatus("success");
    } else if (cmd == "vehicle") {
      // Test vehicle ID lookup
      LOG_INFO(MODULE_MAIN, "Testing vehicle ID lookup...");
      String vehicleId = getCurrentVehicleId();
      LOG_INFO(MODULE_MAIN, "Vehicle ID: %s", vehicleId.c_str());
    } else if (cmd == "relay") {
      // Test relay state
      bool currentState = digitalRead(RELAY_PIN) == RELAY_ON;
      LOG_INFO(MODULE_MAIN, "Current relay state: %s (Pin %d = %s)", 
               currentState ? "ON" : "OFF", RELAY_PIN, digitalRead(RELAY_PIN) ? "HIGH" : "LOW");
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
  
  // GPS Status
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f) Speed: %.1f km/h, Sats: %d",
             gpsManager.getLatitude(), gpsManager.getLongitude(),
             gpsManager.getSpeed(), gpsManager.getSatellites());
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix");
  }
  
  // Movement Status
  LOG_INFO(MODULE_MAIN, "Movement: %s (avg: %.1f km/h, interval: %lums)", 
           getMovementStateString(currentMovementState), 
           getAverageSpeed(), 
           getCurrentGpsInterval());
  
  // Relay Status with pin state
  bool physicalState = digitalRead(RELAY_PIN) == RELAY_ON;
  LOG_INFO(MODULE_MAIN, "Relay: %s (Physical: %s, Pin %d = %s)", 
           relayState ? "ON" : "OFF",
           physicalState ? "ON" : "OFF",
           RELAY_PIN, 
           digitalRead(RELAY_PIN) ? "HIGH" : "LOW");
  
  // Pending Command Status
  if (pendingCommand.isWaitingForConfirmation) {
    LOG_INFO(MODULE_MAIN, "Pending Command: Vehicle=%s, Type=%s", 
             pendingCommand.vehicleId.c_str(), 
             pendingCommand.commandType.c_str());
  } else {
    LOG_INFO(MODULE_MAIN, "Pending Command: None");
  }
  
  // Modem Status
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
  LOG_INFO(MODULE_MAIN, "on/off     - Manual relay control");
  LOG_INFO(MODULE_MAIN, "gps        - Send GPS data now");
  LOG_INFO(MODULE_MAIN, "ws         - Show WebSocket status");
  LOG_INFO(MODULE_MAIN, "connect    - Connect WebSocket");
  LOG_INFO(MODULE_MAIN, "disconnect - Disconnect WebSocket");
  LOG_INFO(MODULE_MAIN, "test       - Test relay status notification");
  LOG_INFO(MODULE_MAIN, "vehicle    - Test vehicle ID lookup");
  LOG_INFO(MODULE_MAIN, "relay      - Show current relay pin state");
  LOG_INFO(MODULE_MAIN, "reset      - Restart system");
}