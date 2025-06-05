/**
 * Integrated ESP32 System with A7670C GSM Module - Vehicle Endpoint Relay Control
 * - Direct relay status checking from /items/vehicle endpoint
 * - Adaptive monitoring: Normal (5s) → Active (2s) → Real-time (1s)
 * - IMMEDIATE relay updates (consensus removed)
 * - Hybrid GPS intervals based on movement
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

// ----- RELAY MONITORING MODES -----
enum RelayMonitoringMode {
  RELAY_MODE_NORMAL,    // 5 seconds - normal operation
  RELAY_MODE_ACTIVE,    // 2 seconds - after status change detected
  RELAY_MODE_REALTIME   // 1 second - for manual testing or immediate response
};

// ----- GLOBAL OBJECTS -----
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
GpsManager gpsManager(gps, SerialGPS);
ModemManager modemManager(modem, SerialAT);
HttpClientWrapper httpClient(gsmClient);

// ----- STATE VARIABLES -----
SystemState currentState = STATE_INIT;
unsigned long lastGpsSendTime = 0;
unsigned long lastVehicleCheckTime = 0;
unsigned long lastSuccessfulOperation = 0;
bool relayState = true; // Default relay state is ON

// ----- MOVEMENT DETECTION VARIABLES -----
MovementState currentMovementState = MOVEMENT_UNKNOWN;
float speedSamples[MOVEMENT_DETECTION_SAMPLES];
int speedSampleIndex = 0;
bool speedSamplesReady = false;
unsigned long lastMovementLogTime = 0;

// ----- VEHICLE ENDPOINT RELAY MONITORING VARIABLES -----
RelayMonitoringMode currentRelayMode = RELAY_MODE_NORMAL;
unsigned long relayModeStartTime = 0;
unsigned long lastRelayModeLogTime = 0;
int consecutiveRelayFailures = 0;
unsigned long lastStatusChangeTime = 0;

// Simplified variables - consensus-based variables removed
String lastApiRelayStatus = "";
bool relayStatusStable = true;

// REMOVED VARIABLES (no longer needed for immediate updates):
// String pendingRelayStatus = "";
// int consecutiveStatusCount = 0;


// ----- FUNCTION PROTOTYPES -----
void handleInitState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSerialCommands();
void printStatus();
void printHelp();
bool sendVehicleData();
bool testServerConnectivity();
bool updateRelayStatusManually(bool state);

// Movement detection functions
void updateMovementDetection();
MovementState detectMovementState();
float getAverageSpeed();
unsigned long getCurrentGpsInterval();
const char* getMovementStateString(MovementState state);

// Vehicle endpoint relay monitoring functions
void updateRelayMonitoring();
bool checkVehicleRelayStatus();
void processRelayStatusChange(String newStatus);
void switchRelayMode(RelayMonitoringMode newMode);
unsigned long getCurrentRelayInterval();
const char* getRelayModeString(RelayMonitoringMode mode);
bool updatePhysicalRelay(bool newState, String reason);

// ----- SETUP -----
void setup() {
  // Initialize serial monitor
  SerialMon.begin(115200);
  delay(100);
  
  // Initialize logging
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 System Starting ===");
  LOG_INFO(MODULE_MAIN, "Version: 2.4 (Vehicle Endpoint Relay Control - Immediate Update)"); // Version updated
  LOG_INFO(MODULE_MAIN, "GPS Intervals: Moving=%dms, Static=%dms", 
           GPS_SEND_INTERVAL_MOVING, GPS_SEND_INTERVAL_STATIC);
  LOG_INFO(MODULE_MAIN, "Relay Intervals: Normal=%dms, Active=%dms, Real-time=%dms", 
           RELAY_CHECK_INTERVAL_NORMAL, RELAY_CHECK_INTERVAL_ACTIVE, RELAY_CHECK_INTERVAL_REALTIME);
  LOG_INFO(MODULE_MAIN, "Vehicle endpoint: %s", VEHICLE_ENDPOINT);
  
  // Initialize movement detection arrays
  for (int i = 0; i < MOVEMENT_DETECTION_SAMPLES; i++) {
    speedSamples[i] = 0.0;
  }
  
  // Initialize relay monitoring (in setup() function)
  relayModeStartTime = millis();
  lastApiRelayStatus = "UNKNOWN";
  relayStatusStable = true;  // Simplified initialization
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  // Initialize hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON); // Default relay state is ON
  
  // Initialize managers
  gpsManager.begin();
  modemManager.begin();
  
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
  
  // Update relay monitoring
  updateRelayMonitoring();
  
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

// ----- VEHICLE ENDPOINT RELAY MONITORING FUNCTIONS -----
void updateRelayMonitoring() {
  unsigned long currentTime = millis();
  
  // Handle relay mode transitions based on time since last status change
  unsigned long timeSinceLastChange = currentTime - lastStatusChangeTime;
  
  switch (currentRelayMode) {
    case RELAY_MODE_REALTIME:
      // Switch to active mode after real-time duration
      if (timeSinceLastChange >= REALTIME_MODE_DURATION) {
        switchRelayMode(RELAY_MODE_ACTIVE);
      }
      break;
      
    case RELAY_MODE_ACTIVE:
      // Switch to normal mode after active duration
      if (timeSinceLastChange >= ACTIVE_MODE_DURATION) {
        switchRelayMode(RELAY_MODE_NORMAL);
      }
      break;
      
    case RELAY_MODE_NORMAL:
      // Stay in normal mode unless manual override
      break;
  }
  
  // Fallback to normal mode after consecutive failures
  if (consecutiveRelayFailures >= MAX_CONSECUTIVE_RELAY_FAILURES) {
    if (currentRelayMode != RELAY_MODE_NORMAL) {
      LOG_WARN(MODULE_VEHICLE, "Too many failures, falling back to normal mode");
      switchRelayMode(RELAY_MODE_NORMAL);
    }
    consecutiveRelayFailures = 0; // Reset counter
  }
  
  // Log relay mode status periodically
  if (currentTime - lastRelayModeLogTime >= 60000) { // Every minute
    LOG_INFO(MODULE_VEHICLE, "📡 Vehicle endpoint monitoring: %s mode (interval: %lums)", 
             getRelayModeString(currentRelayMode), getCurrentRelayInterval());
    LOG_INFO(MODULE_VEHICLE, "📊 Last API status: %s, Physical relay: %s, Failures: %d", 
             lastApiRelayStatus.c_str(), relayState ? "ON" : "OFF", consecutiveRelayFailures);
    lastRelayModeLogTime = currentTime;
  }
}

bool checkVehicleRelayStatus() {
  if (!modemManager.ensureConnection()) {
    consecutiveRelayFailures++;
    LOG_DEBUG(MODULE_VEHICLE, "No connection for vehicle relay status check (failures: %d)", 
              consecutiveRelayFailures);
    return false;
  }
  
  LOG_DEBUG(MODULE_VEHICLE, "🔍 Checking vehicle relay status... [%s mode]", 
            getRelayModeString(currentRelayMode));
  
  String response;
  bool success = httpClient.get(VEHICLE_ENDPOINT, response);
  
  if (!success) {
    consecutiveRelayFailures++;
    LOG_ERROR(MODULE_VEHICLE, "❌ Failed to get vehicle data from API (failures: %d)", 
              consecutiveRelayFailures);
    return false;
  }
  
  // Reset failure counter on successful HTTP request
  consecutiveRelayFailures = 0;
  
  // Parse response
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, response);
  
  if (error) {
    LOG_ERROR(MODULE_VEHICLE, "JSON parsing failed: %s", error.c_str());
    return false;
  }
  
  // Check Directus API response format
  if (!doc.containsKey("data") || !doc["data"].is<JsonArray>()) {
    LOG_ERROR(MODULE_VEHICLE, "Invalid vehicle response format");
    LOG_DEBUG(MODULE_VEHICLE, "Response: %s", response.c_str());
    return false;
  }
  
  JsonArray vehicles = doc["data"].as<JsonArray>();
  
  if (vehicles.size() == 0) {
    LOG_WARN(MODULE_VEHICLE, "No vehicles found in API response");
    return false;
  }
  
  // Find vehicle with matching gps_id
  bool vehicleFound = false;
  String currentApiRelayStatus = "";
  
  for (JsonObject vehicle : vehicles) {
    if (vehicle.containsKey("gps_id") &&
        vehicle["gps_id"].as<String>() == String(GPS_ID)) {
      vehicleFound = true;
      
      if (vehicle.containsKey("relay_status") && !vehicle["relay_status"].isNull()) {
        currentApiRelayStatus = vehicle["relay_status"].as<String>();
      } else {
        currentApiRelayStatus = "NULL";
      }
      break;
    }
  }
  
  if (!vehicleFound) {
    LOG_WARN(MODULE_VEHICLE, "GPS ID %s not found in API response", GPS_ID);
    return false;
  }
  
  // Process the relay status
  processRelayStatusChange(currentApiRelayStatus);
  
  return true;
}

void processRelayStatusChange(String newApiStatus) {
  // Log the API status check
  if (currentRelayMode == RELAY_MODE_REALTIME) {
    LOG_INFO(MODULE_VEHICLE, "🚀 API relay status: %s [REAL-TIME CHECK]", newApiStatus.c_str());
  } else {
    LOG_DEBUG(MODULE_VEHICLE, "📡 API relay status: %s", newApiStatus.c_str());
  }
  
  // Handle NULL or empty status
  if (newApiStatus == "NULL" || newApiStatus.isEmpty()) {
    LOG_DEBUG(MODULE_VEHICLE, "Relay status is null/empty, keeping current state");
    return;
  }
  
  // Convert API status to boolean
  bool newRelayState = (newApiStatus == "ON");
  
  // Check if this is different from current physical relay state
  if (newRelayState != relayState) {
    // IMMEDIATE UPDATE - No consensus needed
    LOG_INFO(MODULE_VEHICLE, "🔄 API status change detected: %s → %s", 
             relayState ? "ON" : "OFF", newApiStatus.c_str());
    
    String reason = "Immediate API response";
    updatePhysicalRelay(newRelayState, reason);
    lastStatusChangeTime = millis();
    
    // Switch to real-time mode for immediate verification
    switchRelayMode(RELAY_MODE_REALTIME);
    
    // Update tracking variables
    lastApiRelayStatus = newApiStatus;
    relayStatusStable = true;
    // consecutiveStatusCount = 0; // Removed
    // pendingRelayStatus = "";    // Removed
    
    LOG_INFO(MODULE_VEHICLE, "✅ Relay updated immediately - Physical: %s, API: %s", 
             relayState ? "ON" : "OFF", newApiStatus.c_str());
  } else {
    // Status matches current state
    if (newApiStatus != lastApiRelayStatus) {
      LOG_DEBUG(MODULE_VEHICLE, "📡 API status confirmed: %s (matches physical relay)", newApiStatus.c_str());
      lastApiRelayStatus = newApiStatus;
    }
    
    // Everything is in sync
    if (currentRelayMode == RELAY_MODE_REALTIME) {
      LOG_DEBUG(MODULE_VEHICLE, "✅ Relay status verified: %s [REAL-TIME]", newApiStatus.c_str());
    }
    
    relayStatusStable = true;
    // consecutiveStatusCount = 0; // Removed
    // pendingRelayStatus = "";    // Removed
  }
}

bool updatePhysicalRelay(bool newState, String reason) {
  LOG_INFO(MODULE_VEHICLE, "🔌 Updating physical relay: %s → %s (%s)", 
           relayState ? "ON" : "OFF", newState ? "ON" : "OFF", reason.c_str());
  
  // Update physical pin
  digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
  
  // Update local state
  relayState = newState;
  
  LOG_INFO(MODULE_VEHICLE, "✅ Physical relay updated successfully to: %s", newState ? "ON" : "OFF");
  
  return true;
}

void switchRelayMode(RelayMonitoringMode newMode) {
  if (newMode == currentRelayMode) {
    return; // No change needed
  }
  
  RelayMonitoringMode oldMode = currentRelayMode;
  unsigned long oldInterval = getCurrentRelayInterval();
  
  currentRelayMode = newMode;
  relayModeStartTime = millis();
  
  unsigned long newInterval = getCurrentRelayInterval();
  
  LOG_INFO(MODULE_VEHICLE, "🔄 Relay monitoring mode: %s → %s (interval: %lu → %lums)", 
           getRelayModeString(oldMode), getRelayModeString(newMode),
           oldInterval, newInterval);
}

unsigned long getCurrentRelayInterval() {
  switch (currentRelayMode) {
    case RELAY_MODE_REALTIME: return RELAY_CHECK_INTERVAL_REALTIME;
    case RELAY_MODE_ACTIVE: return RELAY_CHECK_INTERVAL_ACTIVE;
    case RELAY_MODE_NORMAL: 
    default: return RELAY_CHECK_INTERVAL_NORMAL;
  }
}

const char* getRelayModeString(RelayMonitoringMode mode) {
  switch (mode) {
    case RELAY_MODE_REALTIME: return "REAL-TIME";
    case RELAY_MODE_ACTIVE: return "ACTIVE";
    case RELAY_MODE_NORMAL: return "NORMAL";
    default: return "UNKNOWN";
  }
}

// ----- MOVEMENT DETECTION FUNCTIONS -----
void updateMovementDetection() {
  if (!gpsManager.isValid() || !gpsManager.isSpeedValid()) {
    return; // Skip if GPS data is not valid
  }
  
  // Add current speed to samples array
  speedSamples[speedSampleIndex] = gpsManager.getSpeed();
  speedSampleIndex = (speedSampleIndex + 1) % MOVEMENT_DETECTION_SAMPLES;
  
  // Mark samples as ready when we have filled the array at least once
  if (speedSampleIndex == 0 && !speedSamplesReady) {
    speedSamplesReady = true;
    LOG_INFO(MODULE_GPS, "Movement detection samples ready");
  }
  
  if (speedSamplesReady) {
    MovementState newState = detectMovementState();
    
    // Log state changes
    if (newState != currentMovementState) {
      LOG_INFO(MODULE_GPS, "🚗 Movement state changed: %s → %s (avg speed: %.1f km/h)", 
               getMovementStateString(currentMovementState),
               getMovementStateString(newState),
               getAverageSpeed());
      currentMovementState = newState;
      lastMovementLogTime = millis();
    }
    
    // Periodic logging (every 30 seconds)
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
  
  // Use hysteresis to prevent rapid state changes
  if (currentMovementState == MOVEMENT_MOVING) {
    // If currently moving, need speed to drop below threshold - 1 to become static
    return (avgSpeed < (MOVEMENT_SPEED_THRESHOLD - 1.0)) ? MOVEMENT_STATIC : MOVEMENT_MOVING;
  } else {
    // If currently static or unknown, need speed above threshold to become moving
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
      return GPS_SEND_INTERVAL_MOVING; // Default to shorter interval when unknown
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

// ----- STATE HANDLERS -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "Initializing system...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "System initialization successful");
    
    // Test server connectivity
    if (testServerConnectivity()) {
      LOG_INFO(MODULE_SYS, "Server connectivity verified");
    } else {
      LOG_WARN(MODULE_SYS, "Server connectivity test failed, but continuing...");
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
  unsigned long currentRelayInterval = getCurrentRelayInterval();
  
  // Check if time to send vehicle data (using dynamic interval)
  if (currentTime - lastGpsSendTime >= currentGpsInterval || gpsManager.hasNewFix()) {
    bool sendReason = (currentTime - lastGpsSendTime >= currentGpsInterval) ? false : true;
    
    if (sendVehicleData()) {
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
  
  // Check if time to check vehicle relay status (using adaptive interval)
  if (currentTime - lastVehicleCheckTime >= currentRelayInterval) {
    if (checkVehicleRelayStatus()) {
      lastVehicleCheckTime = currentTime;
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
  
  // Simple recovery: disconnect and reconnect
  modemManager.disconnectGprs();
  Utils::safeDelay(2000);
  
  if (modemManager.connectGprs()) {
    LOG_INFO(MODULE_SYS, "Connection recovery successful");
    currentState = STATE_OPERATIONAL;
    lastSuccessfulOperation = millis(); // Reset the timeout counter
  } else {
    LOG_ERROR(MODULE_SYS, "Connection recovery failed, trying modem reset");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

// ----- VEHICLE DATA OPERATIONS -----
bool sendVehicleData() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No connection for vehicle data");
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "Sending vehicle data... [%s mode, interval: %lums]", 
           getMovementStateString(currentMovementState), getCurrentGpsInterval());
  
  // Prepare timestamp in ISO format (UTC)
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  
  // Convert timestamp to the required format (2025-05-28T04:42:26.000Z)
  String isoTimestamp = String(timestamp);
  if (isoTimestamp.endsWith("Z")) {
    // Already in UTC format, just add milliseconds
    isoTimestamp.replace("Z", ".000Z");
  } else {
    // Fallback: replace any timezone with .000Z
    int tzIndex = isoTimestamp.lastIndexOf('+');
    if (tzIndex == -1) tzIndex = isoTimestamp.lastIndexOf('-', isoTimestamp.length() - 6);
    if (tzIndex > 0) {
      isoTimestamp = isoTimestamp.substring(0, tzIndex) + ".000Z";
    } else {
      isoTimestamp += ".000Z";
    }
  }
  
  // Create JSON payload in the required format for vehicle_datas
  DynamicJsonDocument doc(1024);
  
  // GPS data
  if (gpsManager.isValid()) {
    doc["latitude"] = String(gpsManager.getLatitude(), 5);  // 5 decimal places
    doc["longitude"] = String(gpsManager.getLongitude(), 5); // 5 decimal places
    doc["satellites_used"] = gpsManager.getSatellites();
    
    // Add speed data from GPS
    if (gpsManager.isSpeedValid()) {
      doc["speed"] = static_cast<int>(gpsManager.getSpeed()); // Convert to int as in NEW SKETCH
    } else {
      doc["speed"] = nullptr;
    }
  } else {
    doc["latitude"] = nullptr;
    doc["longitude"] = nullptr;
    doc["satellites_used"] = nullptr;
    doc["speed"] = nullptr;
  }
  
  // Fields to be left empty as requested
  doc["rpm"] = nullptr;
  doc["fuel_level"] = nullptr;
  doc["ignition_status"] = nullptr;
  
  // Battery level as specified
  doc["battery_level"] = 12.5;
  
  // Timestamp
  doc["timestamp"] = isoTimestamp;
  
  // GPS ID at the end as requested
  doc["gps_id"] = GPS_ID;  // UUID from config
  
  String payload;
  serializeJson(doc, payload);
  
  LOG_DEBUG(MODULE_GPS, "Vehicle data payload: %s", payload.c_str());
  
  // Send with retry
  bool success = Utils::retryOperation(
    MODULE_GPS,
    "Vehicle data sending",
    [&]() {
      String response;
      bool result = httpClient.post(VEHICLE_DATA_ENDPOINT, payload, response);
      if (result) {
        LOG_INFO(MODULE_GPS, "✅ Vehicle data sent successfully to server");
        LOG_INFO(MODULE_GPS, "📊 Data sent: gps_id=%s, timestamp=%s", GPS_ID, isoTimestamp.c_str());
        if (gpsManager.isValid()) {
          if (gpsManager.isSpeedValid()) {
            LOG_INFO(MODULE_GPS, "📍 GPS: lat=%.6f, lon=%.6f, speed=%.1f km/h (%s), sats=%d",
                     gpsManager.getLatitude(), gpsManager.getLongitude(),
                     gpsManager.getSpeed(), getMovementStateString(currentMovementState), 
                     gpsManager.getSatellites());
          } else {
            LOG_INFO(MODULE_GPS, "📍 GPS: lat=%.6f, lon=%.6f, speed=N/A, sats=%d",
                     gpsManager.getLatitude(), gpsManager.getLongitude(), gpsManager.getSatellites());
          }
        } else {
          LOG_INFO(MODULE_GPS, "📍 GPS: No valid fix");
        }
        LOG_DEBUG(MODULE_GPS, "Server response: %s", response.c_str());
      }
      return result;
    },
    MAX_HTTP_RETRIES
  );
  
  if (!success) {
    LOG_ERROR(MODULE_GPS, "Failed to send vehicle data after retries");
  }
  
  return success;
}

// ----- SERVER CONNECTIVITY TEST -----
bool testServerConnectivity() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_HTTP, "No connection for server test");
    return false;
  }
  
  LOG_INFO(MODULE_HTTP, "🔍 Testing server connectivity to %s:%d", SERVER_HOST, SERVER_PORT);
  
  // Test with the vehicle endpoint
  String response;
  bool connected = httpClient.get(VEHICLE_ENDPOINT, response);
  
  if (connected) {
    LOG_INFO(MODULE_HTTP, "✅ Server connectivity test PASSED");
    LOG_INFO(MODULE_HTTP, "✅ Vehicle endpoint (%s) is accessible", VEHICLE_ENDPOINT);
    LOG_INFO(MODULE_HTTP, "✅ Server %s:%d is ready to receive data", SERVER_HOST, SERVER_PORT);
    LOG_DEBUG(MODULE_HTTP, "Server response length: %d bytes", response.length());
  } else {
    LOG_WARN(MODULE_HTTP, "⚠️ Server connectivity test failed, but will continue");
    LOG_WARN(MODULE_HTTP, "⚠️ This might be normal if the endpoint requires specific parameters");
    LOG_INFO(MODULE_HTTP, "📡 Will attempt to check vehicle endpoint anyway");
  }
  
  return connected;
}

// ----- MANUAL RELAY OPERATIONS -----
bool updateRelayStatusManually(bool state) {
  LOG_INFO(MODULE_VEHICLE, "🖱️ Manual relay update requested: %s", state ? "ON" : "OFF");
  
  // Update physical relay immediately for manual commands
  updatePhysicalRelay(state, "Manual command");
  
  // Switch to real-time mode to verify the change quickly
  switchRelayMode(RELAY_MODE_REALTIME);
  lastStatusChangeTime = millis();
  
  LOG_INFO(MODULE_VEHICLE, "✅ Manual relay update completed, switched to real-time monitoring");
  
  return true;
}

// ----- SERIAL COMMAND HANDLER -----
void handleSerialCommands() {
  if (SerialMon.available()) {
    String cmd = SerialMon.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "help") {
      printHelp();
    } else if (cmd == "status") {
      printStatus();
    } else if (cmd == "on") {
      updateRelayStatusManually(true);
    } else if (cmd == "off") {
      updateRelayStatusManually(false);
    } else if (cmd == "gps") {
      sendVehicleData();
    } else if (cmd == "test") {
      testServerConnectivity();
    } else if (cmd == "vehicle") {
      checkVehicleRelayStatus();
    } else if (cmd == "realtime") {
      switchRelayMode(RELAY_MODE_REALTIME);
      lastStatusChangeTime = millis();
    } else if (cmd == "active") {
      switchRelayMode(RELAY_MODE_ACTIVE);
      lastStatusChangeTime = millis();
    } else if (cmd == "normal") {
      switchRelayMode(RELAY_MODE_NORMAL);
    } else if (cmd == "movement") {
      LOG_INFO(MODULE_MAIN, "=== MOVEMENT STATUS ===");
      LOG_INFO(MODULE_MAIN, "Current State: %s", getMovementStateString(currentMovementState));
      LOG_INFO(MODULE_MAIN, "Average Speed: %.1f km/h", getAverageSpeed());
      LOG_INFO(MODULE_MAIN, "Speed Threshold: %.1f km/h", MOVEMENT_SPEED_THRESHOLD);
      LOG_INFO(MODULE_MAIN, "Current GPS Interval: %lu ms", getCurrentGpsInterval());
      LOG_INFO(MODULE_MAIN, "Speed Samples Ready: %s", speedSamplesReady ? "Yes" : "No");
      if (speedSamplesReady) {
        LOG_INFO(MODULE_MAIN, "Recent speeds: %.1f, %.1f, %.1f km/h", 
                 speedSamples[0], speedSamples[1], speedSamples[2]);
      }
    } else if (cmd == "relay") {
      LOG_INFO(MODULE_MAIN, "=== VEHICLE ENDPOINT RELAY STATUS ===");
      LOG_INFO(MODULE_MAIN, "Monitoring Mode: %s", getRelayModeString(currentRelayMode));
      LOG_INFO(MODULE_MAIN, "Check Interval: %lu ms", getCurrentRelayInterval());
      LOG_INFO(MODULE_MAIN, "Physical Relay: %s", relayState ? "ON" : "OFF");
      LOG_INFO(MODULE_MAIN, "Last API Status: %s", lastApiRelayStatus.c_str());
      // LOG_INFO(MODULE_MAIN, "Pending Status: %s", pendingRelayStatus.c_str()); // Removed
      // LOG_INFO(MODULE_MAIN, "Consensus Count: %d/%d", consecutiveStatusCount, RELAY_STATUS_CHANGE_THRESHOLD); // Removed
      LOG_INFO(MODULE_MAIN, "Status Stable: %s", relayStatusStable ? "Yes" : "No");
      LOG_INFO(MODULE_MAIN, "Consecutive Failures: %d", consecutiveRelayFailures);
      unsigned long timeSinceLastChange = millis() - lastStatusChangeTime;
      LOG_INFO(MODULE_MAIN, "Time Since Last Change: %lu ms", timeSinceLastChange);
      LOG_INFO(MODULE_MAIN, "Vehicle Endpoint: %s", VEHICLE_ENDPOINT);
    } else if (cmd == "data") {
      LOG_INFO(MODULE_MAIN, "=== CURRENT VEHICLE DATA ===");
      char timestamp[30];
      gpsManager.getTimestamp(timestamp, sizeof(timestamp));
      String isoTimestamp = String(timestamp);
      if (isoTimestamp.endsWith("Z")) {
        // Already in UTC format, just add milliseconds
        isoTimestamp.replace("Z", ".000Z");
      } else {
        // Fallback: replace any timezone with .000Z
        int tzIndex = isoTimestamp.lastIndexOf('+');
        if (tzIndex == -1) tzIndex = isoTimestamp.lastIndexOf('-', isoTimestamp.length() - 6);
        if (tzIndex > 0) {
          isoTimestamp = isoTimestamp.substring(0, tzIndex) + ".000Z";
        } else {
          isoTimestamp += ".000Z";
        }
      }
      
      LOG_INFO(MODULE_MAIN, "GPS ID: %s", GPS_ID);
      LOG_INFO(MODULE_MAIN, "Timestamp: %s", isoTimestamp.c_str());
      
      if (gpsManager.isValid()) {
        if (gpsManager.isSpeedValid()) {
          LOG_INFO(MODULE_MAIN, "GPS: Valid - Lat: %.6f, Lon: %.6f, Speed: %.1f km/h, Sats: %d",
                   gpsManager.getLatitude(), gpsManager.getLongitude(),
                   gpsManager.getSpeed(), gpsManager.getSatellites());
        } else {
          LOG_INFO(MODULE_MAIN, "GPS: Valid - Lat: %.6f, Lon: %.6f, Speed: N/A, Sats: %d",
                   gpsManager.getLatitude(), gpsManager.getLongitude(), gpsManager.getSatellites());
        }
      } else {
        LOG_INFO(MODULE_MAIN, "GPS: No valid fix");
      }
      
      LOG_INFO(MODULE_MAIN, "Movement: %s (avg: %.1f km/h)", 
               getMovementStateString(currentMovementState), getAverageSpeed());
      LOG_INFO(MODULE_MAIN, "Relay Mode: %s (interval: %lums)", 
               getRelayModeString(currentRelayMode), getCurrentRelayInterval());
      LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
      LOG_INFO(MODULE_MAIN, "Battery Level: 12.5V");
      LOG_INFO(MODULE_MAIN, "Vehicle Data Endpoint: %s", VEHICLE_DATA_ENDPOINT);
      LOG_INFO(MODULE_MAIN, "Vehicle Endpoint: %s", VEHICLE_ENDPOINT);
    } else if (cmd == "reset") {
      ESP.restart();
    } else if (cmd == "loglevel") {
      LOG_INFO(MODULE_MAIN, "Log levels: 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=TRACE");
    } else if (cmd.startsWith("loglevel ")) {
      int level = cmd.substring(9).toInt();
      Logger::setLevel((LogLevel)level);
      LOG_INFO(MODULE_MAIN, "Log level set to %d", level);
    } else {
      LOG_WARN(MODULE_MAIN, "Unknown command: %s", cmd.c_str());
    }
  }
}

// ----- STATUS DISPLAY -----
void printStatus() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM STATUS ===");
  LOG_INFO(MODULE_MAIN, "State: %d", currentState);
  LOG_INFO(MODULE_MAIN, "Uptime: %s", Utils::formatUptime(millis()).c_str());
  
  // GPS Status
  if (gpsManager.isValid()) {
    if (gpsManager.isSpeedValid()) {
      LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f) Speed: %.1f km/h, Sats: %d",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               gpsManager.getSpeed(), gpsManager.getSatellites());
    } else {
      LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f) Speed: N/A, Sats: %d",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               gpsManager.getSatellites());
    }
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix");
  }
  
  // Movement Status
  LOG_INFO(MODULE_MAIN, "Movement: %s (avg: %.1f km/h, interval: %lums)", 
           getMovementStateString(currentMovementState), 
           getAverageSpeed(), 
           getCurrentGpsInterval());
  
  // Vehicle Endpoint Relay Status
  LOG_INFO(MODULE_MAIN, "Vehicle Relay: %s mode (interval: %lums, API: %s, Physical: %s)", 
           getRelayModeString(currentRelayMode), 
           getCurrentRelayInterval(),
           lastApiRelayStatus.c_str(),
           relayState ? "ON" : "OFF");
  
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
  LOG_INFO(MODULE_MAIN, "help          - Show this help");
  LOG_INFO(MODULE_MAIN, "status        - Show system status");
  LOG_INFO(MODULE_MAIN, "movement      - Show movement detection status");
  LOG_INFO(MODULE_MAIN, "relay         - Show vehicle endpoint relay status");
  LOG_INFO(MODULE_MAIN, "on/off        - Manual relay control");
  LOG_INFO(MODULE_MAIN, "gps           - Send vehicle data now");
  LOG_INFO(MODULE_MAIN, "test          - Test server connectivity");
  LOG_INFO(MODULE_MAIN, "vehicle       - Check vehicle endpoint now");
  LOG_INFO(MODULE_MAIN, "realtime      - Switch to real-time mode (1s)");
  LOG_INFO(MODULE_MAIN, "active        - Switch to active mode (2s)");
  LOG_INFO(MODULE_MAIN, "normal        - Switch to normal mode (5s)");
  LOG_INFO(MODULE_MAIN, "data          - Show current vehicle data");
  LOG_INFO(MODULE_MAIN, "reset         - Restart system");
  LOG_INFO(MODULE_MAIN, "loglevel [0-4] - Set log level");
}