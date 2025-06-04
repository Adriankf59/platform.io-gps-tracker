/**
 * Integrated ESP32 System with A7670C GSM Module - Refactored Version
 * - Modular architecture with separate components
 * - Unified logging system
 * - Helper functions to reduce redundancy
 * - Unit testing support
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
unsigned long lastRelayCheckTime = 0;
unsigned long lastSuccessfulOperation = 0;
bool relayState = false;

// ----- FUNCTION PROTOTYPES -----
void handleInitState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSerialCommands();
void printStatus();
void printHelp();
bool sendGpsData();
bool sendVehicleData();
bool testServerConnectivity();
bool checkRelayStatus();
bool updateRelayStatus(bool state);
bool updateCommandStatus(const String& commandId, const String& status);

// ----- SETUP -----
void setup() {
  // Initialize serial monitor
  SerialMon.begin(115200);
  delay(100);
  
  // Initialize logging
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 System Starting ===");
  LOG_INFO(MODULE_MAIN, "Version: 2.0 (Refactored)");
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  // Initialize hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  
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
  
  // Check if time to send vehicle data
  if (currentTime - lastGpsSendTime >= GPS_SEND_INTERVAL || gpsManager.hasNewFix()) {
    if (sendVehicleData()) {
      lastGpsSendTime = currentTime;
      lastSuccessfulOperation = currentTime;
    }
  }
  
  // Check if time to check relay
  if (currentTime - lastRelayCheckTime >= RELAY_CHECK_INTERVAL) {
    if (checkRelayStatus()) {
      lastRelayCheckTime = currentTime;
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
  } else {
    LOG_ERROR(MODULE_SYS, "Connection recovery failed, trying modem reset");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

// ----- GPS OPERATIONS -----
bool sendGpsData() {
  // This function is now replaced by sendVehicleData()
  // Redirect to the new implementation
  return sendVehicleData();
}

// ----- VEHICLE DATA OPERATIONS -----
bool sendVehicleData() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No connection for vehicle data");
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "Sending vehicle data...");
  
  // Prepare timestamp in ISO format
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  
  // Convert timestamp to the required format (2025-05-28T04:42:26.000Z)
  String isoTimestamp = String(timestamp);
  isoTimestamp.replace("+07:00", ".000Z");
  
  // Create JSON payload in the required format for vehicle_datas
  DynamicJsonDocument doc(1024);
  JsonArray dataArray = doc.createNestedArray("data");
  JsonObject vehicleData = dataArray.createNestedObject();
  
  // GPS data
  if (gpsManager.isValid()) {
    vehicleData["latitude"] = String(gpsManager.getLatitude(), 5);  // 5 decimal places
    vehicleData["longitude"] = String(gpsManager.getLongitude(), 5); // 5 decimal places
    vehicleData["satellites_used"] = gpsManager.getSatellites();
  } else {
    vehicleData["latitude"] = nullptr;
    vehicleData["longitude"] = nullptr;
    vehicleData["satellites_used"] = nullptr;
  }
  
  // Fields to be left empty as requested
  vehicleData["speed"] = nullptr;
  vehicleData["rpm"] = nullptr;
  vehicleData["fuel_level"] = nullptr;
  vehicleData["ignition_status"] = nullptr;
  
  // Battery level as specified
  vehicleData["battery_level"] = 12.5;
  
  // Timestamp
  vehicleData["timestamp"] = isoTimestamp;
  
  // GPS ID at the end as requested
  vehicleData["gps_id"] = GPS_ID;  // UUID from config
  
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
          LOG_INFO(MODULE_GPS, "📍 GPS: lat=%.6f, lon=%.6f, sats=%d",
                   gpsManager.getLatitude(), gpsManager.getLongitude(), gpsManager.getSatellites());
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
  
  // Test with the actual vehicle_datas endpoint instead of root
  String response;
  bool connected = httpClient.get(VEHICLE_DATA_ENDPOINT, response);
  
  if (connected) {
    LOG_INFO(MODULE_HTTP, "✅ Server connectivity test PASSED");
    LOG_INFO(MODULE_HTTP, "✅ Vehicle data endpoint (%s) is accessible", VEHICLE_DATA_ENDPOINT);
    LOG_INFO(MODULE_HTTP, "✅ Server %s:%d is ready to receive data", SERVER_HOST, SERVER_PORT);
    LOG_DEBUG(MODULE_HTTP, "Server response length: %d bytes", response.length());
  } else {
    LOG_WARN(MODULE_HTTP, "⚠️ Server connectivity test failed, but will continue");
    LOG_WARN(MODULE_HTTP, "⚠️ This might be normal if the endpoint requires POST requests");
    LOG_INFO(MODULE_HTTP, "📡 Will attempt to send vehicle data anyway");
  }
  
  return connected;
}

// ----- RELAY OPERATIONS -----
bool checkRelayStatus() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No connection for relay check");
    return false;
  }
  
  LOG_INFO(MODULE_RELAY, "Checking relay commands...");
  
  // Format Directus API filter query correctly
  String endpoint = String(RELAY_ENDPOINT);
  endpoint += "?filter[vehicle_id][_eq]=" + String(VEHICLE_ID);
  endpoint += "&filter[status][_eq]=pending";
  endpoint += "&sort=-date_sent"; // Sort by newest first
  
  String response;
  bool success = Utils::retryOperation(
    MODULE_RELAY,
    "Relay check",
    [&]() {
      return httpClient.get(endpoint.c_str(), response);
    },
    MAX_HTTP_RETRIES
  );
  
  if (!success) {
    LOG_ERROR(MODULE_RELAY, "Failed to get commands from API");
    return false;
  }
  
  // Parse response
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, response);
  
  if (error) {
    LOG_ERROR(MODULE_RELAY, "JSON parsing failed: %s", error.c_str());
    return false;
  }
  
  // Check Directus API response format
  if (!doc.containsKey("data") || !doc["data"].is<JsonArray>()) {
    LOG_ERROR(MODULE_RELAY, "Invalid response format");
    LOG_DEBUG(MODULE_RELAY, "Response: %s", response.c_str());
    return false;
  }
  
  JsonArray commands = doc["data"].as<JsonArray>();
  
  if (commands.size() == 0) {
    LOG_DEBUG(MODULE_RELAY, "No pending commands");
    return true;
  }
  
  // Process the first command
  JsonObject command = commands[0];
  
  String commandId = "";
  // Try different possible field names for command ID
  if (command.containsKey("command_id") && !command["command_id"].isNull()) {
    commandId = command["command_id"].as<String>();
  } else if (command.containsKey("id") && !command["id"].isNull()) {
    commandId = command["id"].as<String>();
  }
  
  String commandType = command.containsKey("command_type") ?
                      command["command_type"].as<String>() : "";
  
  LOG_INFO(MODULE_RELAY, "Received command: %s (ID: %s)",
           commandType.c_str(), commandId.c_str());
  
  // Execute command first
  bool commandExecuted = false;
  if (commandType == "ENGINE_ON") {
    LOG_INFO(MODULE_RELAY, "🔌 Executing ENGINE_ON command - Turning relay ON");
    digitalWrite(RELAY_PIN, RELAY_ON);
    relayState = true;
    commandExecuted = true;
  } else if (commandType == "ENGINE_OFF") {
    LOG_INFO(MODULE_RELAY, "🔌 Executing ENGINE_OFF command - Turning relay OFF");
    digitalWrite(RELAY_PIN, RELAY_OFF);
    relayState = false;
    commandExecuted = true;
  } else {
    LOG_WARN(MODULE_RELAY, "Unknown command type: %s", commandType.c_str());
    return false;
  }
  
  // Update command status to "acknowledge" if command was executed
  if (commandExecuted && !commandId.isEmpty()) {
    LOG_INFO(MODULE_RELAY, "📤 Sending acknowledge status for command %s", commandId.c_str());
    return updateCommandStatus(commandId, "acknowledge");
  } else if (commandId.isEmpty()) {
    LOG_WARN(MODULE_RELAY, "⚠️ Command ID is empty, cannot send acknowledge status");
    LOG_INFO(MODULE_RELAY, "✅ Command executed successfully but status not updated");
    return true; // Still return success since command was executed
  }
  
  return false;
}

bool updateCommandStatus(const String& commandId, const String& status) {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No connection for command status update");
    return false;
  }
  
  LOG_INFO(MODULE_RELAY, "📤 Updating command %s status to %s",
           commandId.c_str(), status.c_str());
  
  // Construct the endpoint for the specific command
  String endpoint = "/items/commands/" + commandId;
  
  // Prepare the payload with proper JSON format
  DynamicJsonDocument doc(256);
  doc["status"] = status;
  
  String payload;
  serializeJson(doc, payload);
  
  LOG_DEBUG(MODULE_RELAY, "Status update payload: %s", payload.c_str());
  
  String response;
  bool success = Utils::retryOperation(
    MODULE_RELAY,
    "Command status update",
    [&]() {
      bool result = httpClient.patch(endpoint.c_str(), payload, response);
      if (result) {
        LOG_INFO(MODULE_RELAY, "✅ Command status updated successfully");
        LOG_INFO(MODULE_RELAY, "📋 Command %s marked as %s", commandId.c_str(), status.c_str());
        LOG_DEBUG(MODULE_RELAY, "Server response: %s", response.c_str());
      } else {
        LOG_ERROR(MODULE_RELAY, "❌ Failed to update command status");
      }
      return result;
    },
    MAX_HTTP_RETRIES
  );
  
  return success;
}

bool updateRelayStatus(bool state) {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No connection for relay update");
    return false;
  }
  
  LOG_INFO(MODULE_RELAY, "Manually updating relay to %s", state ? "ON" : "OFF");
  
  // Create a new command in the API
  String commandType = state ? "ENGINE_ON" : "ENGINE_OFF";
  
  // Prepare current timestamp in ISO format
  char timestamp[30];
  time_t now;
  time(&now);
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S.000Z", gmtime(&now));
  
  // Construct the payload according to the specified format
  String payload = "{\"data\":[{";
  payload += "\"command_id\":1,";
  payload += "\"vehicle_id\":\"1\",";
  payload += "\"issued_by\":\"c84b5015-ac42-45b1-9c36-7d8114ae8b5a\",";
  payload += "\"command_type\":\"" + commandType + "\",";
  payload += "\"status\":\"executed\",";
  payload += "\"date_sent\":\"" + String(timestamp) + "\"";
  payload += "}]}";
  
  LOG_DEBUG(MODULE_RELAY, "Payload: %s", payload.c_str());
  
  String response;
  bool success = Utils::retryOperation(
    MODULE_RELAY,
    "Manual relay update",
    [&]() {
      return httpClient.post("/items/commands", payload, response);
    },
    MAX_HTTP_RETRIES
  );
  
  if (success) {
    digitalWrite(RELAY_PIN, state ? RELAY_ON : RELAY_OFF);
    relayState = state;
    return true;
  }
  
  return false;
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
      updateRelayStatus(true);
    } else if (cmd == "off") {
      updateRelayStatus(false);
    } else if (cmd == "gps") {
      sendVehicleData();
    } else if (cmd == "test") {
      testServerConnectivity();
    } else if (cmd == "data") {
      LOG_INFO(MODULE_MAIN, "=== CURRENT VEHICLE DATA ===");
      char timestamp[30];
      gpsManager.getTimestamp(timestamp, sizeof(timestamp));
      String isoTimestamp = String(timestamp);
      isoTimestamp.replace("+07:00", ".000Z");
      
      LOG_INFO(MODULE_MAIN, "GPS ID: %s", GPS_ID);
      LOG_INFO(MODULE_MAIN, "Vehicle ID: %s", VEHICLE_ID);
      LOG_INFO(MODULE_MAIN, "Timestamp: %s", isoTimestamp.c_str());
      
      if (gpsManager.isValid()) {
        LOG_INFO(MODULE_MAIN, "GPS: Valid - Lat: %.6f, Lon: %.6f, Sats: %d",
                 gpsManager.getLatitude(), gpsManager.getLongitude(), gpsManager.getSatellites());
      } else {
        LOG_INFO(MODULE_MAIN, "GPS: No valid fix");
      }
      
      LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
      LOG_INFO(MODULE_MAIN, "Battery Level: 12.5V");
      LOG_INFO(MODULE_MAIN, "Vehicle Data Endpoint: %s", VEHICLE_DATA_ENDPOINT);
      LOG_INFO(MODULE_MAIN, "Commands Endpoint: %s", RELAY_ENDPOINT);
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
    LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f) Sats: %d", 
             gpsManager.getLatitude(), gpsManager.getLongitude(), 
             gpsManager.getSatellites());
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix");
  }
  
    // Modem Status
  if (modemManager.isGprsConnected()) {
    int csq = modemManager.getSignalQuality();
    LOG_INFO(MODULE_MAIN, "GPRS: Connected, Signal: %d (%s)", 
             csq, Utils::getSignalQualityString(csq));
  } else {
    LOG_INFO(MODULE_MAIN, "GPRS: Not connected");
  }
  
  // Relay Status
  LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "help     - Show this help");
  LOG_INFO(MODULE_MAIN, "status   - Show system status");
  LOG_INFO(MODULE_MAIN, "on/off   - Control relay");
  LOG_INFO(MODULE_MAIN, "gps      - Send vehicle data now");
  LOG_INFO(MODULE_MAIN, "test     - Test server connectivity");
  LOG_INFO(MODULE_MAIN, "data     - Show current vehicle data");
  LOG_INFO(MODULE_MAIN, "reset    - Restart system");
  LOG_INFO(MODULE_MAIN, "loglevel [0-4] - Set log level");
}