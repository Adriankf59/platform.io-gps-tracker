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
bool relayState = true; // Default relay state is ON

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
bool updateRelayStatus(bool state);
bool checkVehicleRelayStatus();

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
  
  // Check if time to check relay from vehicle API
  if (currentTime - lastRelayCheckTime >= RELAY_CHECK_INTERVAL) {
    if (checkVehicleRelayStatus()) {
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
  
  LOG_INFO(MODULE_GPS, "Sending vehicle data...");
  
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
            LOG_INFO(MODULE_GPS, "📍 GPS: lat=%.6f, lon=%.6f, speed=%.1f km/h, sats=%d",
                     gpsManager.getLatitude(), gpsManager.getLongitude(),
                     gpsManager.getSpeed(), gpsManager.getSatellites());
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

// ----- VEHICLE RELAY STATUS CHECK -----
bool checkVehicleRelayStatus() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No connection for vehicle relay status check");
    return false;
  }
  
  LOG_INFO(MODULE_RELAY, "Checking vehicle relay status from API...");
  
  // Use the vehicle endpoint
  String endpoint = "/items/vehicle";
  
  String response;
  bool success = Utils::retryOperation(
    MODULE_RELAY,
    "Vehicle relay status check",
    [&]() {
      return httpClient.get(endpoint.c_str(), response);
    },
    MAX_HTTP_RETRIES
  );
  
  if (!success) {
    LOG_ERROR(MODULE_RELAY, "Failed to get vehicle data from API");
    return false;
  }
  
  // Parse response
  DynamicJsonDocument doc(2048); // Larger buffer for vehicle data
  DeserializationError error = deserializeJson(doc, response);
  
  if (error) {
    LOG_ERROR(MODULE_RELAY, "JSON parsing failed: %s", error.c_str());
    return false;
  }
  
  // Check Directus API response format
  if (!doc.containsKey("data") || !doc["data"].is<JsonArray>()) {
    LOG_ERROR(MODULE_RELAY, "Invalid vehicle response format");
    LOG_DEBUG(MODULE_RELAY, "Response: %s", response.c_str());
    return false;
  }
  
  JsonArray vehicles = doc["data"].as<JsonArray>();
  
  if (vehicles.size() == 0) {
    LOG_WARN(MODULE_RELAY, "No vehicles found in API response");
    return false;
  }
  
  // Find vehicle with matching gps_id
  bool vehicleFound = false;
  String apiRelayStatus = "";
  
  for (JsonObject vehicle : vehicles) {
    if (vehicle.containsKey("gps_id") &&
        vehicle["gps_id"].as<String>() == String(GPS_ID)) {
      vehicleFound = true;
      
      if (vehicle.containsKey("relay_status") && !vehicle["relay_status"].isNull()) {
        apiRelayStatus = vehicle["relay_status"].as<String>();
      }
      break;
    }
  }
  
  if (!vehicleFound) {
    LOG_WARN(MODULE_RELAY, "GPS ID %s not found in API response", GPS_ID);
    return false;
  }
  
  if (apiRelayStatus.isEmpty()) {
    LOG_DEBUG(MODULE_RELAY, "Relay status is null/empty for vehicle %s", VEHICLE_ID);
    return true; // Consider this successful but no action needed
  }
  
  LOG_INFO(MODULE_RELAY, "API relay status for GPS ID %s: %s", GPS_ID, apiRelayStatus.c_str());
  
  // Compare with current relay state and update if different
  bool apiRelayOn = (apiRelayStatus == "ON");
  
  if (apiRelayOn != relayState) {
    LOG_INFO(MODULE_RELAY, "🔄 Relay status mismatch! Current: %s, API: %s",
             relayState ? "ON" : "OFF", apiRelayStatus.c_str());
    LOG_INFO(MODULE_RELAY, "🔌 Updating relay to match API status: %s", apiRelayStatus.c_str());
    
    // Update physical relay
    digitalWrite(RELAY_PIN, apiRelayOn ? RELAY_ON : RELAY_OFF);
    relayState = apiRelayOn;
    
    LOG_INFO(MODULE_RELAY, "✅ Relay updated successfully to %s", apiRelayOn ? "ON" : "OFF");
  } else {
    LOG_DEBUG(MODULE_RELAY, "✅ Relay status matches API: %s", apiRelayStatus.c_str());
  }
  
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
      updateRelayStatus(true);
    } else if (cmd == "off") {
      updateRelayStatus(false);
    } else if (cmd == "gps") {
      sendVehicleData();
    } else if (cmd == "test") {
      testServerConnectivity();
    } else if (cmd == "vehicle") {
      checkVehicleRelayStatus();
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
      LOG_INFO(MODULE_MAIN, "Vehicle ID: %s", VEHICLE_ID);
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
      
      LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
      LOG_INFO(MODULE_MAIN, "Battery Level: 12.5V");
      LOG_INFO(MODULE_MAIN, "Vehicle Data Endpoint: %s", VEHICLE_DATA_ENDPOINT);
      LOG_INFO(MODULE_MAIN, "Vehicle Endpoint: /items/vehicle");
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
  LOG_INFO(MODULE_MAIN, "vehicle  - Check vehicle relay status from API");
  LOG_INFO(MODULE_MAIN, "data     - Show current vehicle data");
  LOG_INFO(MODULE_MAIN, "reset    - Restart system");
  LOG_INFO(MODULE_MAIN, "loglevel [0-4] - Set log level");
}