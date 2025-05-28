// main.cpp
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
bool checkRelayStatus();
bool updateRelayStatus(bool state);

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
    if (httpClient.testConnection()) {
      LOG_INFO(MODULE_SYS, "Server connectivity verified");
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
  
  // Check if time to send GPS data
  if (currentTime - lastGpsSendTime >= GPS_SEND_INTERVAL || gpsManager.hasNewFix()) {
    if (gpsManager.isValid()) {
      if (sendGpsData()) {
        lastGpsSendTime = currentTime;
        lastSuccessfulOperation = currentTime;
      }
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
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No connection for GPS data");
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "Sending GPS data...");
  
  // Prepare JSON payload
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  
  StaticJsonDocument<256> doc;
  doc["id"] = DEVICE_ID;
  doc["latitude"] = gpsManager.getLatitude();
  doc["longitude"] = gpsManager.getLongitude();
  doc["timestamp"] = timestamp;
  
  String payload;
  serializeJson(doc, payload);
  
  LOG_DEBUG(MODULE_GPS, "Payload: %s", payload.c_str());
  
  // Send with retry
  return Utils::retryOperation(
    MODULE_GPS,
    "GPS data sending",
    [&]() {
      String response;
      return httpClient.post(GPS_ENDPOINT, payload, response);
    },
    MAX_HTTP_RETRIES
  );
}

// ----- RELAY OPERATIONS -----
bool checkRelayStatus() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No connection for relay check");
    return false;
  }
  
  LOG_INFO(MODULE_RELAY, "Checking relay status...");
  
  String response;
  if (!httpClient.get(RELAY_ENDPOINT, response)) {
    return false;
  }
  
  // Parse response to find relay status
  String searchStr = String("\"id\":") + String(RELAY_ID) + String(",");
  int relayIndex = response.indexOf(searchStr);
  
  if (relayIndex >= 0) {
    int isActiveIndex = response.indexOf("\"is_active\":", relayIndex);
    if (isActiveIndex >= 0) {
      char activeValue = response.charAt(isActiveIndex + 12);
      bool newState = (activeValue == '1');
      
      if (newState != relayState) {
        LOG_INFO(MODULE_RELAY, "Relay state changed: %s", newState ? "ON" : "OFF");
        digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
        relayState = newState;
      }
      
      return true;
    }
  }
  
  LOG_ERROR(MODULE_RELAY, "Relay %d not found in response", RELAY_ID);
  return false;
}

bool updateRelayStatus(bool state) {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_RELAY, "No connection for relay update");
    return false;
  }
  
  LOG_INFO(MODULE_RELAY, "Updating relay to %s", state ? "ON" : "OFF");
  
  String path = String(RELAY_ENDPOINT) + "/" + String(RELAY_ID);
  String payload = "{\"is_active\":" + String(state ? "1" : "0") + "}";
  
  String response;
  if (httpClient.patch(path.c_str(), payload, response)) {
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
      if (gpsManager.isValid()) {
        sendGpsData();
      } else {
        LOG_WARN(MODULE_MAIN, "GPS not valid");
      }
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
  LOG_INFO(MODULE_MAIN, "gps      - Send GPS data now");
  LOG_INFO(MODULE_MAIN, "reset    - Restart system");
  LOG_INFO(MODULE_MAIN, "loglevel [0-4] - Set log level");
}