// ========================================
// Main.cpp - Modified with Network Testing Support
// ========================================

/**
 * ESP32 Vehicle Tracking with WebSocket and Network Testing
 * - Real-time GPS data transmission via WebSocket
 * - Network stability testing capabilities
 * - Comprehensive metrics collection
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
#include "NetworkTestManager.h" // Network testing module

// ----- SYSTEM STATES -----
enum SystemState {
  STATE_INIT,
  STATE_IDLE,
  STATE_OPERATIONAL,
  STATE_MODEM_RESET,
  STATE_CONNECTION_RECOVERY,
  STATE_ERROR,
  STATE_TESTING // New state for testing mode
};

// ----- MOVEMENT DETECTION -----
enum MovementState {
  MOVEMENT_UNKNOWN,
  MOVEMENT_STATIC,
  MOVEMENT_MOVING
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
NetworkTestManager* networkTester = nullptr; // Network test manager

// ----- STATE VARIABLES -----
SystemState currentState = STATE_INIT;
unsigned long lastGpsSendTime = 0;
unsigned long lastSuccessfulOperation = 0;
bool relayState = true; // Default relay state is ON

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
void handleTestingState();
void handleSerialCommands();
void printStatus();
void printHelp();
void printTestCommands();
bool sendVehicleDataViaWebSocket();
bool testServerConnectivity();
void onRelayUpdate(bool newState);

// Movement detection functions
void updateMovementDetection();
MovementState detectMovementState();
float getAverageSpeed();
unsigned long getCurrentGpsInterval();
const char* getMovementStateString(MovementState state);

// ----- WEBSOCKET CALLBACK -----
void onRelayUpdate(bool newState) {
  if (newState != relayState) {
    LOG_INFO(MODULE_RELAY, "🔄 Relay update from WebSocket: %s → %s", 
             relayState ? "ON" : "OFF", newState ? "ON" : "OFF");
    
    // Update physical relay
    digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
    relayState = newState;
    
    LOG_INFO(MODULE_RELAY, "✅ Physical relay updated to: %s", newState ? "ON" : "OFF");
  }
}

// ----- SETUP -----
void setup() {
  // Initialize serial monitor
  SerialMon.begin(115200);
  delay(100);
  
  // Initialize logging
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 Vehicle Tracking System ===");
  LOG_INFO(MODULE_MAIN, "Version: 3.1 (WebSocket + Network Testing)");
  LOG_INFO(MODULE_MAIN, "WebSocket URL: %s", WS_URL);
  LOG_INFO(MODULE_MAIN, "GPS ID: %s", GPS_ID);
  
  // Initialize movement detection arrays
  for (int i = 0; i < MOVEMENT_DETECTION_SAMPLES; i++) {
    speedSamples[i] = 0.0;
  }
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  // Initialize hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON); // Default relay state is ON
  
  // Initialize managers
  gpsManager.begin();
  modemManager.begin();
  
  // Initialize WebSocket manager
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  // Initialize network tester
  networkTester = new NetworkTestManager(&modemManager, &wsManager, &httpClient);
  
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
  if (currentState == STATE_OPERATIONAL || currentState == STATE_TESTING) {
    wsManager.update();
  }
  
  // Update network tester if active
  if (networkTester && networkTester->isTestRunning()) {
    networkTester->update();
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
      
    case STATE_TESTING:
      handleTestingState();
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
  
  // Check for system stuck condition (not during testing)
  if (currentState == STATE_OPERATIONAL && !networkTester->isTestRunning()) {
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
  
  // Don't send normal GPS data if test is running
  if (networkTester->isTestRunning()) {
    return;
  }
  
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

void handleTestingState() {
  // Testing state - network tester handles everything
  if (!networkTester->isTestRunning()) {
    // Test completed, return to operational
    currentState = STATE_OPERATIONAL;
    LOG_INFO(MODULE_SYS, "Returning to operational state");
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
    } else if (cmd == "test") {
      printTestCommands();
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
    } else if (cmd == "reset") {
      ESP.restart();
    }
    // Test commands
    else if (cmd.startsWith("test ")) {
      String testCmd = cmd.substring(5);
      
      if (testCmd == "continuous") {
        networkTester->startTest(TEST_MODE_CONTINUOUS_TRANSMISSION, 1800000); // 30 minutes
        currentState = STATE_TESTING;
      } else if (testCmd == "disruption") {
        networkTester->startTest(TEST_MODE_CONNECTION_DISRUPTION, 1800000);
        currentState = STATE_TESTING;
        // Schedule disruptions at 5, 10, and 15 minutes
        networkTester->scheduleDisruption(300000, 30000);  // 5 min delay, 30s disruption
      } else if (testCmd == "load") {
        networkTester->startTest(TEST_MODE_LOAD_TEST, 300000); // 5 minutes
        currentState = STATE_TESTING;
      } else if (testCmd == "signal") {
        networkTester->startTest(TEST_MODE_SIGNAL_MONITORING, 600000); // 10 minutes
        currentState = STATE_TESTING;
      } else if (testCmd == "stop") {
        networkTester->stopTest();
      } else if (testCmd == "metrics") {
        networkTester->printMetrics();
      } else if (testCmd == "export") {
        networkTester->exportTestResults();
      } else if (testCmd.startsWith("interval ")) {
        int interval = testCmd.substring(9).toInt();
        if (interval >= 1000) {
          networkTester->setDataInterval(interval);
          LOG_INFO(MODULE_MAIN, "Test data interval set to %d ms", interval);
        }
      } else if (testCmd.startsWith("disrupt ")) {
        // Format: test disrupt <delay_sec> <duration_sec>
        int spacePos = testCmd.indexOf(' ', 8);
        if (spacePos > 0) {
          int delaySec = testCmd.substring(8, spacePos).toInt();
          int durationSec = testCmd.substring(spacePos + 1).toInt();
          networkTester->scheduleDisruption(delaySec * 1000, durationSec * 1000);
        }
      }
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
  
  // Test Status
  if (networkTester->isTestRunning()) {
    LOG_INFO(MODULE_MAIN, "Test: %s (Progress: %.1f%%)", 
             networkTester->getTestModeName(), 
             networkTester->getTestProgress());
  }
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "help       - Show this help");
  LOG_INFO(MODULE_MAIN, "test       - Show test commands");
  LOG_INFO(MODULE_MAIN, "status     - Show system status");
  LOG_INFO(MODULE_MAIN, "on/off     - Manual relay control");
  LOG_INFO(MODULE_MAIN, "gps        - Send GPS data now");
  LOG_INFO(MODULE_MAIN, "ws         - Show WebSocket status");
  LOG_INFO(MODULE_MAIN, "connect    - Connect WebSocket");
  LOG_INFO(MODULE_MAIN, "disconnect - Disconnect WebSocket");
  LOG_INFO(MODULE_MAIN, "reset      - Restart system");
}

void printTestCommands() {
  LOG_INFO(MODULE_MAIN, "=== TEST COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "test continuous  - Start 30-min continuous transmission test");
  LOG_INFO(MODULE_MAIN, "test disruption  - Start connection disruption test");
  LOG_INFO(MODULE_MAIN, "test load        - Start 5-min load test");
  LOG_INFO(MODULE_MAIN, "test signal      - Start 10-min signal monitoring");
  LOG_INFO(MODULE_MAIN, "test stop        - Stop current test");
  LOG_INFO(MODULE_MAIN, "test metrics     - Show current test metrics");
  LOG_INFO(MODULE_MAIN, "test export      - Export test results as JSON");
  LOG_INFO(MODULE_MAIN, "test interval <ms> - Set data interval (min 1000ms)");
  LOG_INFO(MODULE_MAIN, "test disrupt <delay_s> <duration_s> - Schedule disruption");
}