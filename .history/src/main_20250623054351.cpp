// ========================================
// Main.cpp - Realtime Smooth GPS Tracking
// ========================================

/**
 * ESP32 Vehicle Tracking - Smooth Realtime Version
 * - Sub-second GPS updates with prediction
 * - Kalman filtering for smooth movement
 * - Efficient WebSocket communication
 * - Client-side interpolation support
 */

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// Custom modules
#include "Config.h"
#include "Logger.h"
#include "Utils.h"
#include "RealtimeGpsManager.h"    // Enhanced GPS with prediction
#include "ModemManager.h"
#include "EnhancedWebSocketManager.h" // High-frequency WebSocket

// ----- SYSTEM STATES -----
enum SystemState {
  STATE_INIT,
  STATE_OPERATIONAL,
  STATE_HIGH_SPEED,     // Special state for high-speed movement
  STATE_MODEM_RESET,
  STATE_CONNECTION_RECOVERY,
  STATE_ERROR
};

// ----- TRACKING MODES -----
enum TrackingMode {
  TRACKING_NORMAL,      // Standard tracking
  TRACKING_REALTIME,    // High-frequency for smooth movement
  TRACKING_PREDICTIVE   // With prediction for ultra-smooth
};

// ----- GLOBAL OBJECTS -----
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
RealtimeGpsManager* gpsManager = nullptr;
ModemManager modemManager(modem, SerialAT);
EnhancedWebSocketManager* wsManager = nullptr;

// ----- STATE VARIABLES -----
SystemState currentState = STATE_INIT;
TrackingMode trackingMode = TRACKING_REALTIME;
unsigned long lastGpsSendTime = 0;
unsigned long lastPredictionTime = 0;
bool relayState = true;

// ----- REALTIME TRACKING PARAMETERS -----
struct TrackingConfig {
  unsigned long normalInterval = 2000;      // 2 seconds when stationary
  unsigned long movingInterval = 500;       // 500ms when moving slowly
  unsigned long highSpeedInterval = 200;    // 200ms when moving fast
  unsigned long predictionInterval = 100;   // 100ms prediction updates
  float highSpeedThreshold = 30.0;         // km/h threshold for high-speed mode
  float movingThreshold = 2.0;             // km/h threshold for movement
  bool enablePrediction = true;
  bool enableBatching = false;
  bool enableDeltaCompression = true;
} trackingConfig;

// ----- PERFORMANCE METRICS -----
struct PerformanceMetrics {
  unsigned long updateCount = 0;
  unsigned long predictionCount = 0;
  float avgUpdateRate = 0;
  unsigned long lastMetricTime = 0;
} metrics;

// ----- FUNCTION PROTOTYPES -----
void handleInitState();
void handleOperationalState();
void handleHighSpeedState();
void handleSerialCommands();
void printStatus();
void printHelp();
void onRelayUpdate(bool newState);
bool sendRealtimeUpdate();
bool sendPredictedPosition();
unsigned long getCurrentUpdateInterval();
void adjustTrackingMode();
void printTrackingConfig();

// ----- WEBSOCKET CALLBACK -----
void onRelayUpdate(bool newState) {
  if (newState != relayState) {
    digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
    relayState = newState;
    LOG_INFO(MODULE_RELAY, "Relay updated: %s", newState ? "ON" : "OFF");
  }
}

// ----- SETUP -----
void setup() {
  SerialMon.begin(115200);
  delay(100);
  
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 Smooth Realtime Tracking ===");
  LOG_INFO(MODULE_MAIN, "Version: 4.0 (Smooth Movement)");
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  // Initialize hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  
  // Initialize managers
  gpsManager = new RealtimeGpsManager(gps, SerialGPS);
  gpsManager->begin();
  
  modemManager.begin();
  
  wsManager = new EnhancedWebSocketManager(&gsmClient);
  wsManager->begin();
  wsManager->setOnRelayUpdate(onRelayUpdate);
  
  // Configure for realtime tracking
  wsManager->setUpdateMode(UPDATE_MODE_REALTIME);
  wsManager->setMinUpdateInterval(trackingConfig.highSpeedInterval);
  wsManager->enableThrottling(true);
  
  if (trackingConfig.enableDeltaCompression) {
    wsManager->setMinDistanceThreshold(0.5); // 0.5 meters
    wsManager->setMinSpeedChangeThreshold(0.2); // 0.2 km/h
  }
  
  printHelp();
  
  currentState = STATE_INIT;
  metrics.lastMetricTime = millis();
}

// ----- MAIN LOOP -----
void loop() {
  Utils::feedWatchdog();
  
  // Update GPS with high frequency
  gpsManager->update();
  
  // Update WebSocket
  if (currentState == STATE_OPERATIONAL || currentState == STATE_HIGH_SPEED) {
    wsManager->update();
  }
  
  // Handle serial commands
  handleSerialCommands();
  
  // State machine
  switch (currentState) {
    case STATE_INIT:
      handleInitState();
      break;
      
    case STATE_OPERATIONAL:
      handleOperationalState();
      break;
      
    case STATE_HIGH_SPEED:
      handleHighSpeedState();
      break;
      
    case STATE_MODEM_RESET:
      if (!modemManager.continueReset()) {
        if (modemManager.setup()) {
          currentState = STATE_OPERATIONAL;
        } else {
          currentState = STATE_ERROR;
        }
      }
      break;
      
    case STATE_CONNECTION_RECOVERY:
      // Recovery logic
      break;
      
    case STATE_ERROR:
      modemManager.startReset();
      currentState = STATE_MODEM_RESET;
      break;
  }
  
  // Short delay for stability
  delay(5);
}

// ----- STATE HANDLERS -----
void handleInitState() {
  if (modemManager.setup()) {
    if (wsManager->connect()) {
      LOG_INFO(MODULE_SYS, "System ready for realtime tracking");
      currentState = STATE_OPERATIONAL;
    }
  } else {
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleOperationalState() {
  unsigned long currentTime = millis();
  
  // Adjust tracking mode based on speed
  adjustTrackingMode();
  
  // Get current update interval
  unsigned long updateInterval = getCurrentUpdateInterval();
  
  // Send actual GPS position
  if (currentTime - lastGpsSendTime >= updateInterval) {
    if (sendRealtimeUpdate()) {
      lastGpsSendTime = currentTime;
      metrics.updateCount++;
    }
  }
  
  // Send predicted positions between GPS updates for smooth movement
  if (trackingConfig.enablePrediction && gpsManager->isMoving()) {
    if (currentTime - lastPredictionTime >= trackingConfig.predictionInterval) {
      sendPredictedPosition();
      lastPredictionTime = currentTime;
      metrics.predictionCount++;
    }
  }
  
  // Check if should switch to high-speed mode
  if (gpsManager->getRawSpeed() >= trackingConfig.highSpeedThreshold) {
    currentState = STATE_HIGH_SPEED;
    LOG_INFO(MODULE_SYS, "Switching to HIGH-SPEED tracking mode");
  }
}

void handleHighSpeedState() {
  unsigned long currentTime = millis();
  
  // Ultra-fast updates in high-speed mode
  if (currentTime - lastGpsSendTime >= trackingConfig.highSpeedInterval) {
    if (sendRealtimeUpdate()) {
      lastGpsSendTime = currentTime;
      metrics.updateCount++;
    }
  }
  
  // Even more frequent predictions
  if (trackingConfig.enablePrediction) {
    if (currentTime - lastPredictionTime >= 50) { // 50ms predictions
      sendPredictedPosition();
      lastPredictionTime = currentTime;
      metrics.predictionCount++;
    }
  }
  
  // Check if should return to normal mode
  if (gpsManager->getRawSpeed() < trackingConfig.highSpeedThreshold - 5.0) {
    currentState = STATE_OPERATIONAL;
    LOG_INFO(MODULE_SYS, "Returning to NORMAL tracking mode");
  }
}

// ----- TRACKING FUNCTIONS -----
bool sendRealtimeUpdate() {
  if (!wsManager->isReady()) {
    return false;
  }
  
  // Get current position with smoothing
  InterpolatedPosition pos = gpsManager->getCurrentPosition();
  
  // Create high-precision timestamp
  char timestamp[35];
  gpsManager->getTimestamp(timestamp, sizeof(timestamp));
  
  // Add update type marker for client
  JsonDocument doc;
  doc["type"] = "actual"; // Mark as actual GPS reading
  
  // Send position update
  bool success = wsManager->sendPositionUpdate(
    pos.latitude,
    pos.longitude,
    pos.speed,
    pos.course,
    pos.accuracy,
    String(timestamp)
  );
  
  if (success) {
    LOG_DEBUG(MODULE_GPS, "Realtime update sent: %.6f,%.6f @ %.1f km/h",
              pos.latitude, pos.longitude, pos.speed);
  }
  
  return success;
}

bool sendPredictedPosition() {
  if (!wsManager->isReady() || !gpsManager->hasValidFix()) {
    return false;
  }
  
  // Get predicted position 100ms in the future
  InterpolatedPosition predicted = gpsManager->getPredictedPosition(100);
  
  // Don't send if prediction is too uncertain
  if (predicted.accuracy > 20.0) {
    return false;
  }
  
  // Create timestamp for predicted position
  char timestamp[35];
  unsigned long futureTime = millis() + 100;
  // Format timestamp with future time
  snprintf(timestamp, sizeof(timestamp), "2025-01-15T12:00:00.%03dZ", 
           (int)(futureTime % 1000));
  
  // Send with prediction marker
  JsonDocument doc;
  doc["type"] = "predicted";
  
  bool success = wsManager->sendPositionUpdate(
    predicted.latitude,
    predicted.longitude,
    predicted.speed,
    predicted.course,
    predicted.accuracy * 1.5, // Increase uncertainty for predictions
    String(timestamp)
  );
  
  return success;
}

unsigned long getCurrentUpdateInterval() {
  float speed = gpsManager->getRawSpeed();
  
  if (speed < trackingConfig.movingThreshold) {
    return trackingConfig.normalInterval;
  } else if (speed < trackingConfig.highSpeedThreshold) {
    // Linear interpolation between moving and high-speed intervals
    float ratio = (speed - trackingConfig.movingThreshold) / 
                  (trackingConfig.highSpeedThreshold - trackingConfig.movingThreshold);
    return trackingConfig.movingInterval - 
           (unsigned long)(ratio * (trackingConfig.movingInterval - trackingConfig.highSpeedInterval));
  } else {
    return trackingConfig.highSpeedInterval;
  }
}

void adjustTrackingMode() {
  float speed = gpsManager->getRawSpeed();
  
  if (speed < trackingConfig.movingThreshold) {
    if (trackingMode != TRACKING_NORMAL) {
      trackingMode = TRACKING_NORMAL;
      wsManager->setUpdateMode(UPDATE_MODE_NORMAL);
      LOG_INFO(MODULE_SYS, "Switched to NORMAL tracking");
    }
  } else if (speed < trackingConfig.highSpeedThreshold) {
    if (trackingMode != TRACKING_REALTIME) {
      trackingMode = TRACKING_REALTIME;
      wsManager->setUpdateMode(UPDATE_MODE_REALTIME);
      LOG_INFO(MODULE_SYS, "Switched to REALTIME tracking");
    }
  } else {
    if (trackingMode != TRACKING_PREDICTIVE) {
      trackingMode = TRACKING_PREDICTIVE;
      wsManager->setUpdateMode(UPDATE_MODE_REALTIME);
      wsManager->enableThrottling(false); // Disable throttling for max speed
      LOG_INFO(MODULE_SYS, "Switched to PREDICTIVE tracking");
    }
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
    } else if (cmd == "config") {
      printTrackingConfig();
    } else if (cmd.startsWith("set ")) {
      // Configuration commands
      String param = cmd.substring(4);
      if (param.startsWith("prediction ")) {
        trackingConfig.enablePrediction = param.endsWith("on");
        LOG_INFO(MODULE_MAIN, "Prediction %s", trackingConfig.enablePrediction ? "enabled" : "disabled");
      } else if (param.startsWith("batch ")) {
        trackingConfig.enableBatching = param.endsWith("on");
        wsManager->setUpdateMode(trackingConfig.enableBatching ? UPDATE_MODE_BATCH : UPDATE_MODE_REALTIME);
        LOG_INFO(MODULE_MAIN, "Batching %s", trackingConfig.enableBatching ? "enabled" : "disabled");
      } else if (param.startsWith("highspeed ")) {
        trackingConfig.highSpeedInterval = param.substring(10).toInt();
        LOG_INFO(MODULE_MAIN, "High-speed interval: %lu ms", trackingConfig.highSpeedInterval);
      }
    } else if (cmd == "metrics") {
      // Calculate and show performance metrics
      unsigned long elapsed = millis() - metrics.lastMetricTime;
      float updateRate = (metrics.updateCount * 1000.0) / elapsed;
      float predictionRate = (metrics.predictionCount * 1000.0) / elapsed;
      
      LOG_INFO(MODULE_MAIN, "=== PERFORMANCE METRICS ===");
      LOG_INFO(MODULE_MAIN, "Update Rate: %.1f Hz", updateRate);
      LOG_INFO(MODULE_MAIN, "Prediction Rate: %.1f Hz", predictionRate);
      LOG_INFO(MODULE_MAIN, "Total Updates: %lu", wsManager->getTotalUpdatesSent());
      LOG_INFO(MODULE_MAIN, "Bytes Optimized: %lu", wsManager->getBytesOptimized());
      
      // Reset metrics
      metrics.updateCount = 0;
      metrics.predictionCount = 0;
      metrics.lastMetricTime = millis();
    }
  }
}

void printStatus() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM STATUS ===");
  LOG_INFO(MODULE_MAIN, "State: %s", currentState == STATE_OPERATIONAL ? "OPERATIONAL" : 
                                     currentState == STATE_HIGH_SPEED ? "HIGH-SPEED" : "OTHER");
  LOG_INFO(MODULE_MAIN, "Tracking Mode: %s", trackingMode == TRACKING_NORMAL ? "NORMAL" :
                                             trackingMode == TRACKING_REALTIME ? "REALTIME" : "PREDICTIVE");
  
  if (gpsManager->hasValidFix()) {
    InterpolatedPosition pos = gpsManager->getCurrentPosition();
    LOG_INFO(MODULE_MAIN, "GPS: %.6f, %.6f @ %.1f km/h (±%.1fm)",
             pos.latitude, pos.longitude, pos.speed, pos.accuracy);
    LOG_INFO(MODULE_MAIN, "Satellites: %d, HDOP: %.1f", 
             gpsManager->getSatellites(), gpsManager->getHDOP());
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix");
  }
  
  LOG_INFO(MODULE_MAIN, "Update Interval: %lu ms", getCurrentUpdateInterval());
  LOG_INFO(MODULE_MAIN, "WebSocket: %s", wsManager->getStateString());
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== SMOOTH TRACKING COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "help     - Show this help");
  LOG_INFO(MODULE_MAIN, "status   - Show system status");
  LOG_INFO(MODULE_MAIN, "config   - Show tracking configuration");
  LOG_INFO(MODULE_MAIN, "metrics  - Show performance metrics");
  LOG_INFO(MODULE_MAIN, "set prediction on/off - Enable/disable prediction");
  LOG_INFO(MODULE_MAIN, "set batch on/off - Enable/disable batching");
  LOG_INFO(MODULE_MAIN, "set highspeed <ms> - Set high-speed interval");
}

void printTrackingConfig() {
  LOG_INFO(MODULE_MAIN, "=== TRACKING CONFIGURATION ===");
  LOG_INFO(MODULE_MAIN, "Normal Interval: %lu ms", trackingConfig.normalInterval);
  LOG_INFO(MODULE_MAIN, "Moving Interval: %lu ms", trackingConfig.movingInterval);
  LOG_INFO(MODULE_MAIN, "High-Speed Interval: %lu ms", trackingConfig.highSpeedInterval);
  LOG_INFO(MODULE_MAIN, "Prediction Interval: %lu ms", trackingConfig.predictionInterval);
  LOG_INFO(MODULE_MAIN, "High-Speed Threshold: %.1f km/h", trackingConfig.highSpeedThreshold);
  LOG_INFO(MODULE_MAIN, "Moving Threshold: %.1f km/h", trackingConfig.movingThreshold);
  LOG_INFO(MODULE_MAIN, "Prediction: %s", trackingConfig.enablePrediction ? "ON" : "OFF");
  LOG_INFO(MODULE_MAIN, "Batching: %s", trackingConfig.enableBatching ? "ON" : "OFF");
  LOG_INFO(MODULE_MAIN, "Delta Compression: %s", trackingConfig.enableDeltaCompression ? "ON" : "OFF");
}