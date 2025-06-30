// ========================================
// Main.cpp - With Power Modes for Testing
// ========================================

/**
 * ESP32 Vehicle Tracking with Power Management Modes
 * - Mode Aktif Penuh: Real-time tracking
 * - Mode Sleep/Standby: Power saving with periodic updates
 * - Mode Darurat: Minimal power for low battery conditions
 */

// ----- ARDUINO FRAMEWORK -----
#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>

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

// ----- POWER MODES -----
enum PowerMode {
  POWER_MODE_FULL,      // Mode Aktif Penuh
  POWER_MODE_STANDBY,   // Mode Sleep/Standby
  POWER_MODE_EMERGENCY  // Mode Darurat
};

// ----- SYSTEM STATES -----
enum SystemState {
  STATE_INIT,
  STATE_IDLE,
  STATE_OPERATIONAL,
  STATE_MODEM_RESET,
  STATE_CONNECTION_RECOVERY,
  STATE_ERROR,
  STATE_SLEEP_PREPARE,
  STATE_SLEEPING
};

// ----- MOVEMENT STATES -----
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

// ----- STATE VARIABLES -----
SystemState currentState = STATE_INIT;
PowerMode currentPowerMode = POWER_MODE_FULL;
MovementState currentMovementState = MOVEMENT_UNKNOWN;
unsigned long lastGpsSendTime = 0;
unsigned long lastSuccessfulOperation = 0;
unsigned long lastActivityTime = 0;
bool relayState = true;

// ----- POWER MODE CONFIGURATIONS -----
struct PowerModeConfig {
  unsigned long gpsInterval;        // GPS update interval
  unsigned long wsKeepAliveInterval; // WebSocket keepalive
  unsigned long sleepDuration;      // Sleep duration for standby
  bool gpsAlwaysOn;                // Keep GPS powered
  bool wsContinuous;               // Maintain WebSocket connection
  bool relayEnabled;               // Allow relay control
};

// Power mode configurations
PowerModeConfig powerConfigs[3] = {
  // POWER_MODE_FULL
  {
    .gpsInterval = 10000,           // 10 seconds
    .wsKeepAliveInterval = 45000,   // 45 seconds
    .sleepDuration = 0,             // No sleep
    .gpsAlwaysOn = true,
    .wsContinuous = true,
    .relayEnabled = true
  },
  // POWER_MODE_STANDBY
  {
    .gpsInterval = 300000,          // 5 minutes
    .wsKeepAliveInterval = 120000,  // 2 minutes
    .sleepDuration = 240000,        // 4 minutes light sleep
    .gpsAlwaysOn = false,
    .wsContinuous = false,
    .relayEnabled = true
  },
  // POWER_MODE_EMERGENCY
  {
    .gpsInterval = 1800000,         // 30 minutes
    .wsKeepAliveInterval = 0,       // No keepalive
    .sleepDuration = 1740000,       // 29 minutes deep sleep
    .gpsAlwaysOn = false,
    .wsContinuous = false,
    .relayEnabled = false
  }
};

// Current battery voltage
float batteryVoltage = 12.6;

// ----- FUNCTION PROTOTYPES -----
void handleInitState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSleepPrepareState();
void handleSerialCommands();
void printStatus();
void printHelp();
void printPowerModeInfo();
bool sendVehicleDataViaWebSocket();
void onRelayUpdate(bool newState);
void setPowerMode(PowerMode mode);
void enterLightSleep(unsigned long duration);
void enterDeepSleep(unsigned long duration);
void disableUnusedPeripherals();
void enablePeripherals();
float readBatteryVoltage();
void updateBatteryStatus();
void checkEmergencyMode();

// ----- SETUP -----
void setup() {
  SerialMon.begin(115200);
  delay(100);
  
  Logger::init(&SerialMon, LOG_INFO);
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 Vehicle Tracking with Power Management ===");
  LOG_INFO(MODULE_MAIN, "Version: 5.0 (Power Test Edition)");
  LOG_INFO(MODULE_MAIN, "Power Mode: %s", getPowerModeString(currentPowerMode));
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  
  // Initialize hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  
  // Initialize battery voltage reading pin (if using ADC)
  // pinMode(BATTERY_ADC_PIN, INPUT);
  
  // Initialize managers
  gpsManager.begin();
  modemManager.begin();
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  // Print initial status
  printHelp();
  printPowerModeInfo();
  
  currentState = STATE_INIT;
  lastSuccessfulOperation = millis();
  lastActivityTime = millis();
}

// ----- MAIN LOOP -----
void loop() {
  Utils::feedWatchdog();
  
  // Update battery status
  updateBatteryStatus();
  
  // Check if emergency mode needed
  checkEmergencyMode();
  
  // Update GPS based on power mode
  if (powerConfigs[currentPowerMode].gpsAlwaysOn || 
      (millis() - lastGpsSendTime > powerConfigs[currentPowerMode].gpsInterval - 30000)) {
    gpsManager.update();
  }
  
  // Update WebSocket if in continuous mode
  if (currentState == STATE_OPERATIONAL && powerConfigs[currentPowerMode].wsContinuous) {
    wsManager.update();
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
      
    case STATE_SLEEP_PREPARE:
      handleSleepPrepareState();
      break;
      
    case STATE_ERROR:
      LOG_ERROR(MODULE_SYS, "System in error state, attempting recovery...");
      modemManager.startReset();
      currentState = STATE_MODEM_RESET;
      break;
  }
  
  delay(10);
}

// ----- STATE HANDLERS -----
void handleOperationalState() {
  unsigned long currentTime = millis();
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  // Check if time to send GPS data
  if (currentTime - lastGpsSendTime >= config.gpsInterval) {
    if (sendVehicleDataViaWebSocket()) {
      lastGpsSendTime = currentTime;
      lastSuccessfulOperation = currentTime;
      lastActivityTime = currentTime;
    }
  }
  
  // Enter sleep if configured
  if (config.sleepDuration > 0 && 
      currentTime - lastActivityTime > 60000) { // 1 minute inactivity
    currentState = STATE_SLEEP_PREPARE;
  }
}

void handleSleepPrepareState() {
  LOG_INFO(MODULE_SYS, "Preparing for sleep mode...");
  
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  // Disconnect WebSocket if not continuous
  if (!config.wsContinuous) {
    wsManager.disconnect();
  }
  
  // Turn off GPS if not always on
  if (!config.gpsAlwaysOn) {
    // Power down GPS (implementation depends on hardware)
    LOG_INFO(MODULE_GPS, "Powering down GPS");
  }
  
  // Disable unused peripherals
  disableUnusedPeripherals();
  
  // Enter appropriate sleep mode
  if (currentPowerMode == POWER_MODE_EMERGENCY) {
    LOG_INFO(MODULE_SYS, "Entering deep sleep for %lu ms", config.sleepDuration);
    enterDeepSleep(config.sleepDuration);
  } else {
    LOG_INFO(MODULE_SYS, "Entering light sleep for %lu ms", config.sleepDuration);
    enterLightSleep(config.sleepDuration);
  }
  
  // After waking up
  LOG_INFO(MODULE_SYS, "Woke up from sleep");
  enablePeripherals();
  currentState = STATE_OPERATIONAL;
  lastActivityTime = millis();
}

// ----- POWER MODE MANAGEMENT -----
void setPowerMode(PowerMode mode) {
  if (mode == currentPowerMode) return;
  
  LOG_INFO(MODULE_SYS, "Switching power mode: %s -> %s", 
           getPowerModeString(currentPowerMode), 
           getPowerModeString(mode));
  
  currentPowerMode = mode;
  PowerModeConfig& config = powerConfigs[mode];
  
  // Adjust WebSocket ping interval
  if (config.wsContinuous) {
    // WebSocket manager should adjust ping interval
    LOG_INFO(MODULE_WS, "WebSocket keepalive: %lu ms", config.wsKeepAliveInterval);
  }
  
  // Log power consumption expectations
  float expectedCurrent = 0;
  switch(mode) {
    case POWER_MODE_FULL:
      expectedCurrent = 250; // ~250mA average
      break;
    case POWER_MODE_STANDBY:
      expectedCurrent = 80;  // ~80mA average
      break;
    case POWER_MODE_EMERGENCY:
      expectedCurrent = 25;  // ~25mA average
      break;
  }
  
  LOG_INFO(MODULE_SYS, "Expected average current: ~%.0f mA", expectedCurrent);
  printPowerModeInfo();
}

void enterLightSleep(unsigned long duration) {
  // Configure wake up timer
  esp_sleep_enable_timer_wakeup(duration * 1000); // Convert to microseconds
  
  // Keep RTC memory powered
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  
  // Enter light sleep
  esp_light_sleep_start();
}

void enterDeepSleep(unsigned long duration) {
  // Configure wake up timer
  esp_sleep_enable_timer_wakeup(duration * 1000); // Convert to microseconds
  
  // Enter deep sleep
  esp_deep_sleep_start();
  // Code after this will not execute
}

void disableUnusedPeripherals() {
  // Disable WiFi (if not used)
  esp_wifi_stop();
  
  // Disable Bluetooth
  esp_bt_controller_disable();
  
  // Set unused GPIOs to input with pulldown
  // (Add specific GPIO configurations based on your hardware)
  
  LOG_DEBUG(MODULE_SYS, "Peripherals disabled for power saving");
}

void enablePeripherals() {
  // Re-enable required peripherals after sleep
  LOG_DEBUG(MODULE_SYS, "Peripherals re-enabled");
}

float readBatteryVoltage() {
  // Implement actual battery voltage reading
  // This is a placeholder - implement based on your hardware
  // Example: Using voltage divider on ADC pin
  /*
  int adcValue = analogRead(BATTERY_ADC_PIN);
  float voltage = (adcValue / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
  return voltage;
  */
  
  // For testing, return simulated value
  return batteryVoltage;
}

void updateBatteryStatus() {
  static unsigned long lastBatteryCheck = 0;
  
  if (millis() - lastBatteryCheck > 60000) { // Check every minute
    batteryVoltage = readBatteryVoltage();
    lastBatteryCheck = millis();
    
    LOG_DEBUG(MODULE_SYS, "Battery voltage: %.2f V", batteryVoltage);
  }
}

void checkEmergencyMode() {
  // Auto-switch to emergency mode if battery is low
  if (batteryVoltage < 11.5 && currentPowerMode != POWER_MODE_EMERGENCY) {
    LOG_WARN(MODULE_SYS, "Low battery detected! Switching to emergency mode");
    setPowerMode(POWER_MODE_EMERGENCY);
  }
  
  // Return to standby mode if battery recovers
  if (batteryVoltage > 12.0 && currentPowerMode == POWER_MODE_EMERGENCY) {
    LOG_INFO(MODULE_SYS, "Battery recovered, switching to standby mode");
    setPowerMode(POWER_MODE_STANDBY);
  }
}

// ----- HELPER FUNCTIONS -----
const char* getPowerModeString(PowerMode mode) {
  switch (mode) {
    case POWER_MODE_FULL: return "FULL";
    case POWER_MODE_STANDBY: return "STANDBY";
    case POWER_MODE_EMERGENCY: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

void printPowerModeInfo() {
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  LOG_INFO(MODULE_MAIN, "=== POWER MODE: %s ===", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "GPS Interval: %lu ms", config.gpsInterval);
  LOG_INFO(MODULE_MAIN, "WS KeepAlive: %lu ms", config.wsKeepAliveInterval);
  LOG_INFO(MODULE_MAIN, "Sleep Duration: %lu ms", config.sleepDuration);
  LOG_INFO(MODULE_MAIN, "GPS Always On: %s", config.gpsAlwaysOn ? "Yes" : "No");
  LOG_INFO(MODULE_MAIN, "WS Continuous: %s", config.wsContinuous ? "Yes" : "No");
  LOG_INFO(MODULE_MAIN, "Relay Enabled: %s", config.relayEnabled ? "Yes" : "No");
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
    } else if (cmd == "full") {
      setPowerMode(POWER_MODE_FULL);
    } else if (cmd == "standby") {
      setPowerMode(POWER_MODE_STANDBY);
    } else if (cmd == "emergency") {
      setPowerMode(POWER_MODE_EMERGENCY);
    } else if (cmd == "power") {
      printPowerModeInfo();
    } else if (cmd == "battery") {
      LOG_INFO(MODULE_MAIN, "Battery voltage: %.2f V", batteryVoltage);
    } else if (cmd.startsWith("setbat ")) {
      // For testing: set simulated battery voltage
      batteryVoltage = cmd.substring(7).toFloat();
      LOG_INFO(MODULE_MAIN, "Battery voltage set to: %.2f V", batteryVoltage);
    } else if (cmd == "on") {
      if (powerConfigs[currentPowerMode].relayEnabled) {
        onRelayUpdate(true);
      } else {
        LOG_WARN(MODULE_MAIN, "Relay control disabled in %s mode", 
                 getPowerModeString(currentPowerMode));
      }
    } else if (cmd == "off") {
      if (powerConfigs[currentPowerMode].relayEnabled) {
        onRelayUpdate(false);
      } else {
        LOG_WARN(MODULE_MAIN, "Relay control disabled in %s mode", 
                 getPowerModeString(currentPowerMode));
      }
    } else if (cmd == "reset") {
      ESP.restart();
    } else {
      LOG_WARN(MODULE_MAIN, "Unknown command: %s", cmd.c_str());
    }
  }
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "help       - Show this help");
  LOG_INFO(MODULE_MAIN, "status     - Show system status");
  LOG_INFO(MODULE_MAIN, "power      - Show current power mode info");
  LOG_INFO(MODULE_MAIN, "full       - Switch to FULL power mode");
  LOG_INFO(MODULE_MAIN, "standby    - Switch to STANDBY mode");
  LOG_INFO(MODULE_MAIN, "emergency  - Switch to EMERGENCY mode");
  LOG_INFO(MODULE_MAIN, "battery    - Show battery voltage");
  LOG_INFO(MODULE_MAIN, "setbat X.X - Set battery voltage (testing)");
  LOG_INFO(MODULE_MAIN, "on/off     - Relay control");
  LOG_INFO(MODULE_MAIN, "reset      - Restart system");
}

void printStatus() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM STATUS ===");
  LOG_INFO(MODULE_MAIN, "Power Mode: %s", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "Battery: %.2f V", batteryVoltage);
  LOG_INFO(MODULE_MAIN, "State: %d", currentState);
  LOG_INFO(MODULE_MAIN, "Uptime: %s", Utils::formatUptime(millis()).c_str());
  
  // GPS Status
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "GPS: Valid (%.6f, %.6f)",
             gpsManager.getLatitude(), gpsManager.getLongitude());
  } else {
    LOG_INFO(MODULE_MAIN, "GPS: No fix");
  }
  
  // Relay Status
  LOG_INFO(MODULE_MAIN, "Relay: %s", relayState ? "ON" : "OFF");
  
  // Connection Status
  LOG_INFO(MODULE_MAIN, "GPRS: %s", modemManager.isGprsConnected() ? "Connected" : "Disconnected");
  LOG_INFO(MODULE_MAIN, "WebSocket: %s", wsManager.getStateString());
}

// ----- OTHER FUNCTIONS (same as before) -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "Initializing system...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "System initialization successful");
    
    if (powerConfigs[currentPowerMode].wsContinuous) {
      if (wsManager.connect()) {
        LOG_INFO(MODULE_SYS, "WebSocket connected successfully");
      } else {
        LOG_WARN(MODULE_SYS, "WebSocket connection failed, will retry...");
      }
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
    
    if (powerConfigs[currentPowerMode].wsContinuous) {
      if (wsManager.connect()) {
        LOG_INFO(MODULE_SYS, "WebSocket reconnected");
        currentState = STATE_OPERATIONAL;
        lastSuccessfulOperation = millis();
      } else {
        LOG_ERROR(MODULE_SYS, "WebSocket reconnection failed");
        currentState = STATE_MODEM_RESET;
      }
    } else {
      currentState = STATE_OPERATIONAL;
    }
  } else {
    LOG_ERROR(MODULE_SYS, "Connection recovery failed, trying modem reset");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

bool sendVehicleDataViaWebSocket() {
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "No GPRS connection");
    return false;
  }
  
  // Connect WebSocket if needed (non-continuous mode)
  if (!powerConfigs[currentPowerMode].wsContinuous) {
    if (!wsManager.isReady()) {
      LOG_INFO(MODULE_WS, "Connecting WebSocket for data transmission...");
      if (!wsManager.connect()) {
        LOG_ERROR(MODULE_WS, "Failed to connect WebSocket");
        return false;
      }
      delay(2000); // Wait for connection to stabilize
    }
  }
  
  LOG_INFO(MODULE_GPS, "Sending vehicle data [%s mode]...", 
           getPowerModeString(currentPowerMode));
  
  // Prepare timestamp
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  
  // Convert to ISO format
  String isoTimestamp = String(timestamp);
  if (isoTimestamp.endsWith("Z")) {
    isoTimestamp.replace("Z", ".000Z");
  }
  
  bool success = false;
  if (gpsManager.isValid()) {
    success = wsManager.sendVehicleData(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      gpsManager.getSpeed(),
      gpsManager.getSatellites(),
      isoTimestamp
    );
    
    if (success) {
      LOG_INFO(MODULE_GPS, "✅ Vehicle data sent successfully");
    }
  } else {
    LOG_WARN(MODULE_GPS, "GPS data not valid");
  }
  
  // Disconnect WebSocket if non-continuous mode
  if (!powerConfigs[currentPowerMode].wsContinuous && success) {
    delay(1000); // Wait for data to be sent
    LOG_INFO(MODULE_WS, "Disconnecting WebSocket (non-continuous mode)");
    wsManager.disconnect();
  }
  
  return success;
}

void onRelayUpdate(bool newState) {
  if (!powerConfigs[currentPowerMode].relayEnabled) {
    LOG_WARN(MODULE_RELAY, "Relay control disabled in %s mode", 
             getPowerModeString(currentPowerMode));
    return;
  }
  
  if (newState != relayState) {
    LOG_INFO(MODULE_RELAY, "🔄 Relay update: %s → %s", 
             relayState ? "ON" : "OFF", newState ? "ON" : "OFF");
    
    digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
    relayState = newState;
    
    LOG_INFO(MODULE_RELAY, "✅ Physical relay updated to: %s", newState ? "ON" : "OFF");
  }
}