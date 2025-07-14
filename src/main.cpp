// ========================================
// Main.cpp - Sistem GPS Tracker dengan Performance Optimization
// ========================================

/**
 * ESP32 Vehicle GPS Tracking dengan Low Latency Optimization
 * - Performance monitoring terintegrasi
 * - Network optimization otomatis
 * - Adaptive transmission intervals
 * - Fast error recovery
 * - Movement state detection (MOVING/PARKED/STATIC)
 * 
 * Versi: 7.0 - Enhanced Movement Detection with Manual Speed Testing
 * Update: Fixed all function declarations and implementations
 */

// ----- FRAMEWORK ARDUINO -----
#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <time.h>

// ----- DEFINISI UNTUK MODEM -----
#define SerialMon Serial
#define SerialAT Serial1

// ----- LIBRARY EKSTERNAL -----
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// ----- MODUL CUSTOM -----
#include "Config.h"
#include "Logger.h"
#include "Utils.h"
#include "GpsManager.h"
#include "ModemManager.h"
#include "WebSocketManager.h"

// ========================================
// KONSTANTA DAN ENUMERASI
// ========================================

// ----- MODE POWER -----
enum PowerMode {
  POWER_MODE_FULL,      // Mode Aktif Penuh - optimized for low latency
  POWER_MODE_STANDBY,   // Mode Standby - balanced performance
  POWER_MODE_EMERGENCY  // Mode Darurat - ultra hemat daya
};

// ----- STATE SISTEM -----
enum SystemState {
  STATE_INIT,                 // Inisialisasi sistem
  STATE_WAIT_GPS,             // Menunggu GPS fix
  STATE_OPERATIONAL,          // Operasional normal
  STATE_MODEM_RESET,         // Reset modem dalam proses
  STATE_CONNECTION_RECOVERY,  // Pemulihan koneksi
  STATE_ERROR,               // Error state
  STATE_SLEEP_PREPARE,       // Persiapan sleep
  STATE_SLEEPING,            // Dalam mode sleep
  STATE_OPTIMIZING           // Applying network optimizations
};

// ----- STATE PERGERAKAN (UPDATED) -----
enum MovementState {
  MOVEMENT_UNKNOWN,   // Status belum diketahui
  MOVEMENT_STATIC,    // Kendaraan diam (> 5 menit)
  MOVEMENT_PARKED,    // Kendaraan parkir (0-5 menit)
  MOVEMENT_MOVING     // Kendaraan bergerak
};

// ----- MOVEMENT INTERVALS (NEW) -----
#define GPS_INTERVAL_MOVING 3000          // 3 detik saat bergerak
#define GPS_INTERVAL_PARKED 15000         // 15 detik saat parkir
#define GPS_INTERVAL_STATIC 60000         // 60 detik saat diam
#define PARKED_TO_STATIC_TIMEOUT 300000   // 5 menit threshold parkir ke diam

// ========================================
// STRUKTUR DATA
// ========================================

// ----- PERFORMANCE METRICS -----
struct PerformanceMetrics {
  unsigned long totalTransmissions;
  unsigned long successfulTransmissions;
  unsigned long failedTransmissions;
  unsigned long totalLatency;
  unsigned long minLatency;
  unsigned long maxLatency;
  int consecutiveSlowTransmissions;
  int consecutiveFailures;
  unsigned long lastOptimizationTime;
  unsigned long lastPerformanceReport;
  
  void reset() {
    totalTransmissions = 0;
    successfulTransmissions = 0;
    failedTransmissions = 0;
    totalLatency = 0;
    minLatency = UINT32_MAX;
    maxLatency = 0;
    consecutiveSlowTransmissions = 0;
    consecutiveFailures = 0;
    lastOptimizationTime = 0;
    lastPerformanceReport = 0;
  }
} performanceMetrics;

// ----- SYSTEM INITIALIZATION FLAGS -----
struct SystemReadyFlags {
  bool gpsReady;
  bool modemReady;
  bool networkReady;
  bool gprsReady;
  bool wsReady;
  bool firstGpsFix;
  unsigned long gpsFirstFixTime;
  unsigned long systemStartTime;
  
  void reset() {
    gpsReady = false;
    modemReady = false;
    networkReady = false;
    gprsReady = false;
    wsReady = false;
    firstGpsFix = false;
    gpsFirstFixTime = 0;
    systemStartTime = millis();
  }
  
  bool isReady() const {
    return gpsReady && modemReady && networkReady && gprsReady;
  }
} systemFlags;

// ----- POWER MODE CONFIGURATION -----
struct PowerModeConfig {
  unsigned long gpsInterval;
  unsigned long wsKeepAliveInterval;
  unsigned long sleepDuration;
  bool gpsAlwaysOn;
  bool wsContinuous;
  bool relayEnabled;
  bool performanceMonitoring;
  bool aggressiveOptimization;
};

// Konfigurasi untuk setiap mode power
const PowerModeConfig powerConfigs[3] = {
  // POWER_MODE_FULL
  {2000, WS_PING_INTERVAL, 0, true, true, true, true, true},
  // POWER_MODE_STANDBY  
  {30000, 60000, 0, true, true, true, false, false},
  // POWER_MODE_EMERGENCY
  {300000, 0, 0, true, false, false, false, false}
};

// ========================================
// OBJEK GLOBAL
// ========================================

// GPS dan Komunikasi
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);

// Managers
GpsManager gpsManager(gps, SerialGPS);
ModemManager modemManager(modem, SerialAT);
WebSocketManager wsManager(&gsmClient);

// ========================================
// VARIABEL GLOBAL
// ========================================

// State Variables
SystemState currentState = STATE_INIT;
PowerMode currentPowerMode = POWER_MODE_FULL;
MovementState currentMovementState = MOVEMENT_UNKNOWN;

// Timing Variables
unsigned long lastGpsSendTime = 0;
unsigned long lastSuccessfulOperation = 0;
unsigned long lastActivityTime = 0;
unsigned long lastSignalCheck = 0;
unsigned long lastMaintenanceCheck = 0;
unsigned long lastPerformanceOptimization = 0;
unsigned long vehicleStopTime = 0;          // NEW: Track when vehicle stopped

// Control Variables
bool relayState = true;
bool waitForSubscription = true;
float batteryVoltage = 12.6;

// Testing Variables (NEW)
float manualSpeed = -1.0;                   // Manual speed for testing
bool useManualSpeed = false;                // Flag to use manual speed

// ========================================
// DEKLARASI FUNGSI
// ========================================

// State Handlers
void handleInitState();
void handleWaitGpsState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSleepPrepareState();
void handleOptimizingState();
void executeStateMachine();

// Serial Command Handlers
void handleSerialCommands();
void processSpeedCommand(const String& cmd);
void processTestingCommand(const String& cmd);

// Status and Info Functions
void printStatus();
void printHelp();
void printPowerModeInfo();
void printWebSocketStats();
void printPerformanceReport();
void printSystemReadyStatus();
void printMovementInfo();
void showSpeedInfo();
void showBatteryInfo();
void showGpsDetails();

// Data Transmission
bool sendVehicleDataViaWebSocket();
void onRelayUpdate(bool newState);
void forceSendGpsData();

// Power Management
void setPowerMode(PowerMode mode);
void enterLightSleep(unsigned long duration);
void enterDeepSleep(unsigned long duration);
void disableUnusedPeripherals();
void enablePeripherals();
void setRelay(bool state);

// System Monitoring
float readBatteryVoltage();
void updateBatteryStatus();
void checkEmergencyMode();
void updateMovementState();
void checkSignalQuality();
void performanceOptimizationCheck();
void updatePerformanceMetrics(bool success, unsigned long latency);
void checkNetworkHealth();
void applyPerformanceOptimizations();
void setBatteryVoltage(float voltage);

// Movement Functions
void logMovementStateChange(float speed);
void logMovementStatus(float speed);

// Connection Management
void maintainWebSocketConnection();
void maintainModemConnection();
void connectWebSocket();
void waitForWebSocketSubscription();
bool ensureWebSocketSubscribed();
void checkGpsReady();
void logGpsNotReady();
void checkWebSocketConnection();
void transmitGpsData(unsigned long currentTime, unsigned long interval);
void checkSleepConditions(unsigned long currentTime);
void checkConnectionHealth(unsigned long currentTime);
void resetWebSocketConnection();

// Utility Functions
unsigned long getGpsIntervalForMovement();
float getCurrentSpeed();
const char* getPowerModeString(PowerMode mode);
const char* getStateString(SystemState state);
const char* getMovementString(MovementState movement);
String formatTimestamp(unsigned long unixTime);
bool isSystemReady();

// ========================================
// SETUP
// ========================================
void setup() {
  // Initialize Serial
  SerialMon.begin(115200);
  SerialMon.setRxBufferSize(1024);
  delay(100);
  
  // Initialize Logger
  #ifdef DEBUG_MODE
    Logger::init(&SerialMon, LOG_DEBUG);
  #else
    Logger::init(&SerialMon, LOG_INFO);
  #endif
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS Tracker v7.0 ===");
  LOG_INFO(MODULE_MAIN, "Enhanced Movement Detection with Manual Speed Testing");
  LOG_INFO(MODULE_MAIN, "Device ID: %s", GPS_ID);
  LOG_INFO(MODULE_MAIN, "Compiled: %s %s", __DATE__, __TIME__);
  
  // Initialize system flags
  systemFlags.reset();
  performanceMetrics.reset();
  
  // Initialize hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  LOG_INFO(MODULE_RELAY, "Relay initialized: ON");
  
  #if ENABLE_BATTERY_MONITORING
    pinMode(BATTERY_ADC_PIN, INPUT);
    LOG_INFO(MODULE_SYS, "Battery monitoring: ENABLED");
  #endif
  
  // Initialize watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  LOG_INFO(MODULE_SYS, "Watchdog timer: %d seconds", WATCHDOG_TIMEOUT/1000);
  
  // Initialize GPS
  LOG_INFO(MODULE_MAIN, "Initializing GPS...");
  gpsManager.begin();
  
  #ifdef GPS_UPDATE_RATE
    gpsManager.enableHighUpdateRate(GPS_UPDATE_RATE);
    LOG_INFO(MODULE_GPS, "GPS update rate: %d Hz", GPS_UPDATE_RATE);
  #endif
  
  // Initialize modem hardware
  LOG_INFO(MODULE_MAIN, "Initializing modem hardware...");
  modemManager.begin();
  
  // Initialize WebSocket manager
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  // Show initial info
  printHelp();
  Utils::printMemoryInfo();
  
  // Set initial state
  currentState = STATE_WAIT_GPS;
  lastActivityTime = millis();
  
  LOG_INFO(MODULE_MAIN, "Setup complete, waiting for GPS fix...");
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  // Essential updates
  Utils::feedWatchdog();
  updateBatteryStatus();
  checkEmergencyMode();
  
  // Always update GPS
  gpsManager.update();
  
  // Update movement state
  if (systemFlags.gpsReady || useManualSpeed) {
    updateMovementState();
  }
  
  // System monitoring (only if ready)
  if (systemFlags.modemReady) {
    checkSignalQuality();
    checkNetworkHealth();
  }
  
  if (systemFlags.networkReady) {
    performanceOptimizationCheck();
  }
  
  // WebSocket updates
  if (currentState == STATE_OPERATIONAL && powerConfigs[currentPowerMode].wsContinuous) {
    wsManager.update();
    maintainWebSocketConnection();
  }
  
  // Modem maintenance
  if (currentState == STATE_OPERATIONAL && systemFlags.modemReady) {
    maintainModemConnection();
  }
  
  // Handle serial commands
  handleSerialCommands();
  
  // State machine
  executeStateMachine();
  
  delay(5); // Small delay for stability
}

// ========================================
// STATE MACHINE
// ========================================
void executeStateMachine() {
  switch (currentState) {
    case STATE_WAIT_GPS:
      handleWaitGpsState();
      break;
    case STATE_INIT:
      handleInitState();
      break;
    case STATE_OPERATIONAL:
      handleOperationalState();
      break;
    case STATE_MODEM_RESET:
      handleModemResetState();
      break;
    case STATE_CONNECTION_RECOVERY:
      handleConnectionRecoveryState();
      break;
    case STATE_OPTIMIZING:
      handleOptimizingState();
      break;
    case STATE_SLEEP_PREPARE:
      handleSleepPrepareState();
      break;
    case STATE_ERROR:
      LOG_ERROR(MODULE_SYS, "System error, attempting recovery...");
      modemManager.startReset();
      currentState = STATE_MODEM_RESET;
      break;
    default:
      LOG_WARN(MODULE_SYS, "Unknown state: %d", currentState);
      currentState = STATE_OPERATIONAL;
      break;
  }
}

// ========================================
// STATE HANDLERS
// ========================================

void handleWaitGpsState() {
  static unsigned long gpsWaitStart = millis();
  static unsigned long lastGpsStatusLog = 0;
  unsigned long elapsed = millis() - gpsWaitStart;
  
  // Log GPS status periodically
  if (millis() - lastGpsStatusLog >= GPS_LOG_INTERVAL) {
    LOG_INFO(MODULE_GPS, "⏳ Waiting GPS: %lu s, Sats: %d, HDOP: %.1f", 
             elapsed / 1000, gpsManager.getSatellites(), gpsManager.getHDOP());
    lastGpsStatusLog = millis();
  }
  
  // Check GPS fix
  if (gpsManager.getSatellites() >= GPS_MIN_SATELLITES && 
      gpsManager.isValid() && 
      gpsManager.getHDOP() <= GPS_MAX_HDOP) {
    
    systemFlags.gpsReady = true;
    systemFlags.firstGpsFix = true;
    systemFlags.gpsFirstFixTime = millis();
    
    LOG_INFO(MODULE_GPS, "✅ GPS FIX ACQUIRED!");
    LOG_INFO(MODULE_GPS, "📍 Position: %.6f, %.6f", 
             gpsManager.getLatitude(), gpsManager.getLongitude());
    LOG_INFO(MODULE_GPS, "⏱️ Time to fix: %lu seconds", elapsed / 1000);
    
    currentState = STATE_INIT;
    return;
  }
  
  // Check timeout
  if (elapsed >= GPS_WAIT_TIMEOUT) {
    LOG_WARN(MODULE_GPS, "⚠️ GPS timeout, continuing without fix");
    systemFlags.gpsReady = false;
    currentState = STATE_INIT;
  }
}

void handleInitState() {
  LOG_INFO(MODULE_SYS, "🚀 Initializing system...");
  
  // Setup modem
  if (modemManager.setup()) {
    systemFlags.modemReady = true;
    systemFlags.networkReady = modemManager.isNetworkConnected();
    systemFlags.gprsReady = modemManager.isGprsConnected();
    
    // Apply optimizations
    if (!modemManager.areOptimizationsApplied()) {
      modemManager.applyNetworkOptimizations();
    }
    
    // Check GPS again
    if (!systemFlags.gpsReady && gpsManager.isValid()) {
      systemFlags.gpsReady = true;
      LOG_INFO(MODULE_GPS, "✅ GPS ready during modem setup");
    }
    
    // Connect WebSocket if needed
    if (powerConfigs[currentPowerMode].wsContinuous) {
      connectWebSocket();
    }
    
    printSystemReadyStatus();
    currentState = STATE_OPERATIONAL;
    lastSuccessfulOperation = millis();
    
    if (systemFlags.gpsReady && systemFlags.wsReady) {
      lastGpsSendTime = 0; // Force immediate send
    }
    
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Modem initialization failed");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleOperationalState() {
  unsigned long currentTime = millis();
  
  // Check if GPS became ready
  checkGpsReady();
  
  // Skip if GPS not ready
  if (!systemFlags.gpsReady && !useManualSpeed) {
    logGpsNotReady();
    return;
  }
  
  // Check WebSocket connection
  checkWebSocketConnection();
  
  // Get appropriate GPS interval
  unsigned long gpsInterval = getGpsIntervalForMovement();
  
  // Send GPS data if interval reached
  if (currentTime - lastGpsSendTime >= gpsInterval) {
    transmitGpsData(currentTime, gpsInterval);
  }
  
  // Check for sleep mode
  checkSleepConditions(currentTime);
  
  // Check connection health
  checkConnectionHealth(currentTime);
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
    // Reset complete
    if (modemManager.setup()) {
      LOG_INFO(MODULE_SYS, "✅ Modem reset successful");
      systemFlags.modemReady = true;
      
      if (powerConfigs[currentPowerMode].aggressiveOptimization) {
        modemManager.applyNetworkOptimizations();
      }
      
      wsManager.resetReconnectAttempts();
      performanceMetrics.consecutiveFailures = 0;
      
      currentState = STATE_OPERATIONAL;
    } else {
      LOG_ERROR(MODULE_SYS, "❌ Modem reset failed");
      currentState = STATE_ERROR;
    }
  }
}

void handleConnectionRecoveryState() {
  LOG_INFO(MODULE_SYS, "🔄 Attempting connection recovery...");
  
  // Disconnect everything
  wsManager.disconnect();
  modemManager.disconnectGprs();
  Utils::safeDelay(1000);
  
  // Reconnect GPRS
  if (modemManager.connectGprs()) {
    systemFlags.gprsReady = true;
    
    if (!modemManager.areOptimizationsApplied()) {
      modemManager.forceOptimizationReapply();
    }
    
    wsManager.resetReconnectAttempts();
    
    // Reconnect WebSocket if needed
    if (powerConfigs[currentPowerMode].wsContinuous) {
      connectWebSocket();
    }
    
    if (systemFlags.wsReady || !powerConfigs[currentPowerMode].wsContinuous) {
      currentState = STATE_OPERATIONAL;
      lastSuccessfulOperation = millis();
      performanceMetrics.consecutiveFailures = 0;
    } else {
      currentState = STATE_MODEM_RESET;
    }
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Recovery failed, resetting modem");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleOptimizingState() {
  LOG_INFO(MODULE_SYS, "🔧 Applying performance optimizations...");
  
  applyPerformanceOptimizations();
  
  performanceMetrics.consecutiveSlowTransmissions = 0;
  performanceMetrics.consecutiveFailures = 0;
  
  currentState = STATE_OPERATIONAL;
  LOG_INFO(MODULE_SYS, "✅ Optimization complete");
}

void handleSleepPrepareState() {
  LOG_INFO(MODULE_SYS, "😴 Preparing for sleep mode...");
  
  const PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  // Log stats before sleep
  if (config.performanceMonitoring) {
    printPerformanceReport();
  }
  
  // Disconnect if needed
  if (!config.wsContinuous) {
    wsManager.disconnect();
  }
  
  // Enter appropriate sleep mode
  if (config.sleepDuration > 0) {
    if (currentPowerMode == POWER_MODE_EMERGENCY) {
      enterDeepSleep(config.sleepDuration);
    } else {
      enterLightSleep(config.sleepDuration);
    }
  }
  
  // After wake
  LOG_INFO(MODULE_SYS, "⏰ Woke from sleep");
  currentState = STATE_OPERATIONAL;
  lastActivityTime = millis();
}

// ========================================
// MOVEMENT DETECTION (UPDATED)
// ========================================

float getCurrentSpeed() {
  return useManualSpeed ? manualSpeed : gpsManager.getSpeed();
}

void updateMovementState() {
  static unsigned long lastMovementLog = 0;
  MovementState previousState = currentMovementState;
  
  float currentSpeed = getCurrentSpeed();
  
  // Determine movement state dengan threshold baru
  if (currentSpeed > MOVEMENT_SPEED_THRESHOLD) {
    // Speed > 4 km/h = MOVING
    currentMovementState = MOVEMENT_MOVING;
    vehicleStopTime = 0;
  } else {
    // Speed <= 4 km/h
    if (vehicleStopTime == 0) {
      // Just stopped
      vehicleStopTime = millis();
      currentMovementState = MOVEMENT_PARKED;
    } else {
      // Already stopped
      unsigned long stopDuration = millis() - vehicleStopTime;
      
      if (stopDuration > PARKED_TO_STATIC_TIMEOUT) {
        // Stopped > 5 minutes = STATIC
        currentMovementState = MOVEMENT_STATIC;
      } else {
        // Stopped 0-5 minutes = PARKED
        currentMovementState = MOVEMENT_PARKED;
      }
    }
  }
  
  // Log state changes
  if (currentMovementState != previousState) {
    logMovementStateChange(currentSpeed);
  }
  
  // Periodic status log
  if (millis() - lastMovementLog > 30000) {
    logMovementStatus(currentSpeed);
    lastMovementLog = millis();
  }
}

void logMovementStateChange(float speed) {
  switch (currentMovementState) {
    case MOVEMENT_MOVING:
      LOG_INFO(MODULE_GPS, "🚗 Vehicle MOVING (%.1f km/h)", speed);
      LOG_INFO(MODULE_GPS, "📡 GPS interval: %d seconds", GPS_INTERVAL_MOVING/1000);
      break;
    case MOVEMENT_PARKED:
      LOG_INFO(MODULE_GPS, "🅿️ Vehicle PARKED (%.1f km/h)", speed);
      LOG_INFO(MODULE_GPS, "📡 GPS interval: %d seconds", GPS_INTERVAL_PARKED/1000);
      break;
    case MOVEMENT_STATIC:
      LOG_INFO(MODULE_GPS, "🛑 Vehicle STATIC (stopped > 5 min)");
      LOG_INFO(MODULE_GPS, "📡 GPS interval: %d minutes", GPS_INTERVAL_STATIC/60000);
      break;
    default:
      break;
  }
}

void logMovementStatus(float speed) {
  String status = String("Movement: ") + getMovementString(currentMovementState);
  status += " | Speed: " + String(speed, 1) + " km/h";
  
  if (useManualSpeed) {
    status += " (MANUAL)";
  }
  
  if (currentMovementState != MOVEMENT_MOVING && vehicleStopTime > 0) {
    unsigned long stopDuration = (millis() - vehicleStopTime) / 1000;
    status += " | Stop: " + String(stopDuration) + "s";
  }
  
  LOG_DEBUG(MODULE_GPS, status.c_str());
}

unsigned long getGpsIntervalForMovement() {
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    switch (currentMovementState) {
      case MOVEMENT_MOVING:
        return GPS_INTERVAL_MOVING;    // 3 seconds
      case MOVEMENT_PARKED:
        return GPS_INTERVAL_PARKED;    // 15 seconds
      case MOVEMENT_STATIC:
        return GPS_INTERVAL_STATIC;    // 60 seconds
      default:
        return powerConfigs[currentPowerMode].gpsInterval;
    }
  #else
    return powerConfigs[currentPowerMode].gpsInterval;
  #endif
}

// ========================================
// DATA TRANSMISSION
// ========================================

bool sendVehicleDataViaWebSocket() {
  // Pre-checks
  if (!systemFlags.gpsReady && !useManualSpeed) {
    LOG_ERROR(MODULE_GPS, "❌ GPS not ready");
    return false;
  }
  
  if (!systemFlags.modemReady || !systemFlags.gprsReady) {
    LOG_ERROR(MODULE_GPS, "❌ Modem/GPRS not ready");
    return false;
  }
  
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "❌ No GPRS connection");
    systemFlags.gprsReady = false;
    return false;
  }
  
  // Connect WebSocket if needed
  if (!powerConfigs[currentPowerMode].wsContinuous && !wsManager.isReady()) {
    LOG_INFO(MODULE_WS, "Connecting WebSocket for transmission...");
    if (!wsManager.connect()) {
      LOG_ERROR(MODULE_WS, "❌ WebSocket connection failed");
      systemFlags.wsReady = false;
      return false;
    }
    delay(1000);
  }
  
  // Wait for subscription if needed
  if (!ensureWebSocketSubscribed()) {
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "📤 Sending vehicle data [%s mode]...", 
           getMovementString(currentMovementState));
  
  // Send data
  bool success = false;
  
  if (gpsManager.isValid() || useManualSpeed) {
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    // Start latency measurement
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      wsManager.startLatencyMeasurement();
      modemManager.startLatencyMeasurement();
    }
    
    // Include manual speed in data if testing
    float displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
    
    success = wsManager.sendVehicleData(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      displaySpeed,
      gpsManager.getSatellites(),
      timestamp,
      batteryVoltage
    );
    
    if (success) {
      if (powerConfigs[currentPowerMode].performanceMonitoring) {
        modemManager.endLatencyMeasurement();
      }
      
      LOG_INFO(MODULE_GPS, "✅ Data sent successfully");
      LOG_INFO(MODULE_GPS, "📍 Pos: %.6f, %.6f | 🚗 %.1f km/h | 🛰️ %d sats",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               displaySpeed, gpsManager.getSatellites());
      
      if (useManualSpeed) {
        LOG_INFO(MODULE_GPS, "📏 Using manual speed: %.1f km/h", manualSpeed);
      }
    } else {
      LOG_ERROR(MODULE_GPS, "❌ Failed to send data");
    }
  } else {
    LOG_WARN(MODULE_GPS, "⚠️ Invalid GPS data");
  }
  
  // Disconnect WebSocket if non-continuous
  if (!powerConfigs[currentPowerMode].wsContinuous && success) {
    delay(500);
    wsManager.disconnect();
    systemFlags.wsReady = false;
  }
  
  return success;
}

// ========================================
// CONNECTION MANAGEMENT
// ========================================

void maintainWebSocketConnection() {
  static unsigned long lastWsMaintenance = 0;
  if (millis() - lastWsMaintenance > CONNECTION_HEALTH_CHECK_INTERVAL) {
    wsManager.maintainConnection();
    lastWsMaintenance = millis();
  }
}

void maintainModemConnection() {
  static unsigned long lastModemMaintenance = 0;
  if (millis() - lastModemMaintenance > ModemManager::MAINTENANCE_INTERVAL) {
    modemManager.maintainConnection();
    lastModemMaintenance = millis();
  }
}

void connectWebSocket() {
  LOG_INFO(MODULE_WS, "Connecting WebSocket...");
  if (wsManager.connect()) {
    waitForWebSocketSubscription();
  } else {
    LOG_WARN(MODULE_WS, "⚠️ WebSocket connection failed");
    systemFlags.wsReady = false;
  }
}

void waitForWebSocketSubscription() {
  LOG_INFO(MODULE_WS, "Waiting for subscription...");
  unsigned long start = millis();
  
  while (wsManager.getState() != WS_SUBSCRIBED && 
         millis() - start < WS_SUBSCRIPTION_TIMEOUT) {
    wsManager.update();
    Utils::feedWatchdog();
    delay(100);
  }
  
  systemFlags.wsReady = (wsManager.getState() == WS_SUBSCRIBED);
  LOG_INFO(MODULE_WS, systemFlags.wsReady ? 
           "✅ WebSocket subscribed" : "⚠️ Subscription timeout");
}

bool ensureWebSocketSubscribed() {
  if (!powerConfigs[currentPowerMode].wsContinuous || !waitForSubscription) {
    return true;
  }
  
  int retries = 0;
  while (wsManager.getState() != WS_SUBSCRIBED && retries < 30) {
    delay(100);
    wsManager.update();
    retries++;
  }
  
  if (wsManager.getState() != WS_SUBSCRIBED) {
    LOG_ERROR(MODULE_WS, "❌ Subscription timeout");
    systemFlags.wsReady = false;
    return false;
  }
  
  systemFlags.wsReady = true;
  return true;
}

void checkGpsReady() {
  static bool gpsReadyLogged = false;
  
  if (!systemFlags.gpsReady && gpsManager.isValid()) {
    systemFlags.gpsReady = true;
    if (!systemFlags.firstGpsFix) {
      systemFlags.firstGpsFix = true;
      systemFlags.gpsFirstFixTime = millis();
    }
    LOG_INFO(MODULE_GPS, "✅ GPS fix acquired!");
    gpsReadyLogged = false;
    lastGpsSendTime = 0; // Force immediate send
  }
}

void logGpsNotReady() {
  static unsigned long lastGpsLog = 0;
  static bool warned = false;
  
  if (!warned) {
    LOG_WARN(MODULE_GPS, "GPS not ready, waiting...");
    warned = true;
  }
  
  if (millis() - lastGpsLog > 10000) {
    LOG_INFO(MODULE_GPS, "GPS: %d sats, HDOP: %.1f", 
             gpsManager.getSatellites(), gpsManager.getHDOP());
    lastGpsLog = millis();
  }
}

void checkWebSocketConnection() {
  if (!systemFlags.wsReady && powerConfigs[currentPowerMode].wsContinuous) {
    if (wsManager.getState() == WS_DISCONNECTED) {
      static unsigned long lastWsReconnect = 0;
      if (millis() - lastWsReconnect > 30000) {
        LOG_INFO(MODULE_WS, "Attempting reconnection...");
        if (wsManager.connect()) {
          systemFlags.wsReady = (wsManager.getState() == WS_SUBSCRIBED);
        }
        lastWsReconnect = millis();
      }
    } else if (wsManager.getState() == WS_SUBSCRIBED) {
      systemFlags.wsReady = true;
    }
  }
}

void transmitGpsData(unsigned long currentTime, unsigned long interval) {
  #if TESTING_MODE && DEBUG_LATENCY_TRACKING
    LOG_INFO(MODULE_GPS, "⏱️ Transmit trigger: interval=%lu ms, state=%s", 
             interval, getMovementString(currentMovementState));
  #endif
  
  unsigned long transmissionStart = millis();
  bool success = sendVehicleDataViaWebSocket();
  
  if (success) {
    unsigned long latency = millis() - transmissionStart;
    updatePerformanceMetrics(true, latency);
    
    lastGpsSendTime = currentTime;
    lastSuccessfulOperation = currentTime;
    lastActivityTime = currentTime;
    
    wsManager.endLatencyMeasurement();
    
    if (powerConfigs[currentPowerMode].performanceMonitoring && 
        DEBUG_LATENCY_TRACKING) {
      LOG_DEBUG(MODULE_PERF, "📊 Transmission completed in %lu ms", latency);
    }
  } else {
    updatePerformanceMetrics(false, 0);
    performanceMetrics.consecutiveFailures++;
    
    if (performanceMetrics.consecutiveFailures >= MAX_CONNECTION_FAILURES) {
      LOG_WARN(MODULE_PERF, "🔧 Multiple failures, triggering optimization");
      currentState = STATE_OPTIMIZING;
    }
  }
}

void checkSleepConditions(unsigned long currentTime) {
  const PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  if (config.sleepDuration > 0 && 
      currentTime - lastActivityTime > ACTIVITY_TIMEOUT) {
    LOG_INFO(MODULE_SYS, "No activity, preparing for sleep");
    currentState = STATE_SLEEP_PREPARE;
  }
}

void checkConnectionHealth(unsigned long currentTime) {
  if (currentTime - lastSuccessfulOperation > 180000) { // 3 minutes
    LOG_WARN(MODULE_SYS, "⚠️ No successful operations in 3 min");
    currentState = STATE_CONNECTION_RECOVERY;
  }
}

void resetWebSocketConnection() {
  LOG_INFO(MODULE_MAIN, "Resetting WebSocket connection...");
  wsManager.disconnect();
  systemFlags.wsReady = false;
  delay(500);
  wsManager.resetReconnectAttempts();
  
  if (powerConfigs[currentPowerMode].wsContinuous) {
    connectWebSocket();
  }
}

// ========================================
// SERIAL COMMAND HANDLERS
// ========================================

void handleSerialCommands() {
  if (!SerialMon.available()) return;
  
  String cmd = SerialMon.readStringUntil('\n');
  cmd.trim();
  
  if (cmd.length() == 0) return;
  
  LOG_DEBUG(MODULE_MAIN, "Command: %s", cmd.c_str());
  
  // Basic commands
  if (cmd == "help") {
    printHelp();
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "init") {
    printSystemReadyStatus();
  } else if (cmd == "reset") {
    LOG_WARN(MODULE_MAIN, "Restarting system...");
    printPerformanceReport();
    delay(1000);
    ESP.restart();
  }
  // Power mode commands
  else if (cmd == "full") {
    setPowerMode(POWER_MODE_FULL);
  } else if (cmd == "standby") {
    setPowerMode(POWER_MODE_STANDBY);
  } else if (cmd == "emergency") {
    setPowerMode(POWER_MODE_EMERGENCY);
  } else if (cmd == "power") {
    printPowerModeInfo();
  }
  // Speed and movement commands
  else if (cmd.startsWith("speed")) {
    processSpeedCommand(cmd);
  } else if (cmd == "movement") {
    printMovementInfo();
  }
  // Testing commands
  else if (cmd == "send") {
    forceSendGpsData();
  } else if (cmd.startsWith("test")) {
    processTestingCommand(cmd);
  }
  // Network commands
  else if (cmd == "network") {
    LOG_INFO(MODULE_MAIN, modemManager.getNetworkInfo().c_str());
  } else if (cmd == "optimize") {
    LOG_INFO(MODULE_MAIN, "🔧 Starting manual optimization...");
    currentState = STATE_OPTIMIZING;
  }
  // WebSocket commands
  else if (cmd == "wsstats") {
    printWebSocketStats();
  } else if (cmd == "wsreset") {
    resetWebSocketConnection();
  }
  // System commands
  else if (cmd == "battery") {
    showBatteryInfo();
  } else if (cmd.startsWith("setbat ")) {
    setBatteryVoltage(cmd.substring(7).toFloat());
  } else if (cmd == "memory") {
    Utils::printMemoryInfo();
  } else if (cmd == "latency") {
    printPerformanceReport();
  }
  // Relay commands
  else if (cmd == "on") {
    setRelay(true);
  } else if (cmd == "off") {
    setRelay(false);
  }
  // GPS commands
  else if (cmd == "gps") {
    showGpsDetails();
  }
  // Unknown command
  else {
    LOG_WARN(MODULE_MAIN, "Unknown command: %s", cmd.c_str());
  }
  
  lastActivityTime = millis();
}

void processSpeedCommand(const String& cmd) {
  if (cmd == "speed") {
    // Show current speed info
    showSpeedInfo();
  } else if (cmd == "speed auto") {
    // Return to GPS speed
    useManualSpeed = false;
    manualSpeed = -1.0;
    LOG_INFO(MODULE_MAIN, "📍 Returned to automatic GPS speed");
    LOG_INFO(MODULE_MAIN, "Current GPS speed: %.1f km/h", gpsManager.getSpeed());
    updateMovementState();
  } else if (cmd.startsWith("speed ")) {
    // Set manual speed
    float speed = cmd.substring(6).toFloat();
    if (speed >= 0 && speed <= 200) {
      manualSpeed = speed;
      useManualSpeed = true;
      LOG_INFO(MODULE_MAIN, "📏 Manual speed set to: %.1f km/h", manualSpeed);
      LOG_INFO(MODULE_MAIN, "Use 'speed auto' to return to GPS speed");
      
      updateMovementState();
      
      LOG_INFO(MODULE_MAIN, "Movement state: %s", getMovementString(currentMovementState));
      LOG_INFO(MODULE_MAIN, "GPS interval: %lu seconds", getGpsIntervalForMovement()/1000);
    } else {
      LOG_WARN(MODULE_MAIN, "Invalid speed (0-200 km/h)");
    }
  }
}

void processTestingCommand(const String& cmd) {
  if (cmd == "teststop") {
    // Test stop transition
    LOG_INFO(MODULE_MAIN, "🧪 Starting stop transition test...");
    manualSpeed = 0.0;
    useManualSpeed = true;
    updateMovementState();
    LOG_INFO(MODULE_MAIN, "Speed set to 0, will transition PARKED → STATIC after 5 min");
  } else if (cmd == "testmove") {
    // Test movement transitions
    LOG_INFO(MODULE_MAIN, "🧪 Testing movement transitions...");
    
    // Moving
    manualSpeed = 10.0;
    useManualSpeed = true;
    updateMovementState();
    LOG_INFO(MODULE_MAIN, "1. MOVING: Speed 10 km/h, interval %ds", GPS_INTERVAL_MOVING/1000);
    delay(2000);
    
    // Parked
    manualSpeed = 0.5;
    updateMovementState();
    LOG_INFO(MODULE_MAIN, "2. PARKED: Speed 0.5 km/h, interval %ds", GPS_INTERVAL_PARKED/1000);
    delay(2000);
    
    // Static (simulate)
    manualSpeed = 0.0;
    vehicleStopTime = millis() - PARKED_TO_STATIC_TIMEOUT - 1000;
    updateMovementState();
    LOG_INFO(MODULE_MAIN, "3. STATIC: Speed 0 km/h, interval %ds", GPS_INTERVAL_STATIC/1000);
  } else if (cmd == "testreset") {
    // Reset test variables
    useManualSpeed = false;
    manualSpeed = -1.0;
    vehicleStopTime = 0;
    LOG_INFO(MODULE_MAIN, "Test variables reset");
    updateMovementState();
  }
}

void showSpeedInfo() {
  LOG_INFO(MODULE_MAIN, "=== SPEED INFO ===");
  
  if (useManualSpeed) {
    LOG_INFO(MODULE_MAIN, "Mode: MANUAL");
    LOG_INFO(MODULE_MAIN, "Manual Speed: %.1f km/h", manualSpeed);
  } else {
    LOG_INFO(MODULE_MAIN, "Mode: AUTOMATIC (GPS)");
  }
  
  LOG_INFO(MODULE_MAIN, "GPS Speed: %.1f km/h", gpsManager.getSpeed());
  LOG_INFO(MODULE_MAIN, "Movement State: %s", getMovementString(currentMovementState));
  LOG_INFO(MODULE_MAIN, "GPS Interval: %lu seconds", getGpsIntervalForMovement()/1000);
  
  if (vehicleStopTime > 0 && currentMovementState != MOVEMENT_MOVING) {
    unsigned long stopDuration = (millis() - vehicleStopTime) / 1000;
    LOG_INFO(MODULE_MAIN, "Stop Duration: %lu seconds", stopDuration);
    
    if (currentMovementState == MOVEMENT_PARKED) {
      unsigned long remaining = (PARKED_TO_STATIC_TIMEOUT - (millis() - vehicleStopTime)) / 1000;
      LOG_INFO(MODULE_MAIN, "Time to STATIC: %lu seconds", remaining);
    }
  }
}

void forceSendGpsData() {
  LOG_INFO(MODULE_MAIN, "🚀 Force sending GPS data...");
  
  if (!systemFlags.gpsReady && !useManualSpeed) {
    LOG_ERROR(MODULE_MAIN, "❌ Cannot send - GPS not ready");
    return;
  }
  
  if (!systemFlags.modemReady) {
    LOG_ERROR(MODULE_MAIN, "❌ Cannot send - Modem not ready");
    return;
  }
  
  if (!systemFlags.gprsReady) {
    LOG_ERROR(MODULE_MAIN, "❌ Cannot send - GPRS not connected");
    return;
  }
  
  if (sendVehicleDataViaWebSocket()) {
    LOG_INFO(MODULE_MAIN, "✅ Data sent successfully");
  } else {
    LOG_ERROR(MODULE_MAIN, "❌ Failed to send data");
  }
}

void showBatteryInfo() {
  float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                     (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  percentage = constrain(percentage, 0.0, 100.0);
  LOG_INFO(MODULE_MAIN, "Battery: %.2fV (%.0f%%) - %s", 
           batteryVoltage, percentage, Utils::getBatteryStatus(batteryVoltage));
}

void setBatteryVoltage(float voltage) {
  batteryVoltage = voltage;
  LOG_INFO(MODULE_MAIN, "Battery voltage set to: %.2fV", batteryVoltage);
}

void showGpsDetails() {
  LOG_INFO(MODULE_MAIN, "=== GPS DETAILS ===");
  LOG_INFO(MODULE_MAIN, "Ready        : %s", systemFlags.gpsReady ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Valid        : %s", gpsManager.isValid() ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Satellites   : %d", gpsManager.getSatellites());
  LOG_INFO(MODULE_MAIN, "HDOP         : %.1f", gpsManager.getHDOP());
  
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "Position     : %.6f, %.6f", 
             gpsManager.getLatitude(), gpsManager.getLongitude());
    LOG_INFO(MODULE_MAIN, "Speed        : %.1f km/h", gpsManager.getSpeed());
    LOG_INFO(MODULE_MAIN, "Altitude     : %.1f m", gpsManager.getAltitude());
    LOG_INFO(MODULE_MAIN, "Heading      : %.1f°", gpsManager.getHeading());
  }
  
  if (systemFlags.firstGpsFix) {
    unsigned long ttff = (systemFlags.gpsFirstFixTime - systemFlags.systemStartTime) / 1000;
    LOG_INFO(MODULE_MAIN, "First Fix    : %lu seconds", ttff);
  }
}

// ========================================
// MONITORING FUNCTIONS
// ========================================

void checkSignalQuality() {
  if (millis() - lastSignalCheck > SIGNAL_QUALITY_CHECK_INTERVAL) {
    int signal = modemManager.getSignalQuality();
    
    if (modemManager.isSignalWeak()) {
      LOG_WARN(MODULE_MODEM, "⚠️ Weak signal: %d", signal);
      
      if (signal < SIGNAL_WEAK_THRESHOLD && 
          powerConfigs[currentPowerMode].aggressiveOptimization) {
        LOG_WARN(MODULE_MODEM, "🔧 Very weak signal, optimizing");
        currentState = STATE_OPTIMIZING;
      }
    }
    
    lastSignalCheck = millis();
  }
}

void checkNetworkHealth() {
  if (millis() - lastMaintenanceCheck < CONNECTION_HEALTH_CHECK_INTERVAL) {
    return;
  }
  
  lastMaintenanceCheck = millis();
  
  systemFlags.networkReady = modemManager.isNetworkConnected();
  systemFlags.gprsReady = modemManager.isGprsConnected();
  
  if (!modemManager.isGprsConnected() && currentState == STATE_OPERATIONAL) {
    LOG_WARN(MODULE_SYS, "⚠️ GPRS disconnected");
    systemFlags.gprsReady = false;
    currentState = STATE_CONNECTION_RECOVERY;
  }
  
  if (performanceMetrics.consecutiveSlowTransmissions >= 3) {
    if (!modemManager.performNetworkDiagnostic()) {
      LOG_WARN(MODULE_SYS, "❌ Network diagnostic failed");
      currentState = STATE_OPTIMIZING;
    }
  }
}

void performanceOptimizationCheck() {
  if (!powerConfigs[currentPowerMode].aggressiveOptimization) {
    return;
  }
  
  if (millis() - lastPerformanceOptimization < AUTO_OPTIMIZATION_INTERVAL) {
    return;
  }
  
  lastPerformanceOptimization = millis();
  
  if (!modemManager.areOptimizationsApplied()) {
    LOG_WARN(MODULE_PERF, "🔧 Reapplying optimizations");
    modemManager.reapplyOptimizations();
  }
  
  int signal = modemManager.getSignalQuality();
  if (signal < SIGNAL_WEAK_THRESHOLD && signal != 99) {
    LOG_WARN(MODULE_PERF, "🔧 Weak signal optimization");
    modemManager.reapplyOptimizations();
  }
  
  if (performanceMetrics.totalTransmissions > 10) {
    unsigned long avgLatency = performanceMetrics.totalLatency / 
                              performanceMetrics.totalTransmissions;
    if (avgLatency > MAX_ACCEPTABLE_LATENCY) {
      LOG_WARN(MODULE_PERF, "🔧 High latency optimization");
      applyPerformanceOptimizations();
    }
  }
}

void applyPerformanceOptimizations() {
  LOG_INFO(MODULE_PERF, "🚀 Applying comprehensive optimizations...");
  
  if (!modemManager.areOptimizationsApplied()) {
    modemManager.forceOptimizationReapply();
  }
  
  if (modemManager.isGprsConnected()) {
    modemManager.connectGprs();
  }
  
  performanceMetrics.consecutiveSlowTransmissions = 0;
  performanceMetrics.consecutiveFailures = 0;
  
  modemManager.logOptimizationDetails();
  
  LOG_INFO(MODULE_PERF, "✅ Optimizations complete");
}

void updatePerformanceMetrics(bool success, unsigned long latency) {
  performanceMetrics.totalTransmissions++;
  
  if (success) {
    performanceMetrics.successfulTransmissions++;
    performanceMetrics.consecutiveFailures = 0;
    
    if (latency > 0) {
      performanceMetrics.totalLatency += latency;
      if (latency < performanceMetrics.minLatency) {
        performanceMetrics.minLatency = latency;
      }
      if (latency > performanceMetrics.maxLatency) {
        performanceMetrics.maxLatency = latency;
      }
      
      if (latency > LATENCY_WARNING_THRESHOLD) {
        performanceMetrics.consecutiveSlowTransmissions++;
      } else {
        performanceMetrics.consecutiveSlowTransmissions = 0;
      }
    }
  } else {
    performanceMetrics.failedTransmissions++;
  }
}

// ========================================
// POWER MANAGEMENT
// ========================================

void setPowerMode(PowerMode mode) {
  if (mode == currentPowerMode) return;
  
  LOG_INFO(MODULE_SYS, "🔄 Power mode: %s → %s", 
           getPowerModeString(currentPowerMode), 
           getPowerModeString(mode));
  
  currentPowerMode = mode;
  const PowerModeConfig& config = powerConfigs[mode];
  
  if (config.performanceMonitoring) {
    wsManager.resetPerformanceStats();
    modemManager.resetPerformanceStats();
  }
  
  if (config.aggressiveOptimization) {
    applyPerformanceOptimizations();
  }
  
  printPowerModeInfo();
}

void updateBatteryStatus() {
  static unsigned long lastBatteryCheck = 0;
  
  #if ENABLE_BATTERY_MONITORING
    if (millis() - lastBatteryCheck > BATTERY_READ_INTERVAL) {
      batteryVoltage = readBatteryVoltage();
      lastBatteryCheck = millis();
    }
  #endif
}

float readBatteryVoltage() {
  #if ENABLE_BATTERY_MONITORING
    int adcValue = analogRead(BATTERY_ADC_PIN);
    float voltage = (adcValue / 4095.0) * 3.3 * BATTERY_VOLTAGE_DIVIDER_RATIO * BATTERY_CALIBRATION_FACTOR;
    return voltage;
  #else
    return batteryVoltage;
  #endif
}

void checkEmergencyMode() {
  if (batteryVoltage < BATTERY_LOW_THRESHOLD && 
      currentPowerMode != POWER_MODE_EMERGENCY) {
    LOG_WARN(MODULE_SYS, "⚠️ Low battery! Switching to emergency mode");
    setPowerMode(POWER_MODE_EMERGENCY);
  }
  
  if (batteryVoltage > BATTERY_RECOVERY_THRESHOLD && 
      currentPowerMode == POWER_MODE_EMERGENCY) {
    LOG_INFO(MODULE_SYS, "✅ Battery recovered, switching to standby");
    setPowerMode(POWER_MODE_STANDBY);
  }
}

void enterLightSleep(unsigned long duration) {
  esp_sleep_enable_timer_wakeup(duration * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

void enterDeepSleep(unsigned long duration) {
  esp_sleep_enable_timer_wakeup(duration * 1000);
  esp_deep_sleep_start();
}

void disableUnusedPeripherals() {
  esp_wifi_stop();
  esp_bt_controller_disable();
  LOG_DEBUG(MODULE_SYS, "Peripherals disabled");
}

void enablePeripherals() {
  LOG_DEBUG(MODULE_SYS, "Peripherals enabled");
}

// ========================================
// RELAY CONTROL
// ========================================

void onRelayUpdate(bool newState) {
  if (!powerConfigs[currentPowerMode].relayEnabled) {
    LOG_WARN(MODULE_RELAY, "⚠️ Relay control disabled in %s mode", 
             getPowerModeString(currentPowerMode));
    return;
  }
  
  if (newState != relayState) {
    LOG_INFO(MODULE_RELAY, "🔄 Relay update: %s → %s", 
             relayState ? "ON" : "OFF", newState ? "ON" : "OFF");
    
    digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
    relayState = newState;
    
    LOG_INFO(MODULE_RELAY, "✅ Relay updated: %s", newState ? "ON" : "OFF");
    lastActivityTime = millis();
  }
}

void setRelay(bool state) {
  if (powerConfigs[currentPowerMode].relayEnabled) {
    onRelayUpdate(state);
  } else {
    LOG_WARN(MODULE_MAIN, "Relay control disabled in %s mode", 
             getPowerModeString(currentPowerMode));
  }
}

// ========================================
// STATUS DISPLAY FUNCTIONS
// ========================================

void printHelp() {
  SerialMon.println("\n========== COMMAND HELP ==========");
  SerialMon.println("=== BASIC COMMANDS ===");
  SerialMon.println("help         - Show this help");
  SerialMon.println("status       - Show system status");
  SerialMon.println("init         - Show initialization status");
  SerialMon.println("reset        - Restart system");
  
  SerialMon.println("\n=== POWER MODES ===");
  SerialMon.println("power        - Show current power mode");
  SerialMon.println("full         - Switch to FULL mode");
  SerialMon.println("standby      - Switch to STANDBY mode");
  SerialMon.println("emergency    - Switch to EMERGENCY mode");
  
  SerialMon.println("\n=== SPEED & MOVEMENT ===");
  SerialMon.println("speed        - Show speed info");
  SerialMon.println("speed N      - Set manual speed (0-200 km/h)");
  SerialMon.println("speed auto   - Return to GPS speed");
  SerialMon.println("movement     - Show movement details");
  
  SerialMon.println("\n=== TESTING COMMANDS ===");
  SerialMon.println("send         - Force send GPS data");
  SerialMon.println("teststop     - Test PARKED→STATIC transition");
  SerialMon.println("testmove     - Test all movement modes");
  SerialMon.println("testreset    - Reset test variables");
  
  SerialMon.println("\n=== NETWORK & OPTIMIZATION ===");
  SerialMon.println("network      - Show network info");
  SerialMon.println("optimize     - Apply optimizations");
  SerialMon.println("latency      - Show performance report");
  
  SerialMon.println("\n=== OTHER COMMANDS ===");
  SerialMon.println("battery      - Show battery status");
  SerialMon.println("memory       - Show memory info");
  SerialMon.println("gps          - Show GPS details");
  SerialMon.println("on/off       - Control relay");
  
  SerialMon.println("==================================\n");
}

void printStatus() {
  SerialMon.println("\n========== SYSTEM STATUS ==========");
  
  // System state
  SerialMon.printf("System State : %s\n", getStateString(currentState));
  SerialMon.printf("Power Mode   : %s\n", getPowerModeString(currentPowerMode));
  SerialMon.printf("System Ready : %s\n", isSystemReady() ? "YES ✅" : "NO ⚠️");
  
  if (!isSystemReady()) {
    if (!systemFlags.gpsReady) SerialMon.println("  - GPS not ready");
    if (!systemFlags.modemReady) SerialMon.println("  - Modem not ready");
    if (!systemFlags.networkReady) SerialMon.println("  - Network not ready");
    if (!systemFlags.gprsReady) SerialMon.println("  - GPRS not ready");
  }
  
  // Movement info
  SerialMon.printf("Movement     : %s", getMovementString(currentMovementState));
  if (useManualSpeed) {
    SerialMon.printf(" [MANUAL: %.1f km/h]", manualSpeed);
  }
  SerialMon.println();
  
  if (vehicleStopTime > 0 && currentMovementState != MOVEMENT_MOVING) {
    unsigned long stopDuration = (millis() - vehicleStopTime) / 1000;
    SerialMon.printf("Stop Time    : %lu seconds", stopDuration);
    
    if (currentMovementState == MOVEMENT_PARKED) {
      unsigned long remaining = (PARKED_TO_STATIC_TIMEOUT - (millis() - vehicleStopTime)) / 1000;
      SerialMon.printf(" (→ STATIC in %lu s)", remaining);
    }
    SerialMon.println();
  }
  
  SerialMon.printf("GPS Interval : %lu seconds", getGpsIntervalForMovement()/1000);
  switch(currentMovementState) {
    case MOVEMENT_MOVING:
      SerialMon.print(" (MOVING)");
      break;
    case MOVEMENT_PARKED:
      SerialMon.print(" (PARKED)");
      break;
    case MOVEMENT_STATIC:
      SerialMon.print(" (STATIC)");
      break;
    default:
      break;
  }
  SerialMon.println();
  
  // GPS info
  if (gpsManager.isValid()) {
    SerialMon.printf("GPS          : VALID (%.6f, %.6f)\n",
                     gpsManager.getLatitude(), gpsManager.getLongitude());
    SerialMon.printf("Speed        : %.1f km/h%s\n", 
                     getCurrentSpeed(), 
                     useManualSpeed ? " (MANUAL)" : " (GPS)");
    SerialMon.printf("Satellites   : %d\n", gpsManager.getSatellites());
    SerialMon.printf("HDOP         : %.1f\n", gpsManager.getHDOP());
  } else {
    SerialMon.printf("GPS          : NO FIX (Sats: %d)\n", 
                     gpsManager.getSatellites());
  }
  
  // Network info
  SerialMon.printf("Modem        : %s", modemManager.getStatusString());
  if (modemManager.areOptimizationsApplied()) {
    SerialMon.print(" [OPT]");
  }
  SerialMon.println();
  
  SerialMon.printf("Signal       : %d (%s)\n", 
                   modemManager.getSignalQuality(),
                   Utils::getSignalQualityString(modemManager.getSignalQuality()));
  
  SerialMon.printf("WebSocket    : %s\n", wsManager.getStateString());
  
  // Battery
  float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                     (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  percentage = constrain(percentage, 0.0, 100.0);
  SerialMon.printf("Battery      : %.2fV (%.0f%%)\n", batteryVoltage, percentage);
  
  // Performance
  if (performanceMetrics.totalTransmissions > 0) {
    SerialMon.printf("Transmissions: %lu/%lu (%.1f%%)\n",
                     performanceMetrics.successfulTransmissions,
                     performanceMetrics.totalTransmissions,
                     (performanceMetrics.successfulTransmissions * 100.0 / 
                      performanceMetrics.totalTransmissions));
  }
  
  SerialMon.printf("Uptime       : %s\n", Utils::formatUptime(millis()).c_str());
  SerialMon.printf("Free Memory  : %u KB\n", Utils::getFreeHeap() / 1024);
  
  SerialMon.println("===================================\n");
}

void printSystemReadyStatus() {
  LOG_INFO(MODULE_SYS, "=== SYSTEM READY STATUS ===");
  LOG_INFO(MODULE_SYS, "GPS Ready    : %s", systemFlags.gpsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Modem Ready  : %s", systemFlags.modemReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Network Ready: %s", systemFlags.networkReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "GPRS Ready   : %s", systemFlags.gprsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "WebSocket    : %s", systemFlags.wsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Overall      : %s", isSystemReady() ? "READY ✅" : "NOT READY ⚠️");
}

void printMovementInfo() {
  LOG_INFO(MODULE_MAIN, "=== MOVEMENT DETAILS ===");
  LOG_INFO(MODULE_MAIN, "State        : %s", getMovementString(currentMovementState));
  LOG_INFO(MODULE_MAIN, "Speed        : %.1f km/h%s", 
           getCurrentSpeed(), useManualSpeed ? " (MANUAL)" : "");
  LOG_INFO(MODULE_MAIN, "GPS Interval : %lu seconds", getGpsIntervalForMovement()/1000);
  
  if (vehicleStopTime > 0 && currentMovementState != MOVEMENT_MOVING) {
    unsigned long stopDuration = (millis() - vehicleStopTime) / 1000;
    LOG_INFO(MODULE_MAIN, "Stop Duration: %lu seconds", stopDuration);
    
    if (currentMovementState == MOVEMENT_PARKED) {
      unsigned long remaining = (PARKED_TO_STATIC_TIMEOUT - (millis() - vehicleStopTime)) / 1000;
      LOG_INFO(MODULE_MAIN, "Time to STATIC: %lu seconds", remaining);
    }
  }
  
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "Distance from last: %.1f m", 
             gpsManager.getDistanceFromLastPosition());
    LOG_INFO(MODULE_MAIN, "Time since movement: %lu s", 
             gpsManager.getTimeSinceLastMovement() / 1000);
  }
}

void printPowerModeInfo() {
  const PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  SerialMon.println("\n====== POWER MODE INFO ======");
  SerialMon.printf("Mode         : %s\n", getPowerModeString(currentPowerMode));
  SerialMon.printf("GPS Always On: %s\n", config.gpsAlwaysOn ? "Yes" : "No");
  SerialMon.printf("WS Continuous: %s\n", config.wsContinuous ? "Yes" : "No");
  SerialMon.printf("Relay Enabled: %s\n", config.relayEnabled ? "Yes" : "No");
  SerialMon.printf("Performance  : %s\n", config.performanceMonitoring ? "Yes" : "No");
  SerialMon.printf("Optimization : %s\n", config.aggressiveOptimization ? "Yes" : "No");
  
  SerialMon.println("\n=== MOVEMENT INTERVALS ===");
  SerialMon.printf("MOVING       : %d seconds (speed > %.1f km/h)\n", GPS_INTERVAL_MOVING/1000, float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("PARKED       : %d seconds (speed 0-%.1f km/h)\n", GPS_INTERVAL_PARKED/1000, float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("STATIC       : %d minutes (parked > 5 min)\n", GPS_INTERVAL_STATIC/60000);
  SerialMon.printf("Speed Thresh : %.1f km/h\n", float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("Park→Static  : %d minutes\n", PARKED_TO_STATIC_TIMEOUT/60000);
  
  SerialMon.println("=============================\n");
}

void printWebSocketStats() {
  LOG_INFO(MODULE_WS, wsManager.getPerformanceReport().c_str());
}

void printPerformanceReport() {
  LOG_INFO(MODULE_PERF, "=== PERFORMANCE REPORT ===");
  LOG_INFO(MODULE_PERF, "Total TX     : %lu", performanceMetrics.totalTransmissions);
  LOG_INFO(MODULE_PERF, "Successful   : %lu (%.1f%%)", 
           performanceMetrics.successfulTransmissions,
           (performanceMetrics.totalTransmissions > 0) ? 
           (performanceMetrics.successfulTransmissions * 100.0 / 
            performanceMetrics.totalTransmissions) : 0);
  
  if (performanceMetrics.successfulTransmissions > 0) {
    unsigned long avgLatency = performanceMetrics.totalLatency / 
                              performanceMetrics.successfulTransmissions;
    LOG_INFO(MODULE_PERF, "Avg Latency  : %lu ms", avgLatency);
    LOG_INFO(MODULE_PERF, "Min Latency  : %lu ms", performanceMetrics.minLatency);
    LOG_INFO(MODULE_PERF, "Max Latency  : %lu ms", performanceMetrics.maxLatency);
    LOG_INFO(MODULE_PERF, "Performance  : %s", 
             avgLatency <= MAX_ACCEPTABLE_LATENCY ? "✅ GOOD" : "⚠️ NEEDS IMPROVEMENT");
  }
  
  LOG_INFO(MODULE_PERF, modemManager.getPerformanceReport().c_str());
  LOG_INFO(MODULE_PERF, wsManager.getPerformanceReport().c_str());
}

// ========================================
// HELPER FUNCTIONS
// ========================================

const char* getPowerModeString(PowerMode mode) {
  switch (mode) {
    case POWER_MODE_FULL: return "FULL";
    case POWER_MODE_STANDBY: return "STANDBY";
    case POWER_MODE_EMERGENCY: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

const char* getStateString(SystemState state) {
  switch (state) {
    case STATE_INIT: return "INIT";
    case STATE_WAIT_GPS: return "WAIT_GPS";
    case STATE_OPERATIONAL: return "OPERATIONAL";
    case STATE_MODEM_RESET: return "MODEM_RESET";
    case STATE_CONNECTION_RECOVERY: return "RECOVERY";
    case STATE_ERROR: return "ERROR";
    case STATE_SLEEP_PREPARE: return "SLEEP_PREPARE";
    case STATE_SLEEPING: return "SLEEPING";
    case STATE_OPTIMIZING: return "OPTIMIZING";
    default: return "UNKNOWN";
  }
}

const char* getMovementString(MovementState movement) {
  switch (movement) {
    case MOVEMENT_UNKNOWN: return "UNKNOWN";
    case MOVEMENT_STATIC: return "STATIC";
    case MOVEMENT_PARKED: return "PARKED";
    case MOVEMENT_MOVING: return "MOVING";
    default: return "INVALID";
  }
}

bool isSystemReady() {
  return systemFlags.isReady();
}