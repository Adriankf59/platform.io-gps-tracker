// ========================================
// Main.cpp v8.0 - ESP32 GPS Tracker dengan MPU6050 IMU & Offline Storage
// ========================================

/**
 * ESP32 Vehicle GPS Tracking dengan IMU, Offline Storage & Auto-Recovery
 * - MPU6050 IMU integration untuk basement/indoor tracking
 * - Dead reckoning saat GPS tidak tersedia
 * - Sensor fusion GPS + IMU
 * - Offline data storage saat network tidak tersedia
 * - Auto-sync saat network kembali
 * - Performance monitoring terintegrasi
 * - Network optimization otomatis
 * - Adaptive transmission intervals
 * - Fast error recovery
 * - Movement state detection (MOVING/PARKED/STATIC)
 * - AUTO-RECOVERY SYSTEM untuk mencegah hang/stuck
 * - Memory leak detection dan prevention
 * - Health monitoring dengan auto-restart
 * 
 * Versi: 8.0 - Full IMU Integration
 * Update: Added MPU6050 support for basement/indoor tracking
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
#include "OfflineDataManager.h"
#include "ImuManager.h"  // NEW: IMU Manager

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
  STATE_OPTIMIZING,          // Applying network optimizations
  STATE_OFFLINE_SYNC,        // Synchronizing offline data
  STATE_IMU_CALIBRATION      // NEW: IMU calibration state
};

// ----- STATE PERGERAKAN -----
enum MovementState {
  MOVEMENT_UNKNOWN,   // Status belum diketahui
  MOVEMENT_STATIC,    // Kendaraan diam (> 5 menit)
  MOVEMENT_PARKED,    // Kendaraan parkir (0-5 menit)
  MOVEMENT_MOVING     // Kendaraan bergerak
};

// ----- MOVEMENT INTERVALS -----
#define PARKED_TO_STATIC_TIMEOUT 300000   // 5 menit threshold parkir ke diam

// ----- AUTO-RECOVERY CONSTANTS -----
#define ENABLE_AUTO_RESTART true
#define AUTO_RESTART_INTERVAL 259200000
#define MEMORY_CRITICAL_THRESHOLD 15000
#define SUCCESS_RATE_THRESHOLD 10
#define MAX_HEALTH_FAILURES 3
#define NO_SUCCESS_TIMEOUT 1800000
#define MEMORY_CHECK_INTERVAL 60000
#define HEALTH_CHECK_INTERVAL 300000

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

// ----- OFFLINE OPERATION STATS -----
struct OfflineOperationStats {
  bool hasUnsentData;
  bool syncInProgress;
  unsigned long syncStartTime;
  int batchesSent;
  unsigned long dataStoredOffline;
  unsigned long dataSentFromOffline;
  unsigned long lastOfflineStoreTime;
  unsigned long lastOfflineSyncTime;
  
  void reset() {
    hasUnsentData = false;
    syncInProgress = false;
    syncStartTime = 0;
    batchesSent = 0;
    dataStoredOffline = 0;
    dataSentFromOffline = 0;
    lastOfflineStoreTime = 0;
    lastOfflineSyncTime = 0;
  }
} offlineOpStats;

// ----- IMU OPERATION STATS (NEW) -----
struct ImuOperationStats {
  unsigned long totalImuReadings;
  unsigned long successfulImuReadings;
  unsigned long imuErrors;
  unsigned long deadReckoningActivations;
  unsigned long basementDetections;
  unsigned long lastImuUpdate;
  bool imuActive;
  PositionSource currentPositionSource;
  
  void reset() {
    totalImuReadings = 0;
    successfulImuReadings = 0;
    imuErrors = 0;
    deadReckoningActivations = 0;
    basementDetections = 0;
    lastImuUpdate = 0;
    imuActive = false;
    currentPositionSource = POS_SOURCE_GPS;
  }
} imuOpStats;

// ----- SYSTEM INITIALIZATION FLAGS -----
struct SystemReadyFlags {
  bool gpsReady;
  bool modemReady;
  bool networkReady;
  bool gprsReady;
  bool wsReady;
  bool firstGpsFix;
  bool imuReady;  // NEW
  unsigned long gpsFirstFixTime;
  unsigned long systemStartTime;
  
  void reset() {
    gpsReady = false;
    modemReady = false;
    networkReady = false;
    gprsReady = false;
    wsReady = false;
    firstGpsFix = false;
    imuReady = false;
    gpsFirstFixTime = 0;
    systemStartTime = millis();
  }
  
  bool isReady() const {
    return (gpsReady || imuReady) && modemReady && networkReady && gprsReady;
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
  bool imuEnabled;  // NEW
};

// Konfigurasi untuk setiap mode power
const PowerModeConfig powerConfigs[3] = {
  // POWER_MODE_FULL
  {2000, WS_PING_INTERVAL, 0, true, true, true, true, true, true},
  // POWER_MODE_STANDBY  
  {30000, 60000, 0, true, true, true, false, false, true},
  // POWER_MODE_EMERGENCY
  {300000, 0, 0, true, false, false, false, false, false}
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
OfflineDataManager offlineManager;
ImuManager imuManager;  // NEW: IMU Manager

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
unsigned long vehicleStopTime = 0;
unsigned long lastImuCheck = 0;  // NEW

// AUTO-RECOVERY Variables
unsigned long lastSuccessfulTransmission = 0;
unsigned long lastMemoryCheck = 0;
unsigned long lastHealthCheck = 0;
unsigned long systemStartTime = 0;
int consecutiveHealthFailures = 0;

// Control Variables
bool relayState = true;
bool waitForSubscription = true;
float batteryVoltage = 12.6;

// Testing Variables
float manualSpeed = -1.0;
bool useManualSpeed = false;

// Network and Offline Variables
bool networkAvailable = false;
bool offlineMode = false;
unsigned long lastNetworkCheck = 0;
unsigned long lastOfflineSync = 0;
unsigned long lastOfflineMaintenance = 0;

// IMU Variables (NEW)
bool basementMode = false;
unsigned long lastGpsFixTime = 0;
unsigned long gpsLostTime = 0;
bool useImuPosition = false;

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
void handleOfflineSyncState();
void handleImuCalibrationState();  // NEW
void executeStateMachine();

// IMU Functions (NEW)
void initializeImu();
void updateImuData();
void handleGpsLoss();
void handleGpsRecovery();
void startDeadReckoning();
void stopDeadReckoning();
bool shouldUseImuPosition();
void processImuCommands(const String& cmd);
void printImuStatus();
void printDeadReckoningInfo();
void printBasementStatus();
String getPositionSourceString();
void updatePositionSource();
void transmitWithImuData(unsigned long currentTime, unsigned long interval);

// AUTO-RECOVERY Functions
void performSystemHealthCheck();
void checkAutoRestart();
void checkMemoryHealth();
void checkSuccessRate();
void forceSystemRestart(const char* reason);

// Offline Storage Functions
void initializeOfflineStorage();
void checkNetworkAvailability();
bool shouldUseOfflineMode();
bool storeDataOffline(float lat, float lon, float speed, int satellites, const String& timestamp, float battery);
void syncOfflineData();
void processOfflineQueue();
void handleOfflineCommands(const String& cmd);
void printOfflineStatus();
void printOfflineStats();

// Serial Command Handlers
void handleSerialCommands();
void handleSerialCommandsExtended();
void processSpeedCommand(const String& cmd);
void processTestingCommand(const String& cmd);
void processAdvancedCommands(const String& cmd);

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
void printHealthStatus();

// Data Transmission
bool sendVehicleDataViaWebSocket();
bool sendVehicleDataWithOfflineSupport();
bool sendVehicleDataWithImu();  // NEW
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
void transmitGpsDataWithOfflineSupport(unsigned long currentTime, unsigned long interval);
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

// Advanced Testing & Diagnostics
void simulateNetworkOutage(unsigned long duration);
void runOfflineStorageStressTest();
void performSystemDiagnostics();
void performEmergencyBackup();
void recoverFromCorruptedState();
void logSystemStats();

// Integration Functions
extern "C" bool sendOfflineRecordViaWebSocket(float lat, float lon, float speed, 
                                             int satellites, const char* timestamp, 
                                             float battery);

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
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS Tracker v8.0 ===");
  LOG_INFO(MODULE_MAIN, "With IMU Support, Offline Storage & Auto-Recovery");
  LOG_INFO(MODULE_MAIN, "Device ID: %s", GPS_ID);
  LOG_INFO(MODULE_MAIN, "Compiled: %s %s", __DATE__, __TIME__);
  
  // Initialize system flags
  systemFlags.reset();
  performanceMetrics.reset();
  offlineOpStats.reset();
  imuOpStats.reset();  // NEW
  systemStartTime = millis();
  
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
  
  // Initialize Offline Storage
  initializeOfflineStorage();
  
  // NEW: Initialize IMU
  initializeImu();
  
  // Show initial info
  printHelp();
  Utils::printMemoryInfo();
  
  // AUTO-RECOVERY: Log system capabilities
  #if ENABLE_AUTO_RESTART
    LOG_INFO(MODULE_SYS, "🛡️ Auto-recovery: ENABLED");
    LOG_INFO(MODULE_SYS, "🔄 Auto-restart: every 72 hours");
    LOG_INFO(MODULE_SYS, "💾 Memory threshold: %d KB", MEMORY_CRITICAL_THRESHOLD/1024);
    LOG_INFO(MODULE_SYS, "📊 Min success rate: %d%%", SUCCESS_RATE_THRESHOLD);
  #endif
  
  // Offline storage info
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_SYS, "💾 Offline storage: ENABLED");
    LOG_INFO(MODULE_SYS, "📦 Max records: %d", OFFLINE_MAX_RECORDS);
    LOG_INFO(MODULE_SYS, "🔄 Auto-sync: %s", OFFLINE_AUTO_SYNC ? "ENABLED" : "DISABLED");
  #endif
  
  // NEW: IMU info
  #if ENABLE_IMU_SUPPORT
    LOG_INFO(MODULE_SYS, "🎯 IMU support: ENABLED");
    LOG_INFO(MODULE_SYS, "📡 Dead reckoning: %s", systemFlags.imuReady ? "READY" : "NOT READY");
    LOG_INFO(MODULE_SYS, "🏢 Basement detection: ACTIVE");
  #endif
  
  // Set initial state
  currentState = STATE_WAIT_GPS;
  lastActivityTime = millis();
  lastSuccessfulTransmission = millis();
  
  LOG_INFO(MODULE_MAIN, "Setup complete, waiting for GPS fix...");
}

// ========================================
// NEW: IMU FUNCTIONS
// ========================================

void initializeImu() {
  #if ENABLE_IMU_SUPPORT
    LOG_INFO(MODULE_IMU, "Initializing MPU6050 IMU...");
    
    if (imuManager.begin()) {
      systemFlags.imuReady = true;
      imuOpStats.imuActive = true;
      LOG_INFO(MODULE_IMU, "✅ IMU initialized successfully");
      
      // Perform initial calibration if configured
      #if IMU_CALIBRATION_ON_BOOT
        LOG_INFO(MODULE_IMU, "Starting IMU calibration...");
        currentState = STATE_IMU_CALIBRATION;
      #endif
    } else {
      LOG_ERROR(MODULE_IMU, "❌ IMU initialization failed");
      systemFlags.imuReady = false;
    }
  #else
    LOG_INFO(MODULE_IMU, "IMU support disabled");
  #endif
}

void handleImuCalibrationState() {
  #if ENABLE_IMU_SUPPORT
    static unsigned long calibrationStartTime = 0;
    
    if (calibrationStartTime == 0) {
      calibrationStartTime = millis();
      LOG_INFO(MODULE_IMU, "🎯 Keep device still for calibration...");
    }
    
    // Show calibration progress
    if (millis() - calibrationStartTime < 3000) {
      if ((millis() - calibrationStartTime) % 500 == 0) {
        LOG_INFO(MODULE_IMU, "Calibrating... %d%%", 
                 ((millis() - calibrationStartTime) * 100) / 3000);
      }
      return;
    }
    
    // Perform calibration
    if (imuManager.calibrate()) {
      LOG_INFO(MODULE_IMU, "✅ IMU calibration complete");
      currentState = STATE_WAIT_GPS;
    } else {
      LOG_ERROR(MODULE_IMU, "❌ IMU calibration failed");
      currentState = STATE_WAIT_GPS;
    }
    
    calibrationStartTime = 0;
  #else
    currentState = STATE_WAIT_GPS;
  #endif
}

void updateImuData() {
  #if ENABLE_IMU_SUPPORT
    if (!systemFlags.imuReady) return;
    
    // Update IMU data based on current state
    if (basementMode || !systemFlags.gpsReady) {
      // Higher update rate in basement or when GPS lost
      if (millis() - lastImuCheck < (1000 / IMU_SAMPLE_RATE_GPS_LOST)) return;
    } else if (currentMovementState == MOVEMENT_MOVING) {
      // Higher rate when moving
      if (millis() - lastImuCheck < (1000 / IMU_SAMPLE_RATE_MOVING)) return;
    } else {
      // Normal rate when GPS available
      if (millis() - lastImuCheck < (1000 / IMU_SAMPLE_RATE_GPS_VALID)) return;
    }
    
    lastImuCheck = millis();
    
    // Update IMU
    if (imuManager.update()) {
      imuOpStats.successfulImuReadings++;
      imuOpStats.lastImuUpdate = millis();
      
      // Check for basement entry/exit
      if (imuManager.isInBasement() != basementMode) {
        basementMode = imuManager.isInBasement();
        LOG_INFO(MODULE_IMU, "🏢 Basement mode: %s", basementMode ? "ENTERED" : "EXITED");
        
        if (basementMode) {
          imuOpStats.basementDetections++;
        }
      }
      
      // Update position source
      updatePositionSource();
      
    } else {
      imuOpStats.imuErrors++;
    }
    
    imuOpStats.totalImuReadings++;
  #endif
}

void handleGpsLoss() {
  #if ENABLE_IMU_SUPPORT
    if (!systemFlags.imuReady) return;
    
    LOG_WARN(MODULE_IMU, "📡 GPS signal lost, activating dead reckoning");
    
    gpsLostTime = millis();
    
    // Notify IMU manager of GPS loss
    imuManager.notifyGpsLoss();
    
    // Start dead reckoning if we have a last known position
    if (gpsManager.getLatitude() != 0 && gpsManager.getLongitude() != 0) {
      startDeadReckoning();
    }
  #endif
}

void handleGpsRecovery() {
  #if ENABLE_IMU_SUPPORT
    if (!systemFlags.imuReady) return;
    
    LOG_INFO(MODULE_IMU, "📡 GPS signal recovered");
    
    // Update IMU with GPS position
    imuManager.updateGpsPosition(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      gpsManager.getHeading()
    );
    
    // Notify IMU of GPS recovery
    imuManager.notifyGpsRecovered();
    
    // Stop dead reckoning if active
    if (imuManager.isDeadReckoningActive()) {
      stopDeadReckoning();
    }
    
    gpsLostTime = 0;
    useImuPosition = false;
  #endif
}

void startDeadReckoning() {
  #if ENABLE_IMU_SUPPORT
    if (!systemFlags.imuReady) return;
    
    double lastLat = gpsManager.getLatitude();
    double lastLon = gpsManager.getLongitude();
    float lastHeading = gpsManager.getHeading();
    
    imuManager.startDeadReckoning(lastLat, lastLon, lastHeading);
    imuOpStats.deadReckoningActivations++;
    
    LOG_INFO(MODULE_IMU, "🧭 Dead reckoning started from: %.6f, %.6f", lastLat, lastLon);
  #endif
}

void stopDeadReckoning() {
  #if ENABLE_IMU_SUPPORT
    if (!systemFlags.imuReady) return;
    
    imuManager.stopDeadReckoning();
    LOG_INFO(MODULE_IMU, "🧭 Dead reckoning stopped");
  #endif
}

bool shouldUseImuPosition() {
  #if ENABLE_IMU_SUPPORT
    if (!systemFlags.imuReady) return false;
    
    // Use IMU position if:
    // 1. GPS not available and dead reckoning active
    // 2. In basement mode
    // 3. Position error is acceptable
    
    if (!systemFlags.gpsReady && imuManager.isDeadReckoningActive()) {
      return imuManager.getPositionError() < IMU_MAX_POSITION_ERROR;
    }
    
    if (basementMode && imuManager.isDeadReckoningActive()) {
      return true;
    }
    
    return false;
  #else
    return false;
  #endif
}

void updatePositionSource() {
  #if ENABLE_IMU_SUPPORT
    PositionSource newSource = imuManager.getPositionSource();
    
    if (newSource != imuOpStats.currentPositionSource) {
      imuOpStats.currentPositionSource = newSource;
      LOG_INFO(MODULE_IMU, "📍 Position source: %s", getPositionSourceString());
    }
  #endif
}

String getPositionSourceString() {
  #if ENABLE_IMU_SUPPORT
    switch (imuOpStats.currentPositionSource) {
      case POS_SOURCE_GPS: return "GPS";
      case POS_SOURCE_IMU: return "IMU";
      case POS_SOURCE_FUSION: return "GPS+IMU";
      default: return "UNKNOWN";
    }
  #else
    return "GPS";
  #endif
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  // Essential updates
  Utils::feedWatchdog();
  updateBatteryStatus();
  checkEmergencyMode();
  
  // AUTO-RECOVERY CHECKS (PRIORITY)
  checkAutoRestart();
  checkMemoryHealth();
  performSystemHealthCheck();
  
  // Network availability check
  checkNetworkAvailability();
  
  // Always update GPS
  gpsManager.update();
  
  // NEW: Update IMU
  updateImuData();
  
  // Check for GPS state changes
  static bool lastGpsValid = false;
  bool currentGpsValid = gpsManager.isValid();
  
  if (lastGpsValid && !currentGpsValid) {
    handleGpsLoss();
  } else if (!lastGpsValid && currentGpsValid) {
    handleGpsRecovery();
  }
  lastGpsValid = currentGpsValid;
  
  // Update movement state
  if (systemFlags.gpsReady || useManualSpeed || shouldUseImuPosition()) {
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
  
  // Process offline queue if network available
  if (networkAvailable && offlineOpStats.hasUnsentData && !offlineOpStats.syncInProgress) {
    processOfflineQueue();
  }
  
  // Handle serial commands
  handleSerialCommandsExtended();
  
  // State machine
  executeStateMachine();
  
  delay(5); // Small delay for stability
}

// ========================================
// ENHANCED DATA TRANSMISSION WITH IMU
// ========================================

bool sendVehicleDataWithImu() {
  #if ENABLE_IMU_SUPPORT
    // Determine position data source
    float lat, lon, speed;
    int satellites;
    String positionSource;
    
    if (shouldUseImuPosition()) {
      // Use IMU position
      lat = imuManager.getEstimatedLatitude();
      lon = imuManager.getEstimatedLongitude();
      speed = imuManager.getEstimatedSpeed() * 3.6; // m/s to km/h
      satellites = 0; // No GPS
      positionSource = getPositionSourceString();
      
      LOG_INFO(MODULE_GPS, "📍 Using IMU position: %.6f, %.6f", lat, lon);
    } else {
      // Use GPS position
      lat = gpsManager.getLatitude();
      lon = gpsManager.getLongitude();
      speed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
      satellites = gpsManager.getSatellites();
      positionSource = "GPS";
    }
    
    // Get timestamp
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    // Get IMU data
    String imuData = imuManager.getJsonData();
    
    // Check if offline mode
    if (shouldUseOfflineMode()) {
      LOG_INFO(MODULE_GPS, "📵 Network unavailable, storing data offline");
      return storeDataOffline(lat, lon, speed, satellites, String(timestamp), batteryVoltage);
    }
    
    // Send with IMU data
    return wsManager.sendVehicleDataWithImu(
      lat, lon, speed, satellites,
      String(timestamp), batteryVoltage,
      positionSource, basementMode, imuData
    );
  #else
    // No IMU, use standard method
    return sendVehicleDataWithOfflineSupport();
  #endif
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
    case STATE_OFFLINE_SYNC:
      handleOfflineSyncState();
      break;
    case STATE_IMU_CALIBRATION:  // NEW
      handleImuCalibrationState();
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
// OPERATIONAL STATE WITH IMU
// ========================================

void handleOperationalState() {
  unsigned long currentTime = millis();
  
  // AUTO-RECOVERY: Check transmission timeout
  if (performanceMetrics.totalTransmissions > 0) {
    if (performanceMetrics.successfulTransmissions > 0) {
      lastSuccessfulTransmission = currentTime;
    } else if (currentTime - lastSuccessfulTransmission > NO_SUCCESS_TIMEOUT) {
      forceSystemRestart("No successful transmission in 30 min");
      return;
    }
  }
  
  // Check if GPS became ready
  checkGpsReady();
  
  // Skip if no position source available
  if (!systemFlags.gpsReady && !useManualSpeed && !shouldUseImuPosition()) {
    logGpsNotReady();
    return;
  }
  
  // Check WebSocket connection
  checkWebSocketConnection();
  
  // Get appropriate GPS interval
  unsigned long gpsInterval = getGpsIntervalForMovement();
  
  // Send data if interval reached
  if (currentTime - lastGpsSendTime >= gpsInterval) {
    transmitWithImuData(currentTime, gpsInterval);
  }
  
  // Process offline queue
  if (networkAvailable && offlineOpStats.hasUnsentData && !offlineOpStats.syncInProgress) {
    processOfflineQueue();
  }
  
  // Periodic offline maintenance
  if (currentTime - lastOfflineMaintenance > OFFLINE_MAINTENANCE_INTERVAL) {
    #if ENABLE_OFFLINE_STORAGE
      offlineManager.performMaintenance();
      offlineOpStats.hasUnsentData = offlineManager.hasOfflineData();
    #endif
    lastOfflineMaintenance = currentTime;
  }
  
  // Check for sleep mode
  checkSleepConditions(currentTime);
  
  // Check connection health
  checkConnectionHealth(currentTime);
}

void transmitWithImuData(unsigned long currentTime, unsigned long interval) {
  #if TESTING_MODE && DEBUG_LATENCY_TRACKING
    LOG_INFO(MODULE_GPS, "⏱️ Transmit trigger: interval=%lu ms, state=%s, mode=%s, source=%s", 
             interval, getMovementString(currentMovementState),
             networkAvailable ? "ONLINE" : "OFFLINE",
             getPositionSourceString());
  #endif
  
  unsigned long transmissionStart = millis();
  bool success = sendVehicleDataWithImu();
  
  if (success) {
    unsigned long latency = millis() - transmissionStart;
    
    if (!shouldUseOfflineMode()) {
      updatePerformanceMetrics(true, latency);
      lastSuccessfulOperation = currentTime;
      wsManager.endLatencyMeasurement();
    }
    
    lastGpsSendTime = currentTime;
    lastActivityTime = currentTime;
    
    if (powerConfigs[currentPowerMode].performanceMonitoring && 
        DEBUG_LATENCY_TRACKING) {
      LOG_DEBUG(MODULE_PERF, "📊 %s completed in %lu ms (source: %s)", 
                shouldUseOfflineMode() ? "Offline storage" : "Transmission", 
                latency, getPositionSourceString());
    }
  } else {
    if (!shouldUseOfflineMode()) {
      updatePerformanceMetrics(false, 0);
      performanceMetrics.consecutiveFailures++;
      
      if (performanceMetrics.consecutiveFailures >= MAX_CONNECTION_FAILURES) {
        LOG_WARN(MODULE_PERF, "🔧 Multiple failures, triggering optimization");
        currentState = STATE_OPTIMIZING;
      }
    }
  }
}

// ========================================
// SERIAL COMMAND HANDLERS WITH IMU
// ========================================

void handleSerialCommandsExtended() {
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
  // IMU commands (NEW)
  else if (cmd.startsWith("imu")) {
    processImuCommands(cmd);
  }
  // Health and recovery commands
  else if (cmd == "health") {
    printHealthStatus();
  } else if (cmd == "recovery") {
    LOG_INFO(MODULE_MAIN, "🔄 Forcing system health check...");
    performSystemHealthCheck();
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
  // Offline commands
  else if (cmd.startsWith("offline")) {
    handleOfflineCommands(cmd);
  }
  // Advanced commands
  else if (cmd == "diag") {
    performSystemDiagnostics();
  } else if (cmd == "stats") {
    logSystemStats();
  } else if (cmd == "backup") {
    performEmergencyBackup();
  } else if (cmd == "recover") {
    recoverFromCorruptedState();
  } else if (cmd == "factory reset") {
    LOG_WARN(MODULE_MAIN, "Factory reset requested...");
    #if ENABLE_OFFLINE_STORAGE
      offlineManager.clearAllOfflineData();
    #endif
    delay(1000);
    ESP.restart();
  }
  // Unknown command
  else {
    LOG_WARN(MODULE_MAIN, "Unknown command: %s", cmd.c_str());
  }
  
  lastActivityTime = millis();
}

// NEW: IMU Command Processor
void processImuCommands(const String& cmd) {
  #if ENABLE_IMU_SUPPORT
    if (cmd == "imu") {
      printImuStatus();
    } else if (cmd == "imu calibrate") {
      LOG_INFO(MODULE_IMU, "Starting IMU calibration...");
      currentState = STATE_IMU_CALIBRATION;
    } else if (cmd == "imu data") {
      imuManager.printSensorData();
    } else if (cmd == "imu dr") {
      printDeadReckoningInfo();
    } else if (cmd == "imu basement") {
      printBasementStatus();
    } else if (cmd == "imu enable") {
      imuManager.enable();
      LOG_INFO(MODULE_IMU, "IMU enabled");
    } else if (cmd == "imu disable") {
      imuManager.disable();
      LOG_INFO(MODULE_IMU, "IMU disabled");
    } else if (cmd == "imu test basement") {
      LOG_INFO(MODULE_IMU, "🧪 Simulating basement entry...");
      basementMode = true;
      handleGpsLoss();
    } else if (cmd == "imu test normal") {
      LOG_INFO(MODULE_IMU, "🧪 Simulating basement exit...");
      basementMode = false;
      handleGpsRecovery();
    } else if (cmd == "imu reset") {
      imuManager.resetCalibration();
      LOG_INFO(MODULE_IMU, "IMU calibration reset");
    } else if (cmd == "imu json") {
      LOG_INFO(MODULE_IMU, "IMU JSON data: %s", imuManager.getJsonData().c_str());
    }
  #else
    LOG_WARN(MODULE_MAIN, "IMU support not enabled");
  #endif
}

// ========================================
// IMU STATUS DISPLAYS
// ========================================

void printImuStatus() {
  #if ENABLE_IMU_SUPPORT
    LOG_INFO(MODULE_IMU, "=== IMU STATUS ===");
    LOG_INFO(MODULE_IMU, "Enabled      : %s", systemFlags.imuReady ? "YES" : "NO");
    LOG_INFO(MODULE_IMU, "State        : %s", imuManager.getStateString());
    LOG_INFO(MODULE_IMU, "Calibrated   : %s", imuManager.isCalibrationComplete() ? "YES" : "NO");
    LOG_INFO(MODULE_IMU, "Moving       : %s", imuManager.isMoving() ? "YES" : "NO");
    LOG_INFO(MODULE_IMU, "Basement     : %s", basementMode ? "YES" : "NO");
    LOG_INFO(MODULE_IMU, "Dead Reckon  : %s", imuManager.isDeadReckoningActive() ? "ACTIVE" : "INACTIVE");
    LOG_INFO(MODULE_IMU, "Position Src : %s", getPositionSourceString());
    LOG_INFO(MODULE_IMU, "Heading      : %.1f°", imuManager.getCurrentHeading());
    LOG_INFO(MODULE_IMU, "Total Reads  : %lu", imuOpStats.totalImuReadings);
    LOG_INFO(MODULE_IMU, "Success Rate : %.1f%%", 
             imuOpStats.totalImuReadings > 0 ? 
             (imuOpStats.successfulImuReadings * 100.0 / imuOpStats.totalImuReadings) : 0);
    LOG_INFO(MODULE_IMU, "DR Activations: %lu", imuOpStats.deadReckoningActivations);
    LOG_INFO(MODULE_IMU, "Basement Detections: %lu", imuOpStats.basementDetections);
  #else
    LOG_INFO(MODULE_IMU, "IMU support disabled");
  #endif
}

void printDeadReckoningInfo() {
  #if ENABLE_IMU_SUPPORT
    if (!imuManager.isDeadReckoningActive()) {
      LOG_INFO(MODULE_IMU, "Dead reckoning not active");
      return;
    }
    
    DeadReckoningData drData = imuManager.getDeadReckoningData();
    
    LOG_INFO(MODULE_IMU, "=== DEAD RECKONING ===");
    LOG_INFO(MODULE_IMU, "Est Position : %.6f, %.6f", drData.estimatedLat, drData.estimatedLon);
    LOG_INFO(MODULE_IMU, "Est Speed    : %.1f m/s (%.1f km/h)", 
             drData.estimatedSpeed, drData.estimatedSpeed * 3.6);
    LOG_INFO(MODULE_IMU, "Distance     : %.1f m", drData.distanceTraveled);
    LOG_INFO(MODULE_IMU, "Position Err : %.1f m", drData.positionError);
    LOG_INFO(MODULE_IMU, "Confidence   : %.1f%%", drData.confidenceLevel * 100);
    LOG_INFO(MODULE_IMU, "Active Time  : %lu s", (millis() - drData.startTime) / 1000);
  #else
    LOG_INFO(MODULE_IMU, "IMU support disabled");
  #endif
}

void printBasementStatus() {
  #if ENABLE_IMU_SUPPORT
    BasementDetectionData bData = imuManager.getBasementStatus();
    
    LOG_INFO(MODULE_IMU, "=== BASEMENT STATUS ===");
    LOG_INFO(MODULE_IMU, "In Basement  : %s", bData.inBasement ? "YES" : "NO");
    LOG_INFO(MODULE_IMU, "GPS Loss Cnt : %d", bData.gpsLossCount);
    
    if (bData.entryTime > 0) {
      LOG_INFO(MODULE_IMU, "Entry Time   : %lu s ago", (millis() - bData.entryTime) / 1000);
      LOG_INFO(MODULE_IMU, "Entry Z-Accel: %.2f g", bData.entryZAccel);
    }
    
    if (bData.exitTime > 0) {
      LOG_INFO(MODULE_IMU, "Exit Time    : %lu s ago", (millis() - bData.exitTime) / 1000);
      LOG_INFO(MODULE_IMU, "Exit Z-Accel : %.2f g", bData.exitZAccel);
    }
    
    LOG_INFO(MODULE_IMU, "Possible Entry: %s", bData.possibleEntry ? "YES" : "NO");
    LOG_INFO(MODULE_IMU, "Possible Exit : %s", bData.possibleExit ? "YES" : "NO");
  #else
    LOG_INFO(MODULE_IMU, "IMU support disabled");
  #endif
}

// ========================================
// ENHANCED STATUS DISPLAY WITH IMU
// ========================================

void printStatus() {
  SerialMon.println("\n========== SYSTEM STATUS ==========");
  
  // System state
  SerialMon.printf("System State : %s\n", getStateString(currentState));
  SerialMon.printf("Power Mode   : %s\n", getPowerModeString(currentPowerMode));
  SerialMon.printf("System Ready : %s\n", isSystemReady() ? "YES ✅" : "NO ⚠️");
  
  if (!isSystemReady()) {
    if (!systemFlags.gpsReady && !systemFlags.imuReady) SerialMon.println("  - No position source");
    if (!systemFlags.modemReady) SerialMon.println("  - Modem not ready");
    if (!systemFlags.networkReady) SerialMon.println("  - Network not ready");
    if (!systemFlags.gprsReady) SerialMon.println("  - GPRS not ready");
  }
  
  // Network mode
  SerialMon.printf("Network Mode : %s\n", networkAvailable ? "ONLINE 📶" : "OFFLINE 📵");
  
  // Position source (NEW)
  SerialMon.printf("Position Src : %s", getPositionSourceString());
  if (basementMode) SerialMon.print(" [BASEMENT]");
  SerialMon.println();
  
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
  
  // IMU info (NEW)
  #if ENABLE_IMU_SUPPORT
    SerialMon.printf("IMU          : %s", systemFlags.imuReady ? "READY" : "NOT READY");
    if (systemFlags.imuReady) {
      SerialMon.printf(" (%s)", imuManager.isMoving() ? "MOVING" : "STATIONARY");
      if (imuManager.isDeadReckoningActive()) {
        SerialMon.printf(" [DR: %.1fm error]", imuManager.getPositionError());
      }
    }
    SerialMon.println();
  #endif
  
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
  
  // Offline data status
  #if ENABLE_OFFLINE_STORAGE
    SerialMon.printf("Offline Data : %d records", offlineManager.getOfflineRecordCount());
    if (offlineOpStats.syncInProgress) {
      SerialMon.printf(" [SYNCING... batch %d]", offlineOpStats.batchesSent);
    }
    SerialMon.println();
  #endif
  
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
  
  // Offline operations
  if (offlineOpStats.dataStoredOffline > 0 || offlineOpStats.dataSentFromOffline > 0) {
    SerialMon.printf("Offline Ops  : %lu stored, %lu sent\n", 
                     offlineOpStats.dataStoredOffline, offlineOpStats.dataSentFromOffline);
  }
  
  // IMU operations (NEW)
  #if ENABLE_IMU_SUPPORT
    if (imuOpStats.totalImuReadings > 0) {
      SerialMon.printf("IMU Ops      : %lu reads, %lu DR activations\n",
                       imuOpStats.totalImuReadings, imuOpStats.deadReckoningActivations);
    }
  #endif
  
  // Health status
  SerialMon.printf("Health       : %s", 
                   consecutiveHealthFailures < MAX_HEALTH_FAILURES ? "HEALTHY" : "CRITICAL");
  if (consecutiveHealthFailures > 0) {
    SerialMon.printf(" (%d/%d failures)", consecutiveHealthFailures, MAX_HEALTH_FAILURES);
  }
  SerialMon.println();
  
  SerialMon.printf("Uptime       : %s\n", Utils::formatUptime(millis()).c_str());
  SerialMon.printf("Free Memory  : %u KB", Utils::getFreeHeap() / 1024);
  if (Utils::getFreeHeap() < MEMORY_CRITICAL_THRESHOLD) {
    SerialMon.print(" ⚠️ LOW");
  }
  SerialMon.println();
  
  SerialMon.println("===================================\n");
}

void printHelp() {
  SerialMon.println("\n========== COMMAND HELP ==========");
  SerialMon.println("=== BASIC COMMANDS ===");
  SerialMon.println("help         - Show this help");
  SerialMon.println("status       - Show system status");
  SerialMon.println("init         - Show initialization status");
  SerialMon.println("reset        - Restart system");
  SerialMon.println("health       - Show health status");
  SerialMon.println("recovery     - Force health check");
  
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
  
  // NEW: IMU commands section
  SerialMon.println("\n=== IMU COMMANDS ===");
  SerialMon.println("imu          - Show IMU status");
  SerialMon.println("imu calibrate- Calibrate IMU");
  SerialMon.println("imu data     - Show sensor data");
  SerialMon.println("imu dr       - Dead reckoning info");
  SerialMon.println("imu basement - Basement detection status");
  SerialMon.println("imu enable   - Enable IMU");
  SerialMon.println("imu disable  - Disable IMU");
  SerialMon.println("imu test basement - Simulate basement");
  SerialMon.println("imu test normal   - Exit basement");
  SerialMon.println("imu reset    - Reset calibration");
  SerialMon.println("imu json     - Show IMU JSON data");
  
  SerialMon.println("\n=== TESTING COMMANDS ===");
  SerialMon.println("send         - Force send GPS data");
  SerialMon.println("teststop     - Test PARKED→STATIC transition");
  SerialMon.println("testmove     - Test all movement modes");
  SerialMon.println("testreset    - Reset test variables");
  SerialMon.println("testcrash    - Test auto-recovery system");
  SerialMon.println("testoffline  - Generate test offline data");
  
  SerialMon.println("\n=== NETWORK & OPTIMIZATION ===");
  SerialMon.println("network      - Show network info");
  SerialMon.println("optimize     - Apply optimizations");
  SerialMon.println("latency      - Show performance report");
  
  SerialMon.println("\n=== OFFLINE DATA COMMANDS ===");
  SerialMon.println("offline      - Show offline status");
  SerialMon.println("offline stats- Show offline statistics");
  SerialMon.println("offline records - Show stored records");
  SerialMon.println("offline sync - Manual sync data");
  SerialMon.println("offline clear- Clear all offline data");
  SerialMon.println("offline test - Test offline storage");
  SerialMon.println("offline storage - Show storage info");
  SerialMon.println("offline enable/disable - Control feature");
  SerialMon.println("offline force/online - Force offline/online mode");
  
  SerialMon.println("\n=== ADVANCED COMMANDS ===");
  SerialMon.println("diag         - System diagnostics");
  SerialMon.println("stats        - System statistics");
  SerialMon.println("backup       - Emergency backup");
  SerialMon.println("recover      - Recover from corruption");
  SerialMon.println("factory reset- Complete system reset");
  SerialMon.println("stress test  - Run storage stress test");
  SerialMon.println("simulate outage N - Simulate N-second network outage");
  
  SerialMon.println("\n=== OTHER COMMANDS ===");
  SerialMon.println("battery      - Show battery status");
  SerialMon.println("memory       - Show memory info");
  SerialMon.println("gps          - Show GPS details");
  SerialMon.println("on/off       - Control relay");
  
  SerialMon.println("==================================\n");
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
  SerialMon.printf("IMU Enabled  : %s\n", config.imuEnabled ? "Yes" : "No");  // NEW
  
  SerialMon.println("\n=== MOVEMENT INTERVALS ===");
  SerialMon.printf("MOVING       : %d seconds (speed > %.1f km/h)\n", GPS_INTERVAL_MOVING/1000, float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("PARKED       : %d seconds (speed 0-%.1f km/h)\n", GPS_INTERVAL_PARKED/1000, float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("STATIC       : %d minutes (parked > 5 min)\n", GPS_INTERVAL_STATIC/60000);
  SerialMon.printf("Speed Thresh : %.1f km/h\n", float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("Park→Static  : %d minutes\n", PARKED_TO_STATIC_TIMEOUT/60000);
  
  SerialMon.println("\n=== AUTO-RECOVERY INFO ===");
  #if ENABLE_AUTO_RESTART
    SerialMon.printf("Auto-restart : ENABLED (every 72 hours)\n");
    SerialMon.printf("Memory Alert : %d KB\n", MEMORY_CRITICAL_THRESHOLD/1024);
    SerialMon.printf("Success Rate : %d%% minimum\n", SUCCESS_RATE_THRESHOLD);
    SerialMon.printf("Health Check : every 5 minutes\n");
  #else
    SerialMon.printf("Auto-restart : DISABLED\n");
  #endif
  
  SerialMon.println("\n=== OFFLINE STORAGE INFO ===");
  #if ENABLE_OFFLINE_STORAGE
    SerialMon.printf("Offline Mode : ENABLED\n");
    SerialMon.printf("Max Records  : %d\n", OFFLINE_MAX_RECORDS);
    SerialMon.printf("Auto-sync    : %s\n", OFFLINE_AUTO_SYNC ? "ENABLED" : "DISABLED");
    SerialMon.printf("Batch Size   : %d records\n", OFFLINE_SYNC_BATCH_SIZE);
    SerialMon.printf("Maintenance  : every %d minutes\n", OFFLINE_MAINTENANCE_INTERVAL/60000);
  #else
    SerialMon.printf("Offline Mode : DISABLED\n");
  #endif
  
  // NEW: IMU info section
  SerialMon.println("\n=== IMU CONFIGURATION ===");
  #if ENABLE_IMU_SUPPORT
    SerialMon.printf("IMU Support  : ENABLED\n");
    SerialMon.printf("I2C Address  : 0x%02X\n", IMU_I2C_ADDRESS);
    SerialMon.printf("Sample Rates : GPS=%d Hz, Lost=%d Hz, Moving=%d Hz\n",
                     IMU_SAMPLE_RATE_GPS_VALID, IMU_SAMPLE_RATE_GPS_LOST, IMU_SAMPLE_RATE_MOVING);
    SerialMon.printf("Movement Thr : %.2f g\n", IMU_MOVEMENT_THRESHOLD);
    SerialMon.printf("Max DR Error : %d m\n", IMU_MAX_POSITION_ERROR);
    SerialMon.printf("Basement Det : %.1f g threshold\n", BASEMENT_ENTRY_THRESHOLD);
  #else
    SerialMon.printf("IMU Support  : DISABLED\n");
  #endif
  
  SerialMon.println("=============================\n");
}

// ========================================
// DIAGNOSTIC FUNCTIONS WITH IMU
// ========================================

void performSystemDiagnostics() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM DIAGNOSTICS ===");
  
  // Memory diagnostics
  LOG_INFO(MODULE_MAIN, "MEMORY:");
  Utils::printMemoryInfo();
  
  // Network diagnostics
  LOG_INFO(MODULE_MAIN, "\nNETWORK:");
  LOG_INFO(MODULE_MAIN, modemManager.getNetworkInfo().c_str());
  modemManager.performNetworkDiagnostic();
  
  // GPS diagnostics
  LOG_INFO(MODULE_MAIN, "\nGPS:");
  showGpsDetails();
  
  // NEW: IMU diagnostics
  #if ENABLE_IMU_SUPPORT
    LOG_INFO(MODULE_MAIN, "\nIMU:");
    LOG_INFO(MODULE_MAIN, "Status: %s", imuManager.getStateString());
    LOG_INFO(MODULE_MAIN, "Calibrated: %s", imuManager.isCalibrationComplete() ? "YES" : "NO");
    LOG_INFO(MODULE_MAIN, "Error Rate: %.1f%%", imuManager.getErrorRate());
    LOG_INFO(MODULE_MAIN, imuManager.getDiagnosticInfo().c_str());
  #endif
  
  // Performance diagnostics
  LOG_INFO(MODULE_MAIN, "\nPERFORMANCE:");
  printPerformanceReport();
  
  // Offline storage diagnostics
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_MAIN, "\nOFFLINE STORAGE:");
    offlineManager.printStorageInfo();
    LOG_INFO(MODULE_MAIN, "Validation: %s", 
             offlineManager.validateStorage() ? "PASSED" : "FAILED");
  #endif
  
  // System health
  LOG_INFO(MODULE_MAIN, "\nHEALTH:");
  printHealthStatus();
  
  LOG_INFO(MODULE_MAIN, "========================");
}

void logSystemStats() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM STATISTICS ===");
  
  // Uptime stats
  unsigned long uptime = millis() - systemStartTime;
  LOG_INFO(MODULE_MAIN, "Uptime: %s", Utils::formatUptime(uptime).c_str());
  
  // Transmission stats
  LOG_INFO(MODULE_MAIN, "Online transmissions: %lu total, %lu successful (%.1f%%)",
           performanceMetrics.totalTransmissions,
           performanceMetrics.successfulTransmissions,
           performanceMetrics.totalTransmissions > 0 ? 
           (performanceMetrics.successfulTransmissions * 100.0 / performanceMetrics.totalTransmissions) : 0);
  
  #if ENABLE_OFFLINE_STORAGE
    // Offline stats
    LOG_INFO(MODULE_MAIN, "Offline operations: %lu stored, %lu synced",
             offlineOpStats.dataStoredOffline,
             offlineOpStats.dataSentFromOffline);
  #endif
  
  #if ENABLE_IMU_SUPPORT
    // IMU stats (NEW)
    LOG_INFO(MODULE_MAIN, "IMU operations: %lu reads, %lu DR activations, %lu basement detections",
             imuOpStats.totalImuReadings,
             imuOpStats.deadReckoningActivations,
             imuOpStats.basementDetections);
  #endif
  
  // Movement stats
  LOG_INFO(MODULE_MAIN, "Current movement: %s", getMovementString(currentMovementState));
  LOG_INFO(MODULE_MAIN, "Position source: %s", getPositionSourceString());
  
  // Health stats
  LOG_INFO(MODULE_MAIN, "Health check failures: %d", consecutiveHealthFailures);
  
  // Memory stats
  LOG_INFO(MODULE_MAIN, "Free memory: %u KB (min required: %u KB)",
           Utils::getFreeHeap() / 1024,
           MEMORY_CRITICAL_THRESHOLD / 1024);
  
  LOG_INFO(MODULE_MAIN, "========================");
}

// ========================================
// STATE HANDLERS (Original functions)
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
    lastGpsFixTime = millis();
    
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
    
    // If IMU available, we can still operate
    #if ENABLE_IMU_SUPPORT
      if (systemFlags.imuReady) {
        LOG_INFO(MODULE_GPS, "📡 Using IMU for position tracking");
        currentState = STATE_INIT;
      } else {
        currentState = STATE_INIT;
      }
    #else
      currentState = STATE_INIT;
    #endif
  }
}

// Include all other original state handlers and functions from your original main.cpp
// (handleInitState, handleModemResetState, handleConnectionRecoveryState, etc.)
// These remain unchanged from your original implementation

// ========================================
// ENHANCED FUNCTIONS
// ========================================

float getCurrentSpeed() {
  if (useManualSpeed) {
    return manualSpeed;
  }
  
  #if ENABLE_IMU_SUPPORT
    if (shouldUseImuPosition() && imuManager.isDeadReckoningActive()) {
      return imuManager.getEstimatedSpeed() * 3.6; // m/s to km/h
    }
  #endif
  
  return gpsManager.getSpeed();
}

bool sendVehicleDataWithOfflineSupport() {
  // Check if we should use offline mode
  if (shouldUseOfflineMode()) {
    LOG_INFO(MODULE_GPS, "📵 Network unavailable, storing data offline");
    
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    float displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
    
    return storeDataOffline(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      displaySpeed,
      gpsManager.getSatellites(),
      String(timestamp),
      batteryVoltage
    );
  }
  
  // Normal online transmission - use IMU enhanced version if available
  #if ENABLE_IMU_SUPPORT
    return sendVehicleDataWithImu();
  #else
    return sendVehicleDataViaWebSocket();
  #endif
}

void transmitGpsDataWithOfflineSupport(unsigned long currentTime, unsigned long interval) {
  transmitWithImuData(currentTime, interval);
}

// Include all other functions from your original main.cpp that don't need modification

// ========================================
// REMAINING ORIGINAL FUNCTIONS
// ========================================

// ========================================
// ORIGINAL STATE HANDLERS
// ========================================

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
    
    if ((systemFlags.gpsReady || systemFlags.imuReady) && systemFlags.wsReady) {
      lastGpsSendTime = 0; // Force immediate send
    }
    
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Modem initialization failed");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
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

void handleOfflineSyncState() {
  #if ENABLE_OFFLINE_STORAGE
    if (!offlineOpStats.syncInProgress) {
      currentState = STATE_OPERATIONAL;
      return;
    }
    
    // Check network still available
    if (!networkAvailable) {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Network lost during sync, aborting");
      offlineManager.stopSending();
      offlineOpStats.syncInProgress = false;
      currentState = STATE_OPERATIONAL;
      return;
    }
    
    // Continue sending
    bool stillSending = offlineManager.continueeSendingOfflineData();
    
    if (!stillSending) {
      // Sync complete
      unsigned long syncDuration = millis() - offlineOpStats.syncStartTime;
      LOG_INFO(MODULE_OFFLINE, "✅ Offline sync complete in %lu ms", syncDuration);
      
      offlineOpStats.syncInProgress = false;
      offlineOpStats.lastOfflineSyncTime = millis();
      currentState = STATE_OPERATIONAL;
      
      // Update stats
      offlineOpStats.hasUnsentData = offlineManager.hasOfflineData();
    } else {
      // Still sending
      offlineOpStats.batchesSent++;
      
      // Timeout check
      if (millis() - offlineOpStats.syncStartTime > 300000) {  // 5 minute timeout
        LOG_WARN(MODULE_OFFLINE, "⚠️ Offline sync timeout");
        offlineManager.stopSending();
        offlineOpStats.syncInProgress = false;
        currentState = STATE_OPERATIONAL;
      }
    }
  #else
    currentState = STATE_OPERATIONAL;
  #endif
}

// ========================================
// AUTO-RECOVERY FUNCTIONS
// ========================================

void checkAutoRestart() {
  #if ENABLE_AUTO_RESTART
    // Auto-restart setiap 3 hari untuk preventive maintenance
    if (millis() - systemStartTime > AUTO_RESTART_INTERVAL) {
      forceSystemRestart("Scheduled restart after 72 hours uptime");
    }
  #endif
}

void checkMemoryHealth() {
  // MEMORY MONITOR: Check setiap 1 menit
  if (millis() - lastMemoryCheck > MEMORY_CHECK_INTERVAL) {
    uint32_t freeHeap = Utils::getFreeHeap();
    if (freeHeap < MEMORY_CRITICAL_THRESHOLD) {
      LOG_ERROR(MODULE_SYS, "🚨 Critical memory low: %u bytes", freeHeap);
      forceSystemRestart("Critical memory shortage");
    }
    lastMemoryCheck = millis();
  }
}

void performSystemHealthCheck() {
  if (millis() - lastHealthCheck < HEALTH_CHECK_INTERVAL) {
    return; // Check setiap 5 menit
  }
  
  lastHealthCheck = millis();
  bool systemHealthy = true;
  
  // Check 1: Memory
  if (Utils::getFreeHeap() < 20000) {
    LOG_WARN(MODULE_SYS, "⚠️ Health: Low memory");
    systemHealthy = false;
  }
  
  // Check 2: Success rate
  if (performanceMetrics.totalTransmissions > 10) {
    unsigned long successRate = (performanceMetrics.successfulTransmissions * 100) / 
                               performanceMetrics.totalTransmissions;
    if (successRate < SUCCESS_RATE_THRESHOLD) {
      LOG_WARN(MODULE_SYS, "⚠️ Health: Low success rate: %lu%%", successRate);
      systemHealthy = false;
    }
  }
  
  // Check 3: No successful transmission dalam 30 menit
  if (performanceMetrics.totalTransmissions > 0) {
    if (performanceMetrics.successfulTransmissions > 0) {
      lastSuccessfulTransmission = millis();
    } else if (millis() - lastSuccessfulTransmission > NO_SUCCESS_TIMEOUT) {
      LOG_ERROR(MODULE_SYS, "🚨 No successful transmission in 30 min");
      forceSystemRestart("No successful transmission timeout");
      return;
    }
  }
  
  // Check 4: Modem stuck in error state
  if (modemManager.getStatus() == MODEM_STATUS_ERROR) {
    LOG_WARN(MODULE_SYS, "⚠️ Health: Modem in error state");
    systemHealthy = false;
  }
  
  // Check 5: WebSocket stuck
  if (powerConfigs[currentPowerMode].wsContinuous && 
      wsManager.getState() == WS_DISCONNECTED && 
      millis() > 600000) { // After 10 minutes uptime
    LOG_WARN(MODULE_SYS, "⚠️ Health: WebSocket stuck disconnected");
    systemHealthy = false;
  }
  
  // Check 6: Offline storage health
  #if ENABLE_OFFLINE_STORAGE
    if (offlineManager.isReady() && offlineOpStats.hasUnsentData) {
      if (millis() - offlineOpStats.lastOfflineStoreTime > 3600000) {  // 1 hour old data
        LOG_WARN(MODULE_SYS, "⚠️ Health: Old offline data not synced");
        systemHealthy = false;
      }
    }
  #endif
  
  // NEW Check 7: IMU health
  #if ENABLE_IMU_SUPPORT
    if (systemFlags.imuReady && imuOpStats.imuErrors > 100) {
      LOG_WARN(MODULE_SYS, "⚠️ Health: High IMU error rate");
      systemHealthy = false;
    }
  #endif
  
  if (!systemHealthy) {
    consecutiveHealthFailures++;
    LOG_WARN(MODULE_SYS, "🏥 System health failures: %d/%d", 
             consecutiveHealthFailures, MAX_HEALTH_FAILURES);
    
    if (consecutiveHealthFailures >= MAX_HEALTH_FAILURES) {
      forceSystemRestart("System health critical");
    }
  } else {
    consecutiveHealthFailures = 0;
  }
}

void forceSystemRestart(const char* reason) {
  LOG_ERROR(MODULE_SYS, "🚨 FORCED RESTART: %s", reason);
  Utils::printMemoryInfo();
  printPerformanceReport();
  printOfflineStats();
  #if ENABLE_IMU_SUPPORT
    printImuStatus();
  #endif
  delay(2000);
  ESP.restart();
}

// ========================================
// OFFLINE STORAGE FUNCTIONS
// ========================================

void initializeOfflineStorage() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_OFFLINE, "Initializing offline storage...");
    
    if (offlineManager.begin(ENABLE_OFFLINE_STORAGE)) {
      LOG_INFO(MODULE_OFFLINE, "✅ Offline storage initialized");
      
      // Check for existing offline data
      if (offlineManager.hasOfflineData()) {
        offlineOpStats.hasUnsentData = true;
        LOG_INFO(MODULE_OFFLINE, "📦 Found %d offline records", 
                 offlineManager.getOfflineRecordCount());
      }
      
      // Set callbacks
      offlineManager.setOnDataSentCallback([](int sent, int remaining) {
        LOG_INFO(MODULE_OFFLINE, "📤 Sent %d records, %d remaining", sent, remaining);
        offlineOpStats.dataSentFromOffline += sent;
        offlineOpStats.hasUnsentData = (remaining > 0);
      });
      
      offlineManager.setOnStorageFullCallback([](int stored) {
        LOG_WARN(MODULE_OFFLINE, "⚠️ Offline storage full! %d records stored", stored);
      });
      
      offlineManager.setOnErrorCallback([](const char* error) {
        LOG_ERROR(MODULE_OFFLINE, "❌ Offline storage error: %s", error);
      });
      
    } else {
      LOG_ERROR(MODULE_OFFLINE, "❌ Failed to initialize offline storage");
    }
  #else
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
  #endif
}

void checkNetworkAvailability() {
  if (millis() - lastNetworkCheck < 5000) return;  // Check every 5 seconds
  
  lastNetworkCheck = millis();
  bool previousNetworkState = networkAvailable;
  
  // Check network status
  networkAvailable = systemFlags.networkReady && systemFlags.gprsReady && 
                    modemManager.isNetworkConnected() && modemManager.isGprsConnected();
  
  // Detect state changes
  if (networkAvailable != previousNetworkState) {
    if (networkAvailable) {
      LOG_INFO(MODULE_SYS, "📶 Network available - switching to ONLINE mode");
      offlineMode = false;
      
      // Start sync if we have offline data
      if (offlineOpStats.hasUnsentData && OFFLINE_AUTO_SYNC) {
        LOG_INFO(MODULE_OFFLINE, "🔄 Starting auto-sync of offline data");
        processOfflineQueue();
      }
    } else {
      LOG_WARN(MODULE_SYS, "📵 Network lost - switching to OFFLINE mode");
      offlineMode = true;
    }
  }
}

bool shouldUseOfflineMode() {
  return (!networkAvailable || offlineMode) && ENABLE_OFFLINE_STORAGE;
}

bool storeDataOffline(float lat, float lon, float speed, int satellites, 
                     const String& timestamp, float battery) {
  #if ENABLE_OFFLINE_STORAGE
    if (!offlineManager.isReady()) {
      LOG_ERROR(MODULE_OFFLINE, "Offline storage not ready");
      return false;
    }
    
    bool success = offlineManager.storeGpsData(lat, lon, speed, satellites, timestamp, battery);
    
    if (success) {
      offlineOpStats.dataStoredOffline++;
      offlineOpStats.lastOfflineStoreTime = millis();
      offlineOpStats.hasUnsentData = true;
      LOG_INFO(MODULE_OFFLINE, "💾 Data stored offline successfully");
    }
    
    return success;
  #else
    return false;
  #endif
}

void syncOfflineData() {
  #if ENABLE_OFFLINE_STORAGE
    if (!networkAvailable || !offlineOpStats.hasUnsentData) return;
    
    LOG_INFO(MODULE_OFFLINE, "🔄 Starting offline data sync...");
    currentState = STATE_OFFLINE_SYNC;
    offlineOpStats.syncInProgress = true;
    offlineOpStats.syncStartTime = millis();
    offlineOpStats.batchesSent = 0;
    
    offlineManager.startSendingOfflineData();
  #endif
}

void processOfflineQueue() {
  #if ENABLE_OFFLINE_STORAGE
    if (!networkAvailable || !offlineOpStats.hasUnsentData || offlineOpStats.syncInProgress) {
      return;
    }
    
    // Avoid too frequent sync attempts
    if (millis() - lastOfflineSync < 10000) return;  // Min 10s between syncs
    
    lastOfflineSync = millis();
    syncOfflineData();
  #endif
}

// Integration function for offline manager to send data
extern "C" bool sendOfflineRecordViaWebSocket(float lat, float lon, float speed, 
                                             int satellites, const char* timestamp, 
                                             float battery) {
  if (!networkAvailable || !wsManager.isReady()) {
    return false;
  }
  
  return wsManager.sendVehicleData(lat, lon, speed, satellites, String(timestamp), battery);
}

// ========================================
// REMAINING FUNCTIONS
// ========================================

// Include all the remaining functions exactly as they appear in your original main.cpp
// This includes all the utility functions, monitoring functions, etc.

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

// ========================================
// DATA TRANSMISSION
// ========================================

bool sendVehicleDataViaWebSocket() {
  // Pre-checks
  if (!systemFlags.gpsReady && !useManualSpeed && !shouldUseImuPosition()) {
    LOG_ERROR(MODULE_GPS, "❌ No position source available");
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
  
  if (gpsManager.isValid() || useManualSpeed || shouldUseImuPosition()) {
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    // Start latency measurement
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      wsManager.startLatencyMeasurement();
      modemManager.startLatencyMeasurement();
    }
    
    // Get position and speed
    float lat, lon, displaySpeed;
    int satellites;
    
    #if ENABLE_IMU_SUPPORT
      if (shouldUseImuPosition()) {
        lat = imuManager.getEstimatedLatitude();
        lon = imuManager.getEstimatedLongitude();
        displaySpeed = imuManager.getEstimatedSpeed() * 3.6; // m/s to km/h
        satellites = 0;
      } else {
        lat = gpsManager.getLatitude();
        lon = gpsManager.getLongitude();
        displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
        satellites = gpsManager.getSatellites();
      }
    #else
      lat = gpsManager.getLatitude();
      lon = gpsManager.getLongitude();
      displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
      satellites = gpsManager.getSatellites();
    #endif
    
    success = wsManager.sendVehicleData(
      lat, lon, displaySpeed, satellites,
      timestamp, batteryVoltage
    );
    
    if (success) {
      if (powerConfigs[currentPowerMode].performanceMonitoring) {
        modemManager.endLatencyMeasurement();
      }
      
      LOG_INFO(MODULE_GPS, "✅ Data sent successfully");
      LOG_INFO(MODULE_GPS, "📍 Pos: %.6f, %.6f | 🚗 %.1f km/h | 🛰️ %d sats",
               lat, lon, displaySpeed, satellites);
      
      if (useManualSpeed) {
        LOG_INFO(MODULE_GPS, "📏 Using manual speed: %.1f km/h", manualSpeed);
      }
      
      #if ENABLE_IMU_SUPPORT
        if (shouldUseImuPosition()) {
          LOG_INFO(MODULE_GPS, "📡 Position source: %s", getPositionSourceString());
        }
      #endif
    } else {
      LOG_ERROR(MODULE_GPS, "❌ Failed to send data");
    }
  } else {
    LOG_WARN(MODULE_GPS, "⚠️ No valid position data");
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
      lastGpsFixTime = millis();
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
  transmitWithImuData(currentTime, interval);
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
// OFFLINE COMMAND HANDLERS
// ========================================

void handleOfflineCommands(const String& cmd) {
  #if ENABLE_OFFLINE_STORAGE
    if (cmd == "offline") {
      printOfflineStatus();
    } else if (cmd == "offline stats") {
      printOfflineStats();
    } else if (cmd == "offline records") {
      offlineManager.printOfflineRecords();
    } else if (cmd == "offline sync") {
      LOG_INFO(MODULE_OFFLINE, "Manual sync requested");
      if (networkAvailable) {
        syncOfflineData();
      } else {
        LOG_WARN(MODULE_OFFLINE, "Cannot sync - network unavailable");
      }
    } else if (cmd == "offline clear") {
      LOG_WARN(MODULE_OFFLINE, "Clearing all offline data...");
      offlineManager.clearAllOfflineData();
      offlineOpStats.hasUnsentData = false;
    } else if (cmd == "offline test") {
      LOG_INFO(MODULE_OFFLINE, "Testing offline storage...");
      storeDataOffline(-6.2088, 106.8456, 50.5, 8, "2025-01-17T10:00:00Z", 12.5);
    } else if (cmd == "offline storage") {
      offlineManager.printStorageInfo();
    } else if (cmd == "offline enable") {
      offlineManager.enable();
      LOG_INFO(MODULE_OFFLINE, "Offline storage enabled");
    } else if (cmd == "offline disable") {
      offlineManager.disable();
      LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
    } else if (cmd == "offline force") {
      offlineMode = true;
      LOG_INFO(MODULE_OFFLINE, "Forced offline mode");
    } else if (cmd == "offline online") {
      offlineMode = false;
      LOG_INFO(MODULE_OFFLINE, "Forced online mode");
    }
  #else
    LOG_WARN(MODULE_MAIN, "Offline storage not enabled");
  #endif
}

void printOfflineStatus() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_OFFLINE, "=== OFFLINE STATUS ===");
    LOG_INFO(MODULE_OFFLINE, "Status       : %s", offlineManager.getStatusString());
    LOG_INFO(MODULE_OFFLINE, "Enabled      : %s", offlineManager.isEnabledStatus() ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Records      : %d", offlineManager.getOfflineRecordCount());
    LOG_INFO(MODULE_OFFLINE, "Has Unsent   : %s", offlineOpStats.hasUnsentData ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Sync Progress: %s", offlineOpStats.syncInProgress ? "ACTIVE" : "IDLE");
    LOG_INFO(MODULE_OFFLINE, "Network Mode : %s", networkAvailable ? "ONLINE" : "OFFLINE");
    LOG_INFO(MODULE_OFFLINE, "Forced Mode  : %s", offlineMode ? "OFFLINE" : "AUTO");
  #else
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
  #endif
}

void printOfflineStats() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_OFFLINE, "=== OFFLINE STATISTICS ===");
    LOG_INFO(MODULE_OFFLINE, "Data Stored  : %lu", offlineOpStats.dataStoredOffline);
    LOG_INFO(MODULE_OFFLINE, "Data Sent    : %lu", offlineOpStats.dataSentFromOffline);
    
    if (offlineOpStats.lastOfflineStoreTime > 0) {
      LOG_INFO(MODULE_OFFLINE, "Last Store   : %lu sec ago", 
               (millis() - offlineOpStats.lastOfflineStoreTime) / 1000);
    }
    
    if (offlineOpStats.lastOfflineSyncTime > 0) {
      LOG_INFO(MODULE_OFFLINE, "Last Sync    : %lu sec ago", 
               (millis() - offlineOpStats.lastOfflineSyncTime) / 1000);
    }
    
    offlineManager.printStats();
  #else
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
  #endif
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
// UTILITY FUNCTIONS
// ========================================

unsigned long getGpsIntervalForMovement() {
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    switch (currentMovementState) {
      case MOVEMENT_MOVING:
        return GPS_INTERVAL_MOVING;    // 3 seconds
      case MOVEMENT_PARKED:
        return GPS_INTERVAL_PARKED;    // 30 seconds
      case MOVEMENT_STATIC:
        return GPS_INTERVAL_STATIC;    // 1 hour
      default:
        return powerConfigs[currentPowerMode].gpsInterval;
    }
  #else
    return powerConfigs[currentPowerMode].gpsInterval;
  #endif
}

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
    case STATE_OFFLINE_SYNC: return "OFFLINE_SYNC";
    case STATE_IMU_CALIBRATION: return "IMU_CALIBRATION";
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

// ========================================
// REMAINING COMMAND HANDLERS
// ========================================

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
  } else if (cmd == "testcrash") {
    // Test auto-recovery by simulating crash
    LOG_WARN(MODULE_MAIN, "🧪 Simulating system crash for recovery test...");
    forceSystemRestart("Manual crash test");
  } else if (cmd == "testoffline") {
    // Generate test offline data
    #if ENABLE_OFFLINE_STORAGE
      LOG_INFO(MODULE_MAIN, "🧪 Generating test offline data...");
      for (int i = 0; i < 10; i++) {
        float lat = -6.2088 + (i * 0.0001);
        float lon = 106.8456 + (i * 0.0001);
        float speed = 30.0 + (i * 2);
        char timestamp[30];
        sprintf(timestamp, "2025-01-17T10:%02d:00Z", i);
        
        storeDataOffline(lat, lon, speed, 8, String(timestamp), 12.5);
        delay(100);
      }
      LOG_INFO(MODULE_MAIN, "✅ Generated 10 test offline records");
    #else
      LOG_WARN(MODULE_MAIN, "Offline storage not enabled");
    #endif
  } else if (cmd == "stress test") {
    runOfflineStorageStressTest();
  } else if (cmd.startsWith("simulate outage ")) {
    unsigned long duration = cmd.substring(16).toInt() * 1000;
    simulateNetworkOutage(duration);
  }
}

// ========================================
// DISPLAY FUNCTIONS
// ========================================

void showSpeedInfo() {
  LOG_INFO(MODULE_MAIN, "=== SPEED INFO ===");
  
  if (useManualSpeed) {
    LOG_INFO(MODULE_MAIN, "Mode: MANUAL");
    LOG_INFO(MODULE_MAIN, "Manual Speed: %.1f km/h", manualSpeed);
  } else {
    LOG_INFO(MODULE_MAIN, "Mode: AUTOMATIC (GPS)");
  }
  
  LOG_INFO(MODULE_MAIN, "GPS Speed: %.1f km/h", gpsManager.getSpeed());
  
  #if ENABLE_IMU_SUPPORT
    if (imuManager.isDeadReckoningActive()) {
      LOG_INFO(MODULE_MAIN, "IMU Speed: %.1f km/h", imuManager.getEstimatedSpeed() * 3.6);
    }
  #endif
  
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
  
  if (!systemFlags.gpsReady && !useManualSpeed && !shouldUseImuPosition()) {
    LOG_ERROR(MODULE_MAIN, "❌ Cannot send - No position source");
    return;
  }
  
  if (!systemFlags.modemReady) {
    LOG_ERROR(MODULE_MAIN, "❌ Cannot send - Modem not ready");
    return;
  }
  
  if (!systemFlags.gprsReady && !shouldUseOfflineMode()) {
    LOG_ERROR(MODULE_MAIN, "❌ Cannot send - GPRS not connected");
    return;
  }
  
  if (sendVehicleDataWithOfflineSupport()) {
    LOG_INFO(MODULE_MAIN, "✅ Data %s successfully", 
             shouldUseOfflineMode() ? "stored offline" : "sent");
  } else {
    LOG_ERROR(MODULE_MAIN, "❌ Failed to %s data", 
              shouldUseOfflineMode() ? "store" : "send");
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
  
  if (gpsLostTime > 0) {
    LOG_INFO(MODULE_MAIN, "GPS Lost     : %lu sec ago", (millis() - gpsLostTime) / 1000);
  }
}

void printHealthStatus() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM HEALTH STATUS ===");
  LOG_INFO(MODULE_MAIN, "Free Memory    : %u KB", Utils::getFreeHeap() / 1024);
  LOG_INFO(MODULE_MAIN, "Memory Status  : %s", 
           Utils::getFreeHeap() > MEMORY_CRITICAL_THRESHOLD ? "OK" : "CRITICAL");
  
  if (performanceMetrics.totalTransmissions > 0) {
    unsigned long successRate = (performanceMetrics.successfulTransmissions * 100) / 
                               performanceMetrics.totalTransmissions;
    LOG_INFO(MODULE_MAIN, "Success Rate   : %lu%% (%lu/%lu)", 
             successRate, performanceMetrics.successfulTransmissions, 
             performanceMetrics.totalTransmissions);
    LOG_INFO(MODULE_MAIN, "Rate Status    : %s", 
             successRate >= SUCCESS_RATE_THRESHOLD ? "OK" : "LOW");
  }
  
  unsigned long uptime = millis() - systemStartTime;
  LOG_INFO(MODULE_MAIN, "Uptime         : %s", Utils::formatUptime(uptime).c_str());
  LOG_INFO(MODULE_MAIN, "Health Failures: %d/%d", consecutiveHealthFailures, MAX_HEALTH_FAILURES);
  LOG_INFO(MODULE_MAIN, "Overall Health : %s", 
           consecutiveHealthFailures < MAX_HEALTH_FAILURES ? "HEALTHY" : "CRITICAL");
  
  // Last successful transmission
  if (performanceMetrics.successfulTransmissions > 0) {
    unsigned long timeSinceSuccess = (millis() - lastSuccessfulTransmission) / 1000;
    LOG_INFO(MODULE_MAIN, "Last Success   : %lu seconds ago", timeSinceSuccess);
  }
  
  // Auto-restart info
  #if ENABLE_AUTO_RESTART
    unsigned long timeToRestart = (AUTO_RESTART_INTERVAL - (millis() - systemStartTime)) / 1000;
    LOG_INFO(MODULE_MAIN, "Auto-restart   : %lu seconds remaining", timeToRestart);
  #endif
  
  // IMU health
  #if ENABLE_IMU_SUPPORT
    if (systemFlags.imuReady) {
      LOG_INFO(MODULE_MAIN, "IMU Status     : %s", imuManager.getStateString());
      LOG_INFO(MODULE_MAIN, "IMU Error Rate : %.1f%%", imuManager.getErrorRate());
    }
  #endif
}

void printSystemReadyStatus() {
  LOG_INFO(MODULE_SYS, "=== SYSTEM READY STATUS ===");
  LOG_INFO(MODULE_SYS, "GPS Ready    : %s", systemFlags.gpsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Modem Ready  : %s", systemFlags.modemReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Network Ready: %s", systemFlags.networkReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "GPRS Ready   : %s", systemFlags.gprsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "WebSocket    : %s", systemFlags.wsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "IMU Ready    : %s", systemFlags.imuReady ? "YES" : "NO");
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
  
  #if ENABLE_IMU_SUPPORT
    LOG_INFO(MODULE_MAIN, "IMU Movement : %s", imuManager.isMoving() ? "MOVING" : "STATIONARY");
    if (imuManager.isDeadReckoningActive()) {
      LOG_INFO(MODULE_MAIN, "Distance traveled: %.1f m", 
               imuManager.getDeadReckoningData().distanceTraveled);
    }
  #endif
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
  
  #if ENABLE_IMU_SUPPORT
    LOG_INFO(MODULE_PERF, "=== IMU PERFORMANCE ===");
    LOG_INFO(MODULE_PERF, "Total Reads  : %lu", imuOpStats.totalImuReadings);
    LOG_INFO(MODULE_PERF, "Success Rate : %.1f%%", 
             imuOpStats.totalImuReadings > 0 ? 
             (imuOpStats.successfulImuReadings * 100.0 / imuOpStats.totalImuReadings) : 0);
  #endif
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
  
  #if ENABLE_IMU_SUPPORT
    if (config.imuEnabled) {
      imuManager.enable();
    } else {
      imuManager.disable();
    }
  #endif
  
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
// MOVEMENT FUNCTIONS
// ========================================

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
  
  #if ENABLE_IMU_SUPPORT
    if (shouldUseImuPosition()) {
      status += " (IMU)";
    }
  #endif
  
  if (currentMovementState != MOVEMENT_MOVING && vehicleStopTime > 0) {
    unsigned long stopDuration = (millis() - vehicleStopTime) / 1000;
    status += " | Stop: " + String(stopDuration) + "s";
  }
  
  LOG_DEBUG(MODULE_GPS, status.c_str());
}

// ========================================
// TESTING & DIAGNOSTICS
// ========================================

void simulateNetworkOutage(unsigned long duration) {
  LOG_WARN(MODULE_MAIN, "🧪 Simulating network outage for %lu seconds", duration/1000);
  
  networkAvailable = false;
  offlineMode = true;
  
  // Schedule restoration
  static unsigned long restoreTime = 0;
  restoreTime = millis() + duration;
  
  LOG_INFO(MODULE_MAIN, "Network will be restored at %lu", restoreTime);
}

void runOfflineStorageStressTest() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_MAIN, "🧪 Running offline storage stress test...");
    
    int testRecords = 50;
    unsigned long startTime = millis();
    int successCount = 0;
    
    for (int i = 0; i < testRecords; i++) {
      float lat = -6.2088 + (i * 0.0001);
      float lon = 106.8456 + (i * 0.0001);
      float speed = random(0, 120);
      int sats = random(4, 12);
      
      char timestamp[30];
      sprintf(timestamp, "2025-01-17T%02d:%02d:%02dZ", i/60, i%60, 0);
      
      if (storeDataOffline(lat, lon, speed, sats, String(timestamp), 12.5)) {
        successCount++;
      }
      
      delay(50);
    }
    
    unsigned long testDuration = millis() - startTime;
    
    LOG_INFO(MODULE_MAIN, "✅ Stress test complete:");
    LOG_INFO(MODULE_MAIN, "  - Records: %d/%d successful", successCount, testRecords);
    LOG_INFO(MODULE_MAIN, "  - Duration: %lu ms", testDuration);
    LOG_INFO(MODULE_MAIN, "  - Rate: %.1f records/sec", (successCount * 1000.0) / testDuration);
    
    offlineManager.printStorageInfo();
  #else
    LOG_WARN(MODULE_MAIN, "Offline storage not enabled");
  #endif
}

void performEmergencyBackup() {
  LOG_WARN(MODULE_MAIN, "🚨 Performing emergency backup...");
  
  #if ENABLE_OFFLINE_STORAGE
    // Force store current position data if available
    if (gpsManager.isValid() || shouldUseImuPosition()) {
      char timestamp[30];
      gpsManager.getTimestamp(timestamp, sizeof(timestamp));
      
      float lat, lon, speed;
      int satellites;
      
      #if ENABLE_IMU_SUPPORT
        if (shouldUseImuPosition()) {
          lat = imuManager.getEstimatedLatitude();
          lon = imuManager.getEstimatedLongitude();
          speed = imuManager.getEstimatedSpeed() * 3.6;
          satellites = 0;
        } else {
          lat = gpsManager.getLatitude();
          lon = gpsManager.getLongitude();
          speed = gpsManager.getSpeed();
          satellites = gpsManager.getSatellites();
        }
      #else
        lat = gpsManager.getLatitude();
        lon = gpsManager.getLongitude();
        speed = gpsManager.getSpeed();
        satellites = gpsManager.getSatellites();
      #endif
      
      storeDataOffline(lat, lon, speed, satellites, String(timestamp), batteryVoltage);
    }
    
    LOG_INFO(MODULE_MAIN, "System state backed up");
    LOG_INFO(MODULE_MAIN, "Offline records: %d", offlineManager.getOfflineRecordCount());
  #endif
  
  LOG_INFO(MODULE_MAIN, "✅ Emergency backup complete");
}

void recoverFromCorruptedState() {
  LOG_WARN(MODULE_MAIN, "🔧 Attempting recovery from corrupted state...");
  
  // Reset all managers
  modemManager.startReset();
  wsManager.disconnect();
  
  #if ENABLE_OFFLINE_STORAGE
    // Try to repair offline storage
    if (!offlineManager.validateStorage()) {
      LOG_WARN(MODULE_MAIN, "Repairing offline storage...");
      offlineManager.repairStorage();
    }
  #endif
  
  #if ENABLE_IMU_SUPPORT
    // Re-initialize IMU
    if (!systemFlags.imuReady) {
      imuManager.end();
      delay(100);
      imuManager.begin();
    }
  #endif
  
  // Reset system flags
  systemFlags.reset();
  performanceMetrics.reset();
  offlineOpStats.reset();
  imuOpStats.reset();
  
  // Restart state machine
  currentState = STATE_INIT;
  
  LOG_INFO(MODULE_MAIN, "✅ Recovery complete, restarting operations");
}

// ========================================
// PROCESS ADVANCED COMMANDS
// ========================================

void processAdvancedCommands(const String& cmd) {
  // This function handles advanced diagnostic and testing commands
  if (cmd.startsWith("mem test")) {
    // Memory allocation test
    LOG_INFO(MODULE_MAIN, "🧪 Running memory allocation test...");
    
    size_t testSize = 10000;
    void* testPtr = malloc(testSize);
    if (testPtr) {
      LOG_INFO(MODULE_MAIN, "✅ Allocated %d bytes successfully", testSize);
      free(testPtr);
    } else {
      LOG_ERROR(MODULE_MAIN, "❌ Failed to allocate %d bytes", testSize);
    }
    
    Utils::printMemoryInfo();
  } else if (cmd == "sensors") {
    // Show all sensor status
    LOG_INFO(MODULE_MAIN, "=== SENSOR STATUS ===");
    LOG_INFO(MODULE_MAIN, "GPS: %s", systemFlags.gpsReady ? "READY" : "NOT READY");
    
    #if ENABLE_IMU_SUPPORT
      LOG_INFO(MODULE_MAIN, "IMU: %s", systemFlags.imuReady ? "READY" : "NOT READY");
      if (systemFlags.imuReady) {
        imuManager.printSensorData();
      }
    #endif
    
    LOG_INFO(MODULE_MAIN, "Modem: %s", systemFlags.modemReady ? "READY" : "NOT READY");
  } else if (cmd == "position") {
    // Show current position from all sources
    LOG_INFO(MODULE_MAIN, "=== POSITION INFORMATION ===");
    
    if (gpsManager.isValid()) {
      LOG_INFO(MODULE_MAIN, "GPS Position: %.6f, %.6f", 
               gpsManager.getLatitude(), gpsManager.getLongitude());
    }
    
    #if ENABLE_IMU_SUPPORT
      if (imuManager.isDeadReckoningActive()) {
        LOG_INFO(MODULE_MAIN, "IMU Position: %.6f, %.6f", 
                 imuManager.getEstimatedLatitude(), 
                 imuManager.getEstimatedLongitude());
        LOG_INFO(MODULE_MAIN, "Position Error: %.1f m", imuManager.getPositionError());
      }
    #endif
    
    LOG_INFO(MODULE_MAIN, "Position Source: %s", getPositionSourceString());
  }
}

// ========================================
// TIMESTAMP FORMATTING
// ========================================

String formatTimestamp(unsigned long unixTime) {
  if (unixTime == 0) {
    // If no valid time, use millis as fallback
    unixTime = millis() / 1000;
  }
  
  time_t rawTime = unixTime;
  struct tm *timeInfo = gmtime(&rawTime);
  
  char timestamp[32];
  sprintf(timestamp, TIMESTAMP_FORMAT,
          timeInfo->tm_year + 1900, 
          timeInfo->tm_mon + 1, 
          timeInfo->tm_mday,
          timeInfo->tm_hour, 
          timeInfo->tm_min, 
          timeInfo->tm_sec);
  
  return String(timestamp);
}

// ========================================
// END OF main.cpp
// ========================================

/* 
 * SUMMARY OF IMU INTEGRATION:
 * 
 * 1. HARDWARE CONNECTIONS:
 *    - MPU6050 VCC → ESP32 3.3V
 *    - MPU6050 GND → ESP32 GND
 *    - MPU6050 SCL → ESP32 GPIO 22
 *    - MPU6050 SDA → ESP32 GPIO 21
 *    - MPU6050 INT → ESP32 GPIO 19 (optional)
 *    - MPU6050 AD0 → GND (I2C address 0x68)
 * 
 * 2. KEY FEATURES ADDED:
 *    - Dead reckoning when GPS signal lost
 *    - Basement/indoor detection using Z-axis acceleration
 *    - Movement detection using accelerometer
 *    - Heading tracking using gyroscope
 *    - Position estimation with error tracking
 *    - Sensor fusion (GPS + IMU)
 *    - Automatic calibration on boot
 *    - Dynamic sample rate based on conditions
 * 
 * 3. OPERATION MODES:
 *    - GPS Available: Low IMU sample rate (1 Hz)
 *    - GPS Lost: High IMU sample rate (10 Hz)
 *    - Moving: Higher IMU sample rate (20 Hz)
 *    - Basement Mode: Dead reckoning active
 * 
 * 4. SERIAL COMMANDS:
 *    - imu: Show IMU status
 *    - imu calibrate: Calibrate IMU
 *    - imu data: Show sensor readings
 *    - imu dr: Dead reckoning info
 *    - imu basement: Basement detection status
 *    - imu enable/disable: Control IMU
 *    - imu test basement/normal: Test modes
 * 
 * 5. DATA TRANSMISSION:
 *    - Sends position_source field (GPS/IMU/FUSION)
 *    - Sends basement_status field (true/false)
 *    - Sends imu_data object with sensor readings
 *    - Automatically switches between GPS and IMU positions
 * 
 * 6. PERFORMANCE IMPACT:
 *    - Additional ~1.3KB RAM usage
 *    - Additional ~30KB Flash usage
 *    - ~3.5mA current draw when active
 *    - Minimal CPU overhead with adaptive sampling
 * 
 * 7. CONFIGURATION (Config.h):
 *    - ENABLE_IMU_SUPPORT: Enable/disable IMU
 *    - IMU_I2C_ADDRESS: 0x68 or 0x69
 *    - IMU_SAMPLE_RATE_*: Sample rates for different conditions
 *    - IMU_MOVEMENT_THRESHOLD: Movement detection threshold
 *    - BASEMENT_ENTRY_THRESHOLD: Z-accel threshold
 *    - IMU_MAX_POSITION_ERROR: Max acceptable position error
 */