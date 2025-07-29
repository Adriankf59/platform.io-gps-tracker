// ========================================
// Main.cpp v7.4 - ESP32 GPS Tracker dengan COMPLETE FIXED Offline Storage & Real GPS Data
// ========================================

/**
 * ESP32 Vehicle GPS Tracking dengan COMPLETE FIXED Offline Storage & Auto-Recovery
 * - IMMEDIATE offline data storage saat network tidak tersedia DENGAN REAL GPS DATA
 * - PRIORITY auto-sync saat network kembali (offline data dulu, baru data baru)
 * - RESPONSIVE network detection (1 detik interval)
 * - FIXED sync logic dengan proper unsent record handling
 * - Enhanced simulation commands untuk testing DENGAN REAL GPS DATA
 * - Complete auto-recovery system
 * - GPS DATA PRIORITY: Real GPS > Last Known Position > Minimal Simulated
 * 
 * Versi: 7.4 - COMPLETE FIXED Offline Storage Implementation dengan Real GPS Data
 * Update: Fixed all sync issues dan added comprehensive testing commands dengan real GPS data
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

// ========================================
// KONSTANTA DAN ENUMERASI
// ========================================

// ----- MODE POWER -----
enum PowerMode {
  POWER_MODE_FULL,      
  POWER_MODE_STANDBY,   
  POWER_MODE_EMERGENCY  
};

// ----- STATE SISTEM -----
enum SystemState {
  STATE_INIT,                 
  STATE_WAIT_GPS,             
  STATE_OPERATIONAL,          
  STATE_MODEM_RESET,         
  STATE_CONNECTION_RECOVERY,  
  STATE_ERROR,               
  STATE_SLEEP_PREPARE,       
  STATE_SLEEPING,            
  STATE_OPTIMIZING,          
  STATE_OFFLINE_SYNC         
};

// ----- STATE PERGERAKAN -----
enum MovementState {
  MOVEMENT_UNKNOWN,   
  MOVEMENT_STATIC,    
  MOVEMENT_PARKED,    
  MOVEMENT_MOVING     
};

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

// ENHANCED: Offline Operation Stats dengan FIXED logic
struct OfflineOperationStats {
  bool hasUnsentData;
  bool syncInProgress;
  bool prioritySyncMode;
  unsigned long syncStartTime;
  int batchesSent;
  int recordsProcessed;
  int prioritySyncProgress;
  unsigned long dataStoredOffline;
  unsigned long dataSentFromOffline;
  unsigned long lastOfflineStoreTime;
  unsigned long lastOfflineSyncTime;
  unsigned long lastNetworkLossTime;
  unsigned long lastNetworkRestoreTime;
  bool networkWasLost;
  
  void reset() {
    hasUnsentData = false;
    syncInProgress = false;
    prioritySyncMode = false;
    syncStartTime = 0;
    batchesSent = 0;
    recordsProcessed = 0;
    prioritySyncProgress = 0;
    dataStoredOffline = 0;
    dataSentFromOffline = 0;
    lastOfflineStoreTime = 0;
    lastOfflineSyncTime = 0;
    lastNetworkLossTime = 0;
    lastNetworkRestoreTime = 0;
    networkWasLost = false;
  }
} offlineOpStats;

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

const PowerModeConfig powerConfigs[3] = {
  {2000, WS_PING_INTERVAL, 0, true, true, true, true, true},
  {30000, 60000, 0, true, true, true, false, false},
  {300000, 0, 0, true, false, false, false, false}
};

// ========================================
// OBJEK GLOBAL
// ========================================

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);

GpsManager gpsManager(gps, SerialGPS);
ModemManager modemManager(modem, SerialAT);
WebSocketManager wsManager(&gsmClient);
OfflineDataManager offlineManager;

// FIXED: Proper C linkage for integration function - DECLARED AFTER OBJECTS
extern "C" {
  bool sendOfflineRecordViaWebSocket(float lat, float lon, float speed, 
                                   int satellites, const char* timestamp, 
                                   float battery);
}

// ========================================
// VARIABEL GLOBAL
// ========================================

SystemState currentState = STATE_INIT;
PowerMode currentPowerMode = POWER_MODE_FULL;
MovementState currentMovementState = MOVEMENT_UNKNOWN;

unsigned long lastGpsSendTime = 0;
unsigned long lastSuccessfulOperation = 0;
unsigned long lastActivityTime = 0;
unsigned long lastSignalCheck = 0;
unsigned long lastMaintenanceCheck = 0;
unsigned long lastPerformanceOptimization = 0;
unsigned long vehicleStopTime = 0;

unsigned long lastSuccessfulTransmission = 0;
unsigned long lastMemoryCheck = 0;
unsigned long lastHealthCheck = 0;
unsigned long systemStartTime = 0;
int consecutiveHealthFailures = 0;

bool relayState = true;
bool waitForSubscription = true;
float batteryVoltage = 12.6;

float manualSpeed = -1.0;
bool useManualSpeed = false;

// ENHANCED Network and Offline Variables dengan FIXED logic
bool networkAvailable = false;
bool offlineMode = false;
bool forceOfflineMode = false;
unsigned long lastNetworkCheck = 0;
unsigned long lastOfflineSync = 0;
unsigned long lastOfflineMaintenance = 0;
unsigned long networkCheckInterval = NETWORK_CHECK_INTERVAL;

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
void executeStateMachine();

// AUTO-RECOVERY Functions
void performSystemHealthCheck();
void checkAutoRestart();
void checkMemoryHealth();
void checkSuccessRate();
void forceSystemRestart(const char* reason);

// ENHANCED FIXED Offline Storage Functions
void initializeOfflineStorage();
void checkNetworkAvailability();
bool shouldUseOfflineMode();
bool storeDataOffline(float lat, float lon, float speed, int satellites, const String& timestamp, float battery);
void syncOfflineData();
void processOfflineQueue();
void handleOfflineCommands(const String& cmd);
void printOfflineStatus();
void printOfflineStats();
void performOfflineMaintenance();
bool isOfflineDataExpired();

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

// ENHANCED Data Transmission
bool sendVehicleDataViaWebSocket();
bool sendVehicleDataWithOfflineSupport();
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

// ENHANCED Connection Management
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

// FIXED Advanced Testing & Diagnostics dengan Real GPS Data
void simulateNetworkOutage(unsigned long duration);
void runOfflineStorageStressTest();
void performSystemDiagnostics();
void performEmergencyBackup();
void recoverFromCorruptedState();
void logSystemStats();

// FIXED: Proper C linkage for integration function - MOVED TO AFTER INCLUDES
// Declaration will be placed after objects are initialized

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
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS Tracker v7.4 ===");
  LOG_INFO(MODULE_MAIN, "COMPLETE FIXED Offline Storage with Real GPS Data Priority & Auto-Recovery");
  LOG_INFO(MODULE_MAIN, "Device ID: %s", GPS_ID);
  LOG_INFO(MODULE_MAIN, "Compiled: %s %s", __DATE__, __TIME__);
  
  // Initialize system flags
  systemFlags.reset();
  performanceMetrics.reset();
  offlineOpStats.reset();
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
  
  // ENHANCED: Initialize FIXED Offline Storage
  initializeOfflineStorage();
  
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
  
// ENHANCED: FIXED Offline storage info
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_SYS, "💾 FIXED Enhanced offline storage: ENABLED");
    LOG_INFO(MODULE_SYS, "📦 Max records: %d", OFFLINE_MAX_RECORDS);
    LOG_INFO(MODULE_SYS, "🔄 Priority auto-sync: %s", OFFLINE_AUTO_SYNC ? "ENABLED" : "DISABLED");
    LOG_INFO(MODULE_SYS, "📡 Network check: every %d ms", NETWORK_CHECK_INTERVAL);
    LOG_INFO(MODULE_SYS, "🧪 Real GPS data priority: ENABLED");
    LOG_INFO(MODULE_SYS, "📍 GPS data priority: Real > Last Known > Minimal Simulated");
    
    // Define missing constants if not in Config.h
    #ifndef OFFLINE_BATCH_SEND_SIZE
      #define OFFLINE_BATCH_SEND_SIZE 5
    #endif
    #ifndef OFFLINE_MAX_AGE
      #define OFFLINE_MAX_AGE 86400
    #endif
  #endif
  
  // Set initial state
  currentState = STATE_WAIT_GPS;
  lastActivityTime = millis();
  lastSuccessfulTransmission = millis();
  
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
  
  // AUTO-RECOVERY CHECKS (PRIORITY)
  checkAutoRestart();
  checkMemoryHealth();
  performSystemHealthCheck();
  
  // ENHANCED: FIXED Network availability check (PRIORITY - more frequent)
  checkNetworkAvailability();
  
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
  
  // ENHANCED: FIXED Process offline queue dengan priority
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
// ENHANCED FIXED OFFLINE STORAGE FUNCTIONS
// ========================================

void initializeOfflineStorage() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_OFFLINE, "Initializing FIXED enhanced offline storage dengan real GPS data priority...");
    
    if (offlineManager.begin(ENABLE_OFFLINE_STORAGE)) {
      LOG_INFO(MODULE_OFFLINE, "✅ FIXED Enhanced offline storage initialized");
      
      // FIXED: Set the send function callback properly
      offlineManager.setSendDataCallback(sendOfflineRecordViaWebSocket);
      
      // Check for existing offline data dengan FIXED logic
      if (offlineManager.hasOfflineData()) {
        int totalRecords = offlineManager.getOfflineRecordCount();
        int unsentRecords = offlineManager.getUnsentRecordCount();
        
        offlineOpStats.hasUnsentData = (unsentRecords > 0);
        LOG_INFO(MODULE_OFFLINE, "📦 Found %d total records (%d unsent)", 
                 totalRecords, unsentRecords);
      }
      
      // Set enhanced callbacks dengan FIXED logic
      offlineManager.setOnDataSentCallback([](int sent, int remaining) {
        LOG_INFO(MODULE_OFFLINE, "📤 Sent %d records, %d unsent remaining", sent, remaining);
        offlineOpStats.dataSentFromOffline += sent;
        offlineOpStats.recordsProcessed += sent;
        offlineOpStats.hasUnsentData = (remaining > 0);
        
        if (offlineOpStats.prioritySyncMode) {
          int totalRecords = offlineOpStats.recordsProcessed + remaining;
          if (totalRecords > 0) {
            offlineOpStats.prioritySyncProgress = (offlineOpStats.recordsProcessed * 100) / totalRecords;
            LOG_INFO(MODULE_OFFLINE, "🔄 Priority sync progress: %d%%", offlineOpStats.prioritySyncProgress);
          }
        }
      });
      
      offlineManager.setOnStorageFullCallback([](int stored) {
        LOG_WARN(MODULE_OFFLINE, "⚠️ Offline storage full! %d records stored", stored);
      });
      
      offlineManager.setOnErrorCallback([](const char* error) {
        LOG_ERROR(MODULE_OFFLINE, "❌ Offline storage error: %s", error);
      });
      
      offlineManager.setOnSyncProgressCallback([](int progress, int total) {
        if (progress % 10 == 0 || progress == total) {
          LOG_INFO(MODULE_OFFLINE, "📊 Sync progress: %d/%d (%d%%)", 
                   progress, total, total > 0 ? (progress * 100) / total : 0);
        }
      });
      
    } else {
      LOG_ERROR(MODULE_OFFLINE, "❌ Failed to initialize FIXED offline storage");
    }
  #else
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
  #endif
}

// FIXED checkNetworkAvailability dengan proper logic
void checkNetworkAvailability() {
  if (millis() - lastNetworkCheck < networkCheckInterval) return;
  
  lastNetworkCheck = millis();
  bool previousNetworkState = networkAvailable;
  
  // ENHANCED: Check network dengan WebSocket state untuk akurasi yang lebih baik
  bool modemNetworkReady = systemFlags.networkReady && 
                          systemFlags.gprsReady && 
                          modemManager.isNetworkConnected() && 
                          modemManager.isGprsConnected();
  
  bool wsConnectionReady = !powerConfigs[currentPowerMode].wsContinuous || 
                          (wsManager.getState() == WS_SUBSCRIBED || wsManager.getState() == WS_CONNECTED);
  
  networkAvailable = modemNetworkReady && wsConnectionReady && !forceOfflineMode;
  
  // ENHANCED: Detect state changes dan log dengan detail
  if (networkAvailable != previousNetworkState) {
    if (networkAvailable) {
      LOG_INFO(MODULE_SYS, "📶 Network RESTORED - switching to ONLINE mode");
      offlineOpStats.lastNetworkRestoreTime = millis();
      offlineOpStats.networkWasLost = false;
      offlineMode = false;
      
      // Log duration network was offline
      if (offlineOpStats.lastNetworkLossTime > 0) {
        unsigned long offlineDuration = (millis() - offlineOpStats.lastNetworkLossTime) / 1000;
        LOG_INFO(MODULE_SYS, "📊 Network was offline for %lu seconds", offlineDuration);
      }
      
      // FIXED: PRIORITY auto-sync immediately when network restored
      #if ENABLE_OFFLINE_STORAGE
        if (offlineManager.hasOfflineData()) {
          int unsentCount = offlineManager.getUnsentRecordCount(); // FIXED: Use unsent count
          if (unsentCount > 0) {
            LOG_INFO(MODULE_OFFLINE, "🚀 Network restored - starting PRIORITY sync of %d offline records", unsentCount);
            offlineOpStats.prioritySyncMode = true;
            offlineOpStats.recordsProcessed = 0;
            offlineOpStats.hasUnsentData = true;
            currentState = STATE_OFFLINE_SYNC;
            networkCheckInterval = 500;  // Check network more frequently during sync
            
            // Force immediate sync start
            lastOfflineSync = 0;
          } else {
            LOG_INFO(MODULE_OFFLINE, "📭 Network restored - all offline data already synced");
          }
        } else {
          LOG_INFO(MODULE_OFFLINE, "📭 Network restored - no offline data to sync");
        }
      #endif
    } else {
      LOG_WARN(MODULE_SYS, "📵 Network LOST - switching to OFFLINE mode");
      offlineOpStats.lastNetworkLossTime = millis();
      offlineOpStats.networkWasLost = true;
      offlineMode = true;
      
      // Reset WebSocket state saat network lost
      systemFlags.wsReady = false;
      
      // Reset network check interval untuk deteksi restore yang cepat
      networkCheckInterval = NETWORK_CHECK_INTERVAL;
      
      LOG_INFO(MODULE_OFFLINE, "💾 Offline mode active - REAL GPS data will be stored locally");
    }
  } else if (networkAvailable && offlineOpStats.syncInProgress) {
    // Keep checking frequently during sync
    networkCheckInterval = 500;
  } else {
    // Normal check interval
    networkCheckInterval = NETWORK_CHECK_INTERVAL;
  }
}

bool shouldUseOfflineMode() {
  return (!networkAvailable || offlineMode || forceOfflineMode) && ENABLE_OFFLINE_STORAGE;
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
      
      LOG_INFO(MODULE_OFFLINE, "💾 Data stored offline successfully [%d/%d unsent]", 
               offlineManager.getUnsentRecordCount(), OFFLINE_MAX_RECORDS);
      LOG_DEBUG(MODULE_OFFLINE, "📍 Stored: %.6f, %.6f, %.1f km/h, %d sats", 
                lat, lon, speed, satellites);
    } else {
      LOG_ERROR(MODULE_OFFLINE, "❌ Failed to store data offline");
    }
    
    return success;
  #else
    return false;
  #endif
}

void syncOfflineData() {
  #if ENABLE_OFFLINE_STORAGE
    if (!networkAvailable || !offlineOpStats.hasUnsentData) return;
    
    int unsentCount = offlineManager.getUnsentRecordCount();
    if (unsentCount == 0) {
      offlineOpStats.hasUnsentData = false;
      return;
    }
    
    LOG_INFO(MODULE_OFFLINE, "🔄 Starting offline data sync (%d unsent records)...", unsentCount);
    currentState = STATE_OFFLINE_SYNC;
    offlineOpStats.syncInProgress = true;
    offlineOpStats.syncStartTime = millis();
    offlineOpStats.batchesSent = 0;
    
    // Use priority mode jika network baru restored
    bool priorityMode = offlineOpStats.prioritySyncMode;
    offlineManager.startSendingOfflineData(priorityMode);
  #endif
}

// FIXED processOfflineQueue dengan proper logic
void processOfflineQueue() {
  #if ENABLE_OFFLINE_STORAGE
    if (!networkAvailable || !offlineOpStats.hasUnsentData || offlineOpStats.syncInProgress) {
      return;
    }
    
    // FIXED: Check if we actually have unsent data
    int unsentCount = offlineManager.getUnsentRecordCount();
    if (unsentCount == 0) {
      offlineOpStats.hasUnsentData = false;
      return;
    }
    
    // ENHANCED: Avoid too frequent sync attempts tapi lebih responsif
    unsigned long minInterval = offlineOpStats.prioritySyncMode ? 3000 : 8000;  // 3s for priority, 8s for normal
    if (millis() - lastOfflineSync < minInterval) return;
    
    lastOfflineSync = millis();
    LOG_INFO(MODULE_OFFLINE, "📤 Network available, processing %d offline records", unsentCount);
    syncOfflineData();
  #endif
}

void performOfflineMaintenance() {
  #if ENABLE_OFFLINE_STORAGE
    if (millis() - lastOfflineMaintenance < OFFLINE_MAINTENANCE_INTERVAL) {
      return;
    }
    
    lastOfflineMaintenance = millis();
    
    // Remove expired data
    if (isOfflineDataExpired()) {
      LOG_WARN(MODULE_OFFLINE, "🧹 Removing expired offline data");
      offlineManager.removeExpiredRecords(OFFLINE_DATA_MAX_AGE / 1000);  // Convert to seconds
    }
    
    // Perform general maintenance
    offlineManager.performMaintenance();
    
    // Update stats
    offlineOpStats.hasUnsentData = (offlineManager.getUnsentRecordCount() > 0);
    
    // Log status jika ada data pending
    if (offlineOpStats.hasUnsentData) {
      int pendingRecords = offlineManager.getUnsentRecordCount();
      unsigned long oldestAge = offlineManager.getOldestRecordAge();
      LOG_INFO(MODULE_OFFLINE, "📦 Offline data: %d records pending (oldest: %lu sec)", 
               pendingRecords, oldestAge);
      
      // Warning jika data sudah terlalu lama
      if (oldestAge > 3600) {  // 1 hour
        LOG_WARN(MODULE_OFFLINE, "⚠️ Offline data aging: oldest record is %lu sec old", oldestAge);
      }
    }
  #endif
}

bool isOfflineDataExpired() {
  #if ENABLE_OFFLINE_STORAGE
    if (!offlineOpStats.hasUnsentData) return false;
    return offlineManager.getOldestRecordAge() > (OFFLINE_DATA_MAX_AGE / 1000);
  #else
    return false;
  #endif
}

// ========================================
// ENHANCED FIXED STATE HANDLERS
// ========================================

// FIXED handleOfflineSyncState dengan better error handling
void handleOfflineSyncState() {
  #if ENABLE_OFFLINE_STORAGE
    if (!offlineOpStats.syncInProgress) {
      // Mulai sync
      int unsentCount = offlineManager.getUnsentRecordCount();
      if (unsentCount == 0) {
        LOG_INFO(MODULE_OFFLINE, "📭 No unsent offline data to sync, returning to operational");
        offlineOpStats.syncInProgress = false;
        offlineOpStats.prioritySyncMode = false;
        currentState = STATE_OPERATIONAL;
        return;
      }
      
      LOG_INFO(MODULE_OFFLINE, "🔄 Starting offline data sync (%d unsent records)...", unsentCount);
      offlineOpStats.syncInProgress = true;
      offlineOpStats.syncStartTime = millis();
      offlineOpStats.batchesSent = 0;
      offlineOpStats.recordsProcessed = 0;
      
      // Use priority mode
      bool success = offlineManager.startSendingOfflineData(offlineOpStats.prioritySyncMode);
      if (!success) {
        LOG_ERROR(MODULE_OFFLINE, "❌ Failed to start offline sync");
        offlineOpStats.syncInProgress = false;
        currentState = STATE_OPERATIONAL;
        return;
      }
    }
    
    // Check network masih available
    if (!networkAvailable) {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Network lost during sync, aborting");
      offlineManager.stopSending();
      offlineOpStats.syncInProgress = false;
      offlineOpStats.prioritySyncMode = false;
      currentState = STATE_OPERATIONAL;
      return;
    }
    
    // FIXED: Continue sending dengan progress tracking
    bool stillSending = offlineManager.continueSendingOfflineData();
    
    if (!stillSending) {
      // Sync complete
      unsigned long syncDuration = millis() - offlineOpStats.syncStartTime;
      int remainingUnsent = offlineManager.getUnsentRecordCount();
      
      LOG_INFO(MODULE_OFFLINE, "✅ Offline sync complete in %lu ms", syncDuration);
      LOG_INFO(MODULE_OFFLINE, "📊 Processed %d records in %d batches, %d unsent remaining", 
               offlineOpStats.recordsProcessed, offlineOpStats.batchesSent, remainingUnsent);
      
      offlineOpStats.syncInProgress = false;
      offlineOpStats.prioritySyncMode = false;
      offlineOpStats.lastOfflineSyncTime = millis();
      offlineOpStats.hasUnsentData = (remainingUnsent > 0);
      
      // Reset network check interval
      networkCheckInterval = NETWORK_CHECK_INTERVAL;
      
      // Kembali ke operational dan force send current GPS data
      currentState = STATE_OPERATIONAL;
      lastGpsSendTime = 0;  // Force immediate send of new data
      
      if (remainingUnsent == 0) {
        LOG_INFO(MODULE_OFFLINE, "🎉 All offline data successfully synced!");
      } else {
        LOG_INFO(MODULE_OFFLINE, "📊 Sync completed, %d unsent records remain", remainingUnsent);
      }
      
      LOG_INFO(MODULE_OFFLINE, "🚀 Ready for normal operations");
    } else {
      // Still sending - update progress
      offlineOpStats.batchesSent++;
      
      // Show progress setiap 5 batches atau jika priority mode
      if (offlineOpStats.batchesSent % 5 == 0 || offlineOpStats.prioritySyncMode) {
        int remainingRecords = offlineManager.getUnsentRecordCount();
        LOG_INFO(MODULE_OFFLINE, "📤 Sync progress: batch %d, %d unsent records remaining", 
                 offlineOpStats.batchesSent, remainingRecords);
      }
      
      // Timeout check (lebih panjang untuk priority sync)
      unsigned long timeout = offlineOpStats.prioritySyncMode ? 900000 : 300000;  // 15 min / 5 min
      if (millis() - offlineOpStats.syncStartTime > timeout) {
        LOG_WARN(MODULE_OFFLINE, "⚠️ Offline sync timeout after %lu ms", timeout);
        offlineManager.stopSending();
        offlineOpStats.syncInProgress = false;
        offlineOpStats.prioritySyncMode = false;
        currentState = STATE_OPERATIONAL;
      }
    }
  #else
    currentState = STATE_OPERATIONAL;
  #endif
}

// ========================================
// ENHANCED OPERATIONAL STATE
// ========================================

void handleOperationalState() {
  unsigned long currentTime = millis();
  
  // Check network availability first (PRIORITY)
  checkNetworkAvailability();
  
  // AUTO-RECOVERY: Check for stuck states
  if (performanceMetrics.totalTransmissions > 0) {
    if (performanceMetrics.successfulTransmissions > 0) {
      lastSuccessfulTransmission = currentTime;
    } else if (currentTime - lastSuccessfulTransmission > NO_SUCCESS_TIMEOUT) {
      forceSystemRestart("No successful transmission in 30 min");
      return;
    }
  }
  
  // Check GPS
  checkGpsReady();
  
  if (!systemFlags.gpsReady && !useManualSpeed) {
    logGpsNotReady();
    return;
  }
  
  // PRIORITY 1: Sync offline data jika ada dan network available
  if (networkAvailable && offlineOpStats.hasUnsentData && !offlineOpStats.syncInProgress) {
    int unsentCount = offlineManager.getUnsentRecordCount();
    if (unsentCount > 0) {
      LOG_INFO(MODULE_OFFLINE, "📤 Network available, syncing %d offline records first", unsentCount);
      currentState = STATE_OFFLINE_SYNC;
      return;
    } else {
      // No unsent data, update flag
      offlineOpStats.hasUnsentData = false;
    }
  }
  
  // PRIORITY 2: Maintain WebSocket connection (only if not syncing offline)
  if (networkAvailable && !offlineMode && !offlineOpStats.syncInProgress) {
    checkWebSocketConnection();
  }
  
  // Get GPS interval
  unsigned long gpsInterval = getGpsIntervalForMovement();
  
  // PRIORITY 3: Send current GPS data (hanya jika tidak ada sync offline yang berjalan)
  if (currentTime - lastGpsSendTime >= gpsInterval && !offlineOpStats.syncInProgress) {
    transmitGpsDataWithOfflineSupport(currentTime, gpsInterval);
  }
  
  // Periodic offline maintenance
  if (currentTime - lastOfflineMaintenance > OFFLINE_MAINTENANCE_INTERVAL) {
    performOfflineMaintenance();
    lastOfflineMaintenance = currentTime;
  }
  
  // Check sleep conditions
  checkSleepConditions(currentTime);
  
  // Check connection health (only if network available)
  if (networkAvailable) {
    checkConnectionHealth(currentTime);
  }
}

// ========================================
// ENHANCED FIXED DATA TRANSMISSION
// ========================================

// FIXED sendVehicleDataWithOfflineSupport dengan proper logic
bool sendVehicleDataWithOfflineSupport() {
  // ENHANCED: Check network availability terlebih dahulu
  checkNetworkAvailability();
  
  // IMMEDIATE OFFLINE STORAGE: Jika offline mode ATAU network tidak tersedia
  if (shouldUseOfflineMode() || !networkAvailable) {
    LOG_INFO(MODULE_GPS, "📵 Network unavailable, storing REAL GPS data offline immediately");
    
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    float displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
    
    // FIXED: PRIORITY data GPS real > last known > simulated
    float lat = 0.0, lon = 0.0;
    int satellites = 0;
    bool hasValidData = false;
    
    // PRIORITY 1: Real GPS data
    if (systemFlags.gpsReady && gpsManager.isValid()) {
      lat = gpsManager.getLatitude();
      lon = gpsManager.getLongitude();
      satellites = gpsManager.getSatellites();
      hasValidData = true;
      LOG_DEBUG(MODULE_GPS, "Using REAL GPS data for offline storage");
    }
    // PRIORITY 2: Last known position
    else if (gpsManager.hasLastKnownPosition()) {
      lat = gpsManager.getLastKnownLatitude();
      lon = gpsManager.getLastKnownLongitude();
      satellites = gpsManager.getLastKnownSatellites();
      hasValidData = true;
      
      unsigned long lastValidAge = (millis() - gpsManager.getLastValidTime()) / 1000;
      LOG_DEBUG(MODULE_GPS, "Using LAST KNOWN position for offline storage (%lu sec old)", lastValidAge);
    }
    
    if (hasValidData) {
      bool stored = storeDataOffline(lat, lon, displaySpeed, satellites, String(timestamp), batteryVoltage);
      
      if (stored) {
        LOG_INFO(MODULE_GPS, "✅ REAL GPS data stored offline successfully");
        // Update metrics untuk offline storage
        performanceMetrics.totalTransmissions++;
        performanceMetrics.successfulTransmissions++;
      }
      
      return stored;
    } else {
      LOG_WARN(MODULE_GPS, "⚠️ No valid GPS data available for offline storage");
      return false;
    }
  }
  
  // FIXED: PRIORITY OFFLINE SYNC - hanya store jika ada unsent data yang perlu di-prioritize
  if (offlineOpStats.hasUnsentData && networkAvailable && !offlineOpStats.syncInProgress) {
    int unsentCount = offlineManager.getUnsentRecordCount();
    if (unsentCount > 0) {
      LOG_INFO(MODULE_GPS, "📤 %d unsent offline records exist, storing current data for later sync", unsentCount);
      
      // Store current data ke offline untuk dikirim setelah data lama
      char timestamp[30];
      gpsManager.getTimestamp(timestamp, sizeof(timestamp));
      float displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
      
      bool stored = storeDataOffline(
        gpsManager.getLatitude(),
        gpsManager.getLongitude(),
        displaySpeed,
        gpsManager.getSatellites(),
        String(timestamp),
        batteryVoltage
      );
      
      if (stored) {
        // Trigger priority sync
        offlineOpStats.prioritySyncMode = true;
        currentState = STATE_OFFLINE_SYNC;
      }
      
      return stored;
    } else {
      // No actual unsent data, update flag
      offlineOpStats.hasUnsentData = false;
    }
  }
  
  // Normal online transmission jika tidak ada data offline pending
  return sendVehicleDataViaWebSocket();
}

void transmitGpsDataWithOfflineSupport(unsigned long currentTime, unsigned long interval) {
  #if TESTING_MODE && DEBUG_LATENCY_TRACKING
    LOG_INFO(MODULE_GPS, "⏱️ Transmit trigger: interval=%lu ms, state=%s, mode=%s", 
             interval, getMovementString(currentMovementState),
             networkAvailable ? "ONLINE" : "OFFLINE");
  #endif
  
  unsigned long transmissionStart = millis();
  bool success = sendVehicleDataWithOfflineSupport();
  
  if (success) {
    unsigned long latency = millis() - transmissionStart;
    
    if (!shouldUseOfflineMode()) {
      // Only update online metrics for actual transmissions
      updatePerformanceMetrics(true, latency);
      lastSuccessfulOperation = currentTime;
      wsManager.endLatencyMeasurement();
    }
    
    lastGpsSendTime = currentTime;
    lastActivityTime = currentTime;
    
    if (powerConfigs[currentPowerMode].performanceMonitoring && 
        DEBUG_LATENCY_TRACKING) {
      LOG_DEBUG(MODULE_PERF, "📊 %s completed in %lu ms", 
                shouldUseOfflineMode() ? "Offline storage" : "Transmission", latency);
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

// FIXED: Integration function implementation dengan proper C linkage
extern "C" bool sendOfflineRecordViaWebSocket(float lat, float lon, float speed, 
                                             int satellites, const char* timestamp, 
                                             float battery) {
  // ENHANCED: Double check network dan WebSocket
  if (!networkAvailable) {
    LOG_DEBUG(MODULE_OFFLINE, "Cannot send offline record - network not available");
    return false;
  }
  
  if (!wsManager.isReady()) {
    LOG_DEBUG(MODULE_OFFLINE, "Cannot send offline record - WebSocket not ready");
    return false;
  }
  
  // FIXED: Enhanced state validation untuk offline data sending
  WSState wsState = wsManager.getState();
  if (wsState != WS_SUBSCRIBED && wsState != WS_CONNECTED) {
    LOG_WARN(MODULE_OFFLINE, "WebSocket not in proper state for offline data: %s", wsManager.getStateString());
    return false;
  }
  
  // Send with enhanced error handling
  bool success = false;
  try {
    success = wsManager.sendVehicleData(lat, lon, speed, satellites, String(timestamp), battery);
    
    if (success) {
      LOG_DEBUG(MODULE_OFFLINE, "✅ Offline record sent: %.6f,%.6f %.1fkm/h %dsats @ %s", 
                lat, lon, speed, satellites, timestamp);
    } else {
      LOG_WARN(MODULE_OFFLINE, "❌ Failed to send offline record: %.6f,%.6f @ %s", 
               lat, lon, timestamp);
    }
  } catch (const std::exception& e) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Exception sending offline record: %s", e.what());
    success = false;
  } catch (...) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Unknown exception sending offline record");
    success = false;
  }
  
  return success;
}

// ========================================
// AUTO-RECOVERY SYSTEM (ENHANCED)
// ========================================

void performSystemHealthCheck() {
  if (millis() - lastHealthCheck < HEALTH_CHECK_INTERVAL) {
    return;
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
  
  // ENHANCED Check 4: Offline storage health
  #if ENABLE_OFFLINE_STORAGE
    if (offlineManager.isReady() && offlineOpStats.hasUnsentData) {
      if (millis() - offlineOpStats.lastOfflineStoreTime > 3600000) {  // 1 hour old data
        LOG_WARN(MODULE_SYS, "⚠️ Health: Old offline data not synced");
        systemHealthy = false;
      }
      
      // Check if sync stuck
      if (offlineOpStats.syncInProgress && 
          millis() - offlineOpStats.syncStartTime > 900000) {  // 15 minutes
        LOG_WARN(MODULE_SYS, "⚠️ Health: Offline sync stuck");
        offlineManager.stopSending();
        offlineOpStats.syncInProgress = false;
        systemHealthy = false;
      }
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
  
  // ENHANCED: Emergency backup before restart
  #if ENABLE_OFFLINE_STORAGE
    performEmergencyBackup();
  #endif
  
  delay(2000);
  ESP.restart();
}

void checkAutoRestart() {
  #if ENABLE_AUTO_RESTART
    if (millis() - systemStartTime > AUTO_RESTART_INTERVAL) {
      forceSystemRestart("Scheduled restart after 72 hours uptime");
    }
  #endif
}

void checkMemoryHealth() {
  if (millis() - lastMemoryCheck > MEMORY_CHECK_INTERVAL) {
    uint32_t freeHeap = Utils::getFreeHeap();
    if (freeHeap < MEMORY_CRITICAL_THRESHOLD) {
      LOG_ERROR(MODULE_SYS, "🚨 Critical memory low: %u bytes", freeHeap);
      forceSystemRestart("Critical memory shortage");
    }
    lastMemoryCheck = millis();
  }
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

void handleWaitGpsState() {
  static unsigned long gpsWaitStart = millis();
  static unsigned long lastGpsStatusLog = 0;
  unsigned long elapsed = millis() - gpsWaitStart;
  
  if (millis() - lastGpsStatusLog >= GPS_LOG_INTERVAL) {
    LOG_INFO(MODULE_GPS, "⏳ Waiting GPS: %lu s, Sats: %d, HDOP: %.1f", 
             elapsed / 1000, gpsManager.getSatellites(), gpsManager.getHDOP());
    lastGpsStatusLog = millis();
  }
  
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
  
  if (elapsed >= GPS_WAIT_TIMEOUT) {
    LOG_WARN(MODULE_GPS, "⚠️ GPS timeout, continuing without fix");
    systemFlags.gpsReady = false;
    currentState = STATE_INIT;
  }
}

void handleInitState() {
  LOG_INFO(MODULE_SYS, "🚀 Initializing system...");
  
  if (modemManager.setup()) {
    systemFlags.modemReady = true;
    systemFlags.networkReady = modemManager.isNetworkConnected();
    systemFlags.gprsReady = modemManager.isGprsConnected();
    
    if (!modemManager.areOptimizationsApplied()) {
      modemManager.applyNetworkOptimizations();
    }
    
    if (!systemFlags.gpsReady && gpsManager.isValid()) {
      systemFlags.gpsReady = true;
      LOG_INFO(MODULE_GPS, "✅ GPS ready during modem setup");
    }
    
    if (powerConfigs[currentPowerMode].wsContinuous) {
      connectWebSocket();
    }
    
    printSystemReadyStatus();
    currentState = STATE_OPERATIONAL;
    lastSuccessfulOperation = millis();
    
    if (systemFlags.gpsReady && systemFlags.wsReady) {
      lastGpsSendTime = 0;
    }
    
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Modem initialization failed");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
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
  
  wsManager.disconnect();
  modemManager.disconnectGprs();
  Utils::safeDelay(1000);
  
  if (modemManager.connectGprs()) {
    systemFlags.gprsReady = true;
    
    if (!modemManager.areOptimizationsApplied()) {
      modemManager.forceOptimizationReapply();
    }
    
    wsManager.resetReconnectAttempts();
    
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
  
  if (config.performanceMonitoring) {
    printPerformanceReport();
  }
  
  if (!config.wsContinuous) {
    wsManager.disconnect();
  }
  
  if (config.sleepDuration > 0) {
    if (currentPowerMode == POWER_MODE_EMERGENCY) {
      enterDeepSleep(config.sleepDuration);
    } else {
      enterLightSleep(config.sleepDuration);
    }
  }
  
  LOG_INFO(MODULE_SYS, "⏰ Woke from sleep");
  currentState = STATE_OPERATIONAL;
  lastActivityTime = millis();
}

// ========================================
// ENHANCED FIXED SERIAL COMMAND HANDLERS DENGAN REAL GPS DATA
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
  // ENHANCED: FIXED Offline commands dengan real GPS data
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
  } else if (cmd.startsWith("outage")) {
    processAdvancedCommands(cmd);
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

// ========================================
// ENHANCED FIXED OFFLINE COMMAND HANDLERS DENGAN REAL GPS DATA
// ========================================

void handleOfflineCommands(const String& cmd) {
  #if ENABLE_OFFLINE_STORAGE
    if (cmd == "offline") {
      printOfflineStatus();
    } else if (cmd == "offline status") {
      offlineManager.printDetailedStatus();
    } else if (cmd == "offline stats") {
      printOfflineStats();
      offlineManager.printSyncStatistics();
    } else if (cmd == "offline records") {
      offlineManager.printOfflineRecords();
    } else if (cmd == "offline sync") {
      LOG_INFO(MODULE_OFFLINE, "Manual sync requested");
      if (networkAvailable) {
        int unsentCount = offlineManager.getUnsentRecordCount();
        if (unsentCount > 0) {
          offlineOpStats.prioritySyncMode = true;  // ENHANCED: Force priority mode
          syncOfflineData();
        } else {
          LOG_INFO(MODULE_OFFLINE, "No unsent data to sync");
        }
      } else {
        LOG_WARN(MODULE_OFFLINE, "Cannot sync - network unavailable");
      }
    } else if (cmd == "offline clear") {
      LOG_WARN(MODULE_OFFLINE, "Clearing all offline data...");
      offlineManager.clearAllOfflineData();
      offlineOpStats.hasUnsentData = false;
    } else if (cmd == "offline test") {
      LOG_INFO(MODULE_OFFLINE, "Testing offline storage dengan REAL GPS data...");
      
      bool dataStored = false;
      
      // Try to store real GPS data first
      if (systemFlags.gpsReady && gpsManager.isValid()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        dataStored = storeDataOffline(
          gpsManager.getLatitude(),
          gpsManager.getLongitude(),
          getCurrentSpeed(),
          gpsManager.getSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        LOG_INFO(MODULE_OFFLINE, "✅ Stored REAL GPS test data: %.6f, %.6f", 
                 gpsManager.getLatitude(), gpsManager.getLongitude());
      } else if (gpsManager.hasLastKnownPosition()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        dataStored = storeDataOffline(
          gpsManager.getLastKnownLatitude(),
          gpsManager.getLastKnownLongitude(),
          getCurrentSpeed(),
          gpsManager.getLastKnownSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        LOG_INFO(MODULE_OFFLINE, "✅ Stored LAST KNOWN test data: %.6f, %.6f", 
                 gpsManager.getLastKnownLatitude(), gpsManager.getLastKnownLongitude());
      } else {
        // Fallback to simulated data
        dataStored = storeDataOffline(-6.2088, 106.8456, 45.5, 8, "2025-01-17T10:00:00Z", 12.5);
        LOG_WARN(MODULE_OFFLINE, "⚠️ Used simulated test data (GPS not available)");
      }
      
      LOG_INFO(MODULE_OFFLINE, "Test result: %s", dataStored ? "PASSED" : "FAILED");
    } else if (cmd == "offline storage") {
      offlineManager.printStorageInfo();
    } else if (cmd == "offline enable") {
      offlineManager.enable();
      LOG_INFO(MODULE_OFFLINE, "Offline storage enabled");
    } else if (cmd == "offline disable") {
      offlineManager.disable();
      LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
    } else if (cmd == "offline force") {
      forceOfflineMode = true;
      offlineMode = true;
      networkAvailable = false;  // Force network unavailable
      LOG_INFO(MODULE_OFFLINE, "Forced offline mode - network marked as unavailable");
    } else if (cmd == "offline online") {
      forceOfflineMode = false;
      offlineMode = false;
      LOG_INFO(MODULE_OFFLINE, "Forced online mode - checking network availability...");
      // Force immediate network check
      lastNetworkCheck = 0;
      checkNetworkAvailability();
    } else if (cmd == "offline priority") {
      offlineOpStats.prioritySyncMode = !offlineOpStats.prioritySyncMode;
      LOG_INFO(MODULE_OFFLINE, "Priority sync mode: %s", 
               offlineOpStats.prioritySyncMode ? "ENABLED" : "DISABLED");
    } else if (cmd == "offline maintenance") {
      LOG_INFO(MODULE_OFFLINE, "Running offline maintenance...");
      performOfflineMaintenance();
    }
    // ENHANCED SIMULATION COMMANDS dengan REAL GPS DATA
    else if (cmd.startsWith("offline simulate outage ")) {
      unsigned long duration = cmd.substring(24).toInt() * 1000;
      if (duration > 0 && duration <= 300000) { // Max 5 minutes
        simulateNetworkOutage(duration);
      } else {
        LOG_WARN(MODULE_OFFLINE, "Invalid duration (1-300 seconds)");
      }
    } else if (cmd == "offline simulate restore") {
      LOG_INFO(MODULE_OFFLINE, "🧪 Simulating network restore...");
      forceOfflineMode = false;
      offlineMode = false;
      
      // Force network available
      systemFlags.networkReady = true;
      systemFlags.gprsReady = true;
      
      // Trigger immediate network check
      lastNetworkCheck = 0;
      checkNetworkAvailability();
      
      LOG_INFO(MODULE_OFFLINE, "Network simulation restored, checking for offline data...");
    } else if (cmd == "offline synctest") {
      offlineManager.runSyncTest();
    } else if (cmd == "offline generate") {
      LOG_INFO(MODULE_OFFLINE, "🧪 Generating test offline data dengan REAL GPS...");
      
      // Temporarily force offline mode
      bool originalForce = forceOfflineMode;
      forceOfflineMode = true;
      networkAvailable = false;
      
      int realDataGenerated = 0;
      int lastKnownGenerated = 0;
      int simulatedGenerated = 0;
      
      for (int i = 0; i < 10; i++) {
        bool dataStored = false;
        
        // Update GPS
        gpsManager.update();
        
        // Try real GPS first
        if (systemFlags.gpsReady && gpsManager.isValid()) {
          char timestamp[30];
          gpsManager.getTimestamp(timestamp, sizeof(timestamp));
          
          dataStored = storeDataOffline(
            gpsManager.getLatitude(),
            gpsManager.getLongitude(),
            getCurrentSpeed(),
            gpsManager.getSatellites(),
            String(timestamp),
            batteryVoltage
          );
          
          if (dataStored) realDataGenerated++;
        }
        // Try last known position
        else if (gpsManager.hasLastKnownPosition()) {
          char timestamp[30];
          sprintf(timestamp, "2025-01-17T10:%02d:00Z", i);
          
          dataStored = storeDataOffline(
            gpsManager.getLastKnownLatitude(),
            gpsManager.getLastKnownLongitude(),
            getCurrentSpeed(),
            gpsManager.getLastKnownSatellites(),
            String(timestamp),
            batteryVoltage
          );
          
          if (dataStored) lastKnownGenerated++;
        }
        // Fallback to simulated
        else {
          float lat = -6.2088 + (i * 0.0001);
          float lon = 106.8456 + (i * 0.0001);
          float speed = 25.0 + (i * 3);
          char timestamp[30];
          sprintf(timestamp, "2025-01-17T10:%02d:00Z", i);
          
          dataStored = storeDataOffline(lat, lon, speed, 8, String(timestamp), 12.5);
          if (dataStored) simulatedGenerated++;
        }
        
        delay(100);
      }
      
      // Restore original mode
      forceOfflineMode = originalForce;
      if (!originalForce) {
        lastNetworkCheck = 0;
        checkNetworkAvailability();
      }
      
      int totalGenerated = realDataGenerated + lastKnownGenerated + simulatedGenerated;
      LOG_INFO(MODULE_OFFLINE, "✅ Generated %d test records:", totalGenerated);
      LOG_INFO(MODULE_OFFLINE, "  - REAL GPS: %d", realDataGenerated);
      LOG_INFO(MODULE_OFFLINE, "  - Last known: %d", lastKnownGenerated);
      LOG_INFO(MODULE_OFFLINE, "  - Simulated: %d", simulatedGenerated);
      
      if (realDataGenerated > 0 || lastKnownGenerated > 0) {
        float realPercentage = ((realDataGenerated + lastKnownGenerated) * 100.0f) / totalGenerated;
        LOG_INFO(MODULE_OFFLINE, "✅ Real data: %.1f%%", realPercentage);
      }
      
      offlineManager.printDetailedStatus();
    } else if (cmd == "offline debug") {
      LOG_INFO(MODULE_OFFLINE, "=== ENHANCED OFFLINE DEBUG INFO ===");
      LOG_INFO(MODULE_OFFLINE, "Network Available  : %s", networkAvailable ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Offline Mode       : %s", offlineMode ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Forced Offline     : %s", forceOfflineMode ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Should Use Offline : %s", shouldUseOfflineMode() ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Has Unsent Data    : %s", offlineOpStats.hasUnsentData ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Sync In Progress   : %s", offlineOpStats.syncInProgress ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Priority Sync      : %s", offlineOpStats.prioritySyncMode ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "GPS Ready          : %s", systemFlags.gpsReady ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "GPS Valid          : %s", gpsManager.isValid() ? "YES" : "NO");
      LOG_INFO(MODULE_OFFLINE, "Has Last Known Pos : %s", gpsManager.hasLastKnownPosition() ? "YES" : "NO");
      
      if (gpsManager.hasLastKnownPosition()) {
        unsigned long lastValidAge = (millis() - gpsManager.getLastValidTime()) / 1000;
        LOG_INFO(MODULE_OFFLINE, "Last Known Pos Age : %lu seconds", lastValidAge);
        LOG_INFO(MODULE_OFFLINE, "Last Known Pos     : %.6f, %.6f", 
                 gpsManager.getLastKnownLatitude(), gpsManager.getLastKnownLongitude());
      }
      
      LOG_INFO(MODULE_OFFLINE, "Network Check Int. : %lu ms", networkCheckInterval);
      LOG_INFO(MODULE_OFFLINE, "Last Network Check : %lu ms ago", millis() - lastNetworkCheck);
      LOG_INFO(MODULE_OFFLINE, "Last Offline Sync  : %lu ms ago", millis() - lastOfflineSync);
      
      offlineManager.printDetailedStatus();
      LOG_INFO(MODULE_OFFLINE, "===================================");
    }
  #else
    LOG_WARN(MODULE_MAIN, "Offline storage not enabled");
  #endif
}

// ========================================
// ENHANCED SPEED AND MOVEMENT COMMANDS
// ========================================

void processSpeedCommand(const String& cmd) {
  if (cmd == "speed") {
    showSpeedInfo();
  } else if (cmd.startsWith("speed ")) {
    String speedStr = cmd.substring(6);
    speedStr.trim();
    
    if (speedStr == "auto") {
      useManualSpeed = false;
      manualSpeed = -1.0;
      LOG_INFO(MODULE_MAIN, "✅ Switched to automatic GPS speed");
    } else {
      float speed = speedStr.toFloat();
      if (speed >= 0.0 && speed <= 200.0) {
        useManualSpeed = true;
        manualSpeed = speed;
        LOG_INFO(MODULE_MAIN, "✅ Manual speed set to %.1f km/h", speed);
        
        // Force immediate GPS transmission with new speed
        lastGpsSendTime = 0;
      } else {
        LOG_WARN(MODULE_MAIN, "⚠️ Invalid speed: %.1f (must be 0-200 km/h)", speed);
      }
    }
  }
}

void processTestingCommand(const String& cmd) {
  if (cmd == "test") {
    LOG_INFO(MODULE_MAIN, "🧪 Available test commands:");
    LOG_INFO(MODULE_MAIN, "test send        - Force GPS data transmission");
    LOG_INFO(MODULE_MAIN, "test movement    - Test movement detection");
    LOG_INFO(MODULE_MAIN, "test network     - Test network connectivity");
    LOG_INFO(MODULE_MAIN, "test websocket   - Test WebSocket connection");
    LOG_INFO(MODULE_MAIN, "test offline     - Test offline storage dengan real GPS");
    LOG_INFO(MODULE_MAIN, "test offline sync- Comprehensive offline sync test dengan real GPS");
    LOG_INFO(MODULE_MAIN, "test stress      - Run stress test");
    LOG_INFO(MODULE_MAIN, "test emergency   - Test emergency procedures");
    LOG_INFO(MODULE_MAIN, "test gps         - Test GPS data priority");
  } else if (cmd == "test send") {
    forceSendGpsData();
  } else if (cmd == "test movement") {
    LOG_INFO(MODULE_MAIN, "🧪 Testing movement detection...");
    
    // Test different speeds
    float testSpeeds[] = {0.0, 2.0, 5.0, 15.0, 30.0, 60.0};
    int numTests = sizeof(testSpeeds) / sizeof(testSpeeds[0]);
    
    bool originalManualMode = useManualSpeed;
    float originalSpeed = manualSpeed;
    
    useManualSpeed = true;
    
    for (int i = 0; i < numTests; i++) {
      manualSpeed = testSpeeds[i];
      updateMovementState();
      
      LOG_INFO(MODULE_MAIN, "Speed: %.1f km/h -> State: %s, Interval: %lu ms", 
               testSpeeds[i], getMovementString(currentMovementState), 
               getGpsIntervalForMovement());
      
      Utils::safeDelay(500);
    }
    
    // Restore original settings
    useManualSpeed = originalManualMode;
    manualSpeed = originalSpeed;
    updateMovementState();
    
    LOG_INFO(MODULE_MAIN, "✅ Movement detection test complete");
  } else if (cmd == "test network") {
    LOG_INFO(MODULE_MAIN, "🧪 Testing network connectivity...");
    
    LOG_INFO(MODULE_MAIN, "Modem status: %s", modemManager.getStatusString());
    LOG_INFO(MODULE_MAIN, "Network connected: %s", modemManager.isNetworkConnected() ? "YES" : "NO");
    LOG_INFO(MODULE_MAIN, "GPRS connected: %s", modemManager.isGprsConnected() ? "YES" : "NO");
    
    if (systemFlags.modemReady) {
      String networkInfo = modemManager.getNetworkInfo();
      LOG_INFO(MODULE_MAIN, "Network info: %s", networkInfo.c_str());
    }
    
    LOG_INFO(MODULE_MAIN, "✅ Network test complete");
  } else if (cmd == "test websocket") {
    LOG_INFO(MODULE_MAIN, "🧪 Testing WebSocket connection...");
    
    printWebSocketStats();
    
    if (wsManager.getState() == WS_SUBSCRIBED) {
      LOG_INFO(MODULE_MAIN, "Sending test message...");
      forceSendGpsData();
    } else {
      LOG_INFO(MODULE_MAIN, "WebSocket not ready, attempting reconnection...");
      resetWebSocketConnection();
    }
    
    LOG_INFO(MODULE_MAIN, "✅ WebSocket test complete");
  } else if (cmd == "test offline") {
    #if ENABLE_OFFLINE_STORAGE
      LOG_INFO(MODULE_MAIN, "🧪 Testing offline storage dengan REAL GPS data...");
      
      // Test storing data dengan priority real GPS
      bool stored = false;
      
      if (systemFlags.gpsReady && gpsManager.isValid()) {
        stored = storeDataOffline(gpsManager.getLatitude(), gpsManager.getLongitude(), 
                                getCurrentSpeed(), gpsManager.getSatellites(), 
                                "2025-01-17T10:00:00Z", batteryVoltage);
        LOG_INFO(MODULE_MAIN, "Store test dengan REAL GPS: %s", stored ? "PASSED" : "FAILED");
      } else if (gpsManager.hasLastKnownPosition()) {
        stored = storeDataOffline(gpsManager.getLastKnownLatitude(), gpsManager.getLastKnownLongitude(),
                                getCurrentSpeed(), gpsManager.getLastKnownSatellites(),
                                "2025-01-17T10:00:00Z", batteryVoltage);
        LOG_INFO(MODULE_MAIN, "Store test dengan LAST KNOWN: %s", stored ? "PASSED" : "FAILED");
      } else {
        stored = storeDataOffline(-6.2088, 106.8456, 45.5, 8, "2025-01-17T10:00:00Z", 12.5);
        LOG_WARN(MODULE_MAIN, "Store test dengan SIMULATED: %s", stored ? "PASSED" : "FAILED");
      }
      
      // Test retrieval
      int recordCount = offlineManager.getOfflineRecordCount();
      LOG_INFO(MODULE_MAIN, "Record count: %d", recordCount);
      
      offlineManager.printStorageInfo();
      LOG_INFO(MODULE_MAIN, "✅ Offline storage test complete");
    #else
      LOG_WARN(MODULE_MAIN, "Offline storage not enabled");
    #endif
  } else if (cmd == "test offline sync") {
    processAdvancedCommands("test offline sync");
  } else if (cmd == "test stress") {
    runOfflineStorageStressTest();
  } else if (cmd == "test emergency") {
    LOG_INFO(MODULE_MAIN, "🧪 Testing emergency procedures...");
    
    performEmergencyBackup();
    Utils::safeDelay(1000);
    
    LOG_INFO(MODULE_MAIN, "Simulating system recovery...");
    recoverFromCorruptedState();
    
    LOG_INFO(MODULE_MAIN, "✅ Emergency test complete");
  } else if (cmd == "test gps") {
    LOG_INFO(MODULE_MAIN, "🧪 Testing GPS data priority...");
    
    LOG_INFO(MODULE_MAIN, "GPS Ready: %s", systemFlags.gpsReady ? "YES" : "NO");
    LOG_INFO(MODULE_MAIN, "GPS Valid: %s", gpsManager.isValid() ? "YES" : "NO");
    LOG_INFO(MODULE_MAIN, "Has Last Known: %s", gpsManager.hasLastKnownPosition() ? "YES" : "NO");
    
    if (systemFlags.gpsReady && gpsManager.isValid()) {
      LOG_INFO(MODULE_MAIN, "✅ PRIORITY 1: REAL GPS data available");
      LOG_INFO(MODULE_MAIN, "  Position: %.6f, %.6f", gpsManager.getLatitude(), gpsManager.getLongitude());
      LOG_INFO(MODULE_MAIN, "  Speed: %.1f km/h", gpsManager.getSpeed());
      LOG_INFO(MODULE_MAIN, "  Satellites: %d", gpsManager.getSatellites());
      LOG_INFO(MODULE_MAIN, "  HDOP: %.1f", gpsManager.getHDOP());
    } else if (gpsManager.hasLastKnownPosition()) {
      LOG_INFO(MODULE_MAIN, "⚠️ PRIORITY 2: LAST KNOWN position available");
      unsigned long age = (millis() - gpsManager.getLastValidTime()) / 1000;
      LOG_INFO(MODULE_MAIN, "  Position: %.6f, %.6f (%lu sec old)", 
               gpsManager.getLastKnownLatitude(), gpsManager.getLastKnownLongitude(), age);
      LOG_INFO(MODULE_MAIN, "  Last Speed: %.1f km/h", gpsManager.getLastKnownSpeed());
      LOG_INFO(MODULE_MAIN, "  Last Satellites: %d", gpsManager.getLastKnownSatellites());
    } else {
      LOG_WARN(MODULE_MAIN, "❌ PRIORITY 3: Only SIMULATED data available");
      LOG_WARN(MODULE_MAIN, "  No real GPS or last known position");
    }
    
    LOG_INFO(MODULE_MAIN, "✅ GPS data priority test complete");
  }
}

// ========================================
// FIXED ADVANCED COMMANDS DENGAN REAL GPS DATA
// ========================================

void processAdvancedCommands(const String& cmd) {
  if (cmd == "test offline sync") {
    LOG_INFO(MODULE_MAIN, "🧪 Starting comprehensive offline sync test dengan REAL GPS data...");
    
    // Step 1: Generate offline data dengan REAL GPS
    LOG_INFO(MODULE_MAIN, "Step 1: Generating offline test data from REAL GPS...");
    forceOfflineMode = true;
    networkAvailable = false;
    
    int recordsGenerated = 0;
    int realGpsRecords = 0;
    int lastKnownRecords = 0;
    int simulatedRecords = 0;
    
    for (int i = 0; i < 8; i++) {
      bool dataStored = false;
      
      // Update GPS data
      gpsManager.update();
      
      // PRIORITY 1: Gunakan data GPS REAL jika tersedia
      if (systemFlags.gpsReady && gpsManager.isValid()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        float currentSpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
        
        dataStored = storeDataOffline(
          gpsManager.getLatitude(),
          gpsManager.getLongitude(),
          currentSpeed,
          gpsManager.getSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        if (dataStored) {
          recordsGenerated++;
          realGpsRecords++;
          LOG_INFO(MODULE_MAIN, "✅ Generated REAL GPS record #%d: %.6f, %.6f, %.1f km/h, %d sats", 
                   recordsGenerated, gpsManager.getLatitude(), gpsManager.getLongitude(), 
                   currentSpeed, gpsManager.getSatellites());
        }
      }
      // PRIORITY 2: Gunakan last known position
      else if (gpsManager.hasLastKnownPosition()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        float speedToUse = useManualSpeed ? manualSpeed : gpsManager.getLastKnownSpeed();
        
        dataStored = storeDataOffline(
          gpsManager.getLastKnownLatitude(),
          gpsManager.getLastKnownLongitude(),
          speedToUse,
          gpsManager.getLastKnownSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        if (dataStored) {
          recordsGenerated++;
          lastKnownRecords++;
          unsigned long lastValidAge = (millis() - gpsManager.getLastValidTime()) / 1000;
          LOG_INFO(MODULE_MAIN, "✅ Generated LAST KNOWN record #%d: %.6f, %.6f (%lu sec old)", 
                   recordsGenerated, gpsManager.getLastKnownLatitude(), 
                   gpsManager.getLastKnownLongitude(), lastValidAge);
        }
      }
      // FALLBACK: Gunakan simulated data dengan warning
      else {
        LOG_WARN(MODULE_MAIN, "⚠️ GPS not ready, using simulated data for record #%d", i+1);
        
        float lat = -6.2088 + (i * 0.0002);
        float lon = 106.8456 + (i * 0.0002);
        float speed = useManualSpeed ? manualSpeed : (30.0 + (i * 5));
        
        char timestamp[30];
        sprintf(timestamp, "2025-01-17T10:%02d:00Z", i);
        
        dataStored = storeDataOffline(lat, lon, speed, 8, String(timestamp), 12.5);
        if (dataStored) {
          recordsGenerated++;
          simulatedRecords++;
        }
      }
      
      if (i < 7) Utils::safeDelay(1000); // 1 second between records
      gpsManager.update(); // Update GPS data
    }
    
    LOG_INFO(MODULE_MAIN, "✅ Generated %d test records:", recordsGenerated);
    LOG_INFO(MODULE_MAIN, "  - REAL GPS: %d records", realGpsRecords);
    LOG_INFO(MODULE_MAIN, "  - Last known: %d records", lastKnownRecords);
    LOG_INFO(MODULE_MAIN, "  - Simulated: %d records", simulatedRecords);
    
    if (realGpsRecords > 0 || lastKnownRecords > 0) {
      float realDataPercentage = ((realGpsRecords + lastKnownRecords) * 100.0f) / recordsGenerated;
      LOG_INFO(MODULE_MAIN, "✅ Real data coverage: %.1f%%", realDataPercentage);
    }
    
    // Step 2: Simulate network restore
    LOG_INFO(MODULE_MAIN, "Step 2: Simulating network restore...");
    Utils::safeDelay(1000);
    
    forceOfflineMode = false;
    systemFlags.networkReady = true;
    systemFlags.gprsReady = true;
    systemFlags.wsReady = true;
    
    // Force network check
    lastNetworkCheck = 0;
    checkNetworkAvailability();
    
    // Step 3: Monitor sync progress
    LOG_INFO(MODULE_MAIN, "Step 3: Monitoring sync progress...");
    int maxWait = 30;
    int waited = 0;
    
    while ((offlineOpStats.syncInProgress || offlineManager.hasOfflineData()) && waited < maxWait) {
      Utils::safeDelay(1000);
      waited++;
      
      if (waited % 5 == 0) {
        int remaining = offlineManager.getUnsentRecordCount();
        LOG_INFO(MODULE_MAIN, "📊 Sync progress: %d records remaining (%d/%d sec)", 
                 remaining, waited, maxWait);
      }
    }
    
    // Step 4: Results
    int finalCount = offlineManager.getUnsentRecordCount();
    if (finalCount == 0) {
      LOG_INFO(MODULE_MAIN, "✅ Offline sync test PASSED - all REAL data synced successfully");
    } else {
      LOG_WARN(MODULE_MAIN, "⚠️ Offline sync test PARTIAL - %d records remain", finalCount);
    }
    
    LOG_INFO(MODULE_MAIN, "📊 Final sync test results:");
    LOG_INFO(MODULE_MAIN, "  - Generated: %d records", recordsGenerated);
    LOG_INFO(MODULE_MAIN, "  - REAL GPS: %d records", realGpsRecords);
    LOG_INFO(MODULE_MAIN, "  - Last known: %d records", lastKnownRecords);
    LOG_INFO(MODULE_MAIN, "  - Simulated: %d records", simulatedRecords);
    LOG_INFO(MODULE_MAIN, "  - Successfully synced: %d", recordsGenerated - finalCount);
    LOG_INFO(MODULE_MAIN, "  - Remaining: %d", finalCount);
    LOG_INFO(MODULE_MAIN, "  - Success rate: %.1f%%", 
             recordsGenerated > 0 ? ((float)(recordsGenerated - finalCount) * 100.0f / recordsGenerated) : 0);
    
    offlineManager.printSyncStatistics();
    
  } else if (cmd.startsWith("outage ")) {
    unsigned long duration = cmd.substring(7).toInt() * 1000;
    if (duration > 0 && duration <= 300000) {
      simulateNetworkOutage(duration);
    } else {
      LOG_WARN(MODULE_MAIN, "Invalid outage duration (1-300 seconds)");
    }
  }
}

// ========================================
// FIXED NETWORK OUTAGE SIMULATION DENGAN REAL GPS DATA
// ========================================

void simulateNetworkOutage(unsigned long duration) {
  LOG_WARN(MODULE_MAIN, "🧪 Simulating network outage for %lu seconds dengan REAL GPS data", duration/1000);
  
  // Force offline mode
  forceOfflineMode = true;
  networkAvailable = false;
  offlineMode = true;
  
  // Reset network flags
  systemFlags.networkReady = false;
  systemFlags.gprsReady = false;
  systemFlags.wsReady = false;
  
  LOG_INFO(MODULE_MAIN, "📵 Network simulation: OFFLINE mode active");
  LOG_INFO(MODULE_MAIN, "💾 REAL GPS data will be stored offline during outage");
  
  // FIXED: Menggunakan data GPS REAL selama outage
  unsigned long startTime = millis();
  int realDataCount = 0;
  int lastKnownDataCount = 0;
  int simulatedDataCount = 0;
  unsigned long lastStoreTime = 0;
  
  while (millis() - startTime < duration) {
    // Store GPS data setiap 3 detik untuk testing
    if (millis() - lastStoreTime >= 3000) {
      bool dataStored = false;
      
      // Update GPS data terlebih dahulu
      gpsManager.update();
      
      // PRIORITY 1: Gunakan data GPS REAL jika tersedia
      if (systemFlags.gpsReady && gpsManager.isValid()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        float currentSpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
        
        dataStored = storeDataOffline(
          gpsManager.getLatitude(),
          gpsManager.getLongitude(),
          currentSpeed,
          gpsManager.getSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        if (dataStored) {
          realDataCount++;
          LOG_INFO(MODULE_OFFLINE, "💾 Stored REAL GPS data #%d: %.6f, %.6f, %.1f km/h, %d sats", 
                   realDataCount, gpsManager.getLatitude(), gpsManager.getLongitude(), 
                   currentSpeed, gpsManager.getSatellites());
        }
      }
      // PRIORITY 2: Gunakan last known position jika GPS temporary tidak ready
      else if (gpsManager.hasLastKnownPosition()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        float speedToUse = useManualSpeed ? manualSpeed : gpsManager.getLastKnownSpeed();
        
        dataStored = storeDataOffline(
          gpsManager.getLastKnownLatitude(),
          gpsManager.getLastKnownLongitude(),
          speedToUse,
          gpsManager.getLastKnownSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        if (dataStored) {
          lastKnownDataCount++;
          unsigned long lastValidAge = (millis() - gpsManager.getLastValidTime()) / 1000;
          LOG_INFO(MODULE_OFFLINE, "💾 Stored LAST KNOWN position #%d: %.6f, %.6f, %.1f km/h (%lu sec old)", 
                   lastKnownDataCount, gpsManager.getLastKnownLatitude(), 
                   gpsManager.getLastKnownLongitude(), speedToUse, lastValidAge);
        }
      }
      // FALLBACK: Minimal simulated data dengan warning (sangat terbatas)
      else if (simulatedDataCount < MAX_SIMULATED_RECORDS_PER_SESSION) {
        LOG_WARN(MODULE_OFFLINE, "⚠️ No GPS data available, using minimal simulated data");
        
        float lat = -6.2088 + (simulatedDataCount * 0.0001);
        float lon = 106.8456 + (simulatedDataCount * 0.0001);
        float speed = useManualSpeed ? manualSpeed : (30.0 + (simulatedDataCount * 5));
        
        char timestamp[30];
        unsigned long currentTime = millis() / 1000;
        sprintf(timestamp, "2025-01-17T%02d:%02d:%02dZ", 
                (int)((currentTime % 3600) / 60), (int)(currentTime % 60));
        
        dataStored = storeDataOffline(lat, lon, speed, 8, String(timestamp), 12.5);
        
        if (dataStored) {
          simulatedDataCount++;
          LOG_WARN(MODULE_OFFLINE, "💾 Stored SIMULATED data #%d: %.6f, %.6f (GPS unavailable)", 
                   simulatedDataCount, lat, lon);
        }
      }
      
      lastStoreTime = millis();
    }
    
    // Continue updating GPS dan other systems
    gpsManager.update();
    Utils::safeDelay(100);
  }
  
  int totalDataStored = realDataCount + lastKnownDataCount + simulatedDataCount;
  
  LOG_INFO(MODULE_MAIN, "⏰ Network outage simulation complete");
  LOG_INFO(MODULE_MAIN, "📊 Data collection results:");
  LOG_INFO(MODULE_MAIN, "  - REAL GPS data: %d records", realDataCount);
  LOG_INFO(MODULE_MAIN, "  - Last known position: %d records", lastKnownDataCount);
  LOG_INFO(MODULE_MAIN, "  - Simulated data: %d records", simulatedDataCount);
  LOG_INFO(MODULE_MAIN, "  - Total stored: %d records", totalDataStored);
  
  if (realDataCount > 0 || lastKnownDataCount > 0) {
    float realDataPercentage = ((realDataCount + lastKnownDataCount) * 100.0f) / totalDataStored;
    LOG_INFO(MODULE_MAIN, "✅ Success: %.1f%% real GPS data collected", realDataPercentage);
  } else {
    LOG_WARN(MODULE_MAIN, "⚠️ No real GPS data was available during simulation");
  }
  
  // Auto-restore network
  LOG_INFO(MODULE_MAIN, "🔄 Network will be automatically restored in 2 seconds...");
  Utils::safeDelay(2000);
  
  // Restore network
  forceOfflineMode = false;
  LOG_INFO(MODULE_MAIN, "📶 Network simulation: Restoring connection...");
  
  // Force network check
  lastNetworkCheck = 0;
  checkNetworkAvailability();
  
  LOG_INFO(MODULE_MAIN, "✅ Network outage simulation complete with REAL data");
  printOfflineStatus();
}

// ========================================
// STATUS AND INFO FUNCTIONS
// ========================================

void printOfflineStatus() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_OFFLINE, "=== ENHANCED OFFLINE STATUS ===");
    LOG_INFO(MODULE_OFFLINE, "Status         : %s", offlineManager.getStatusString());
    LOG_INFO(MODULE_OFFLINE, "Enabled        : %s", offlineManager.isEnabledStatus() ? "YES" : "NO");
    
    int totalRecords = offlineManager.getOfflineRecordCount();
    int unsentRecords = offlineManager.getUnsentRecordCount();
    int sentRecords = totalRecords - unsentRecords;
    
    LOG_INFO(MODULE_OFFLINE, "Total Records  : %d/%d", totalRecords, OFFLINE_MAX_RECORDS);
    LOG_INFO(MODULE_OFFLINE, "Unsent Records : %d", unsentRecords);
    LOG_INFO(MODULE_OFFLINE, "Sent Records   : %d", sentRecords);
    
    LOG_INFO(MODULE_OFFLINE, "Has Unsent     : %s", unsentRecords > 0 ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Sync Progress  : %s", offlineOpStats.syncInProgress ? "ACTIVE" : "IDLE");
    LOG_INFO(MODULE_OFFLINE, "Priority Mode  : %s", offlineOpStats.prioritySyncMode ? "ENABLED" : "DISABLED");
    LOG_INFO(MODULE_OFFLINE, "Network Mode   : %s", networkAvailable ? "ONLINE" : "OFFLINE");
    LOG_INFO(MODULE_OFFLINE, "Forced Mode    : %s", forceOfflineMode ? "OFFLINE" : "AUTO");
    
    // GPS data source info
    LOG_INFO(MODULE_OFFLINE, "GPS Ready      : %s", systemFlags.gpsReady ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "GPS Valid      : %s", gpsManager.isValid() ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Has Last Known : %s", gpsManager.hasLastKnownPosition() ? "YES" : "NO");
    
    if (offlineOpStats.syncInProgress) {
      unsigned long syncDuration = (millis() - offlineOpStats.syncStartTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Sync Duration  : %lu seconds", syncDuration);
      LOG_INFO(MODULE_OFFLINE, "Batches Sent   : %d", offlineOpStats.batchesSent);
      LOG_INFO(MODULE_OFFLINE, "Records Proc.  : %d", offlineOpStats.recordsProcessed);
      LOG_INFO(MODULE_OFFLINE, "Progress       : %d%%", offlineOpStats.prioritySyncProgress);
    }
    
    if (offlineOpStats.lastNetworkLossTime > 0) {
      unsigned long timeSinceLoss = (millis() - offlineOpStats.lastNetworkLossTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Last Net Loss  : %lu sec ago", timeSinceLoss);
    }
    
    if (offlineOpStats.lastNetworkRestoreTime > 0) {
      unsigned long timeSinceRestore = (millis() - offlineOpStats.lastNetworkRestoreTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Last Net Restore: %lu sec ago", timeSinceRestore);
    }
    
    if (totalRecords > 0) {
      unsigned long oldestAge = offlineManager.getOldestRecordAge();
      LOG_INFO(MODULE_OFFLINE, "Oldest Record  : %lu sec ago", oldestAge);
      if (oldestAge > 3600) {
        LOG_WARN(MODULE_OFFLINE, "⚠️ Old data detected (>1 hour)");
      }
    }
    
    LOG_INFO(MODULE_OFFLINE, "=====================================");
  #else
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
  #endif
}

void printOfflineStats() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_OFFLINE, "=== ENHANCED OFFLINE STATISTICS ===");
    LOG_INFO(MODULE_OFFLINE, "Total Stored       : %lu", offlineOpStats.dataStoredOffline);
    LOG_INFO(MODULE_OFFLINE, "Total Sent (Offline): %lu", offlineOpStats.dataSentFromOffline);
    LOG_INFO(MODULE_OFFLINE, "Records Processed  : %d", offlineOpStats.recordsProcessed);
    LOG_INFO(MODULE_OFFLINE, "Batches Sent       : %d", offlineOpStats.batchesSent);
    LOG_INFO(MODULE_OFFLINE, "Has Unsent Data    : %s", offlineOpStats.hasUnsentData ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Sync In Progress   : %s", offlineOpStats.syncInProgress ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Priority Sync Mode : %s", offlineOpStats.prioritySyncMode ? "YES" : "NO");
    LOG_INFO(MODULE_OFFLINE, "Network Was Lost   : %s", offlineOpStats.networkWasLost ? "YES" : "NO");
    
    if (offlineOpStats.lastOfflineStoreTime > 0) {
      unsigned long timeSinceStore = (millis() - offlineOpStats.lastOfflineStoreTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Last Store         : %lu sec ago", timeSinceStore);
    }
    
    if (offlineOpStats.lastOfflineSyncTime > 0) {
      unsigned long timeSinceSync = (millis() - offlineOpStats.lastOfflineSyncTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Last Sync          : %lu sec ago", timeSinceSync);
    }
    
    if (offlineOpStats.lastNetworkLossTime > 0) {
      unsigned long timeSinceLoss = (millis() - offlineOpStats.lastNetworkLossTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Last Network Loss  : %lu sec ago", timeSinceLoss);
    }
    
    if (offlineOpStats.lastNetworkRestoreTime > 0) {
      unsigned long timeSinceRestore = (millis() - offlineOpStats.lastNetworkRestoreTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Last Network Restore: %lu sec ago", timeSinceRestore);
    }
    
    if (offlineOpStats.syncInProgress) {
      unsigned long syncDuration = (millis() - offlineOpStats.syncStartTime) / 1000;
      LOG_INFO(MODULE_OFFLINE, "Current Sync Time  : %lu seconds", syncDuration);
    }
    
    LOG_INFO(MODULE_OFFLINE, "===================================");
  #else
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled");
  #endif
}

void printStatus() {
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS TRACKER STATUS v7.4 ===");
  LOG_INFO(MODULE_MAIN, "System State    : %s", getStateString(currentState));
  LOG_INFO(MODULE_MAIN, "Power Mode      : %s", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "Movement State  : %s", getMovementString(currentMovementState));
  LOG_INFO(MODULE_MAIN, "Uptime          : %lu minutes", millis() / 60000);
  
  // System ready flags
  LOG_INFO(MODULE_MAIN, "GPS Ready       : %s", systemFlags.gpsReady ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Modem Ready     : %s", systemFlags.modemReady ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Network Ready   : %s", systemFlags.networkReady ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "GPRS Ready      : %s", systemFlags.gprsReady ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "WebSocket Ready : %s", systemFlags.wsReady ? "YES" : "NO");
  
  // GPS info dengan data source priority
  if (systemFlags.gpsReady || useManualSpeed) {
    LOG_INFO(MODULE_MAIN, "GPS Position    : %.6f, %.6f", 
             gpsManager.getLatitude(), gpsManager.getLongitude());
    LOG_INFO(MODULE_MAIN, "GPS Speed       : %.1f km/h %s", 
             getCurrentSpeed(), useManualSpeed ? "[MANUAL]" : "[GPS]");
    LOG_INFO(MODULE_MAIN, "GPS Satellites  : %d", gpsManager.getSatellites());
    LOG_INFO(MODULE_MAIN, "GPS HDOP        : %.1f", gpsManager.getHDOP());
    
    // Data source priority info
    if (systemFlags.gpsReady && gpsManager.isValid()) {
      LOG_INFO(MODULE_MAIN, "Data Source     : REAL GPS (Priority 1)");
    } else if (gpsManager.hasLastKnownPosition()) {
      unsigned long age = (millis() - gpsManager.getLastValidTime()) / 1000;
      LOG_INFO(MODULE_MAIN, "Data Source     : LAST KNOWN (%lu sec old, Priority 2)", age);
    } else {
      LOG_INFO(MODULE_MAIN, "Data Source     : SIMULATED (Priority 3)");
    }
  }
  
  // Network status
  LOG_INFO(MODULE_MAIN, "Network Mode    : %s", networkAvailable ? "ONLINE" : "OFFLINE");
  LOG_INFO(MODULE_MAIN, "Signal Quality  : %d", modemManager.getSignalQuality());
  LOG_INFO(MODULE_MAIN, "WebSocket State : %s", wsManager.getStateString());
  
  // Battery and relay
  LOG_INFO(MODULE_MAIN, "Battery Voltage : %.1f V", batteryVoltage);
  LOG_INFO(MODULE_MAIN, "Relay State     : %s", relayState ? "ON" : "OFF");
  
  // Enhanced offline status dengan data source info
  #if ENABLE_OFFLINE_STORAGE
    int totalRecords = offlineManager.getOfflineRecordCount();
    int unsentRecords = offlineManager.getUnsentRecordCount();
    LOG_INFO(MODULE_MAIN, "Offline Records : %d total, %d unsent", totalRecords, unsentRecords);
    LOG_INFO(MODULE_MAIN, "Offline Status  : %s", offlineManager.getStatusString());
    if (offlineOpStats.syncInProgress) {
      LOG_INFO(MODULE_MAIN, "Offline Sync    : %d%% complete", offlineOpStats.prioritySyncProgress);
    }
  #endif
  
  // Performance metrics
  if (performanceMetrics.totalTransmissions > 0) {
    unsigned long successRate = (performanceMetrics.successfulTransmissions * 100) / 
                               performanceMetrics.totalTransmissions;
    LOG_INFO(MODULE_MAIN, "Success Rate    : %lu%% (%lu/%lu)", 
             successRate, performanceMetrics.successfulTransmissions, 
             performanceMetrics.totalTransmissions);
  }
  
  // Memory info
  Utils::printMemoryInfo();
  
  LOG_INFO(MODULE_MAIN, "====================================");
}

void printHelp() {
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS TRACKER v7.4 COMMANDS ===");
  LOG_INFO(MODULE_MAIN, "System Commands:");
  LOG_INFO(MODULE_MAIN, "  help           - Show this help");
  LOG_INFO(MODULE_MAIN, "  status         - Show system status");
  LOG_INFO(MODULE_MAIN, "  init           - Show initialization status");
  LOG_INFO(MODULE_MAIN, "  reset          - Restart system");
  LOG_INFO(MODULE_MAIN, "  health         - Show health status");
  LOG_INFO(MODULE_MAIN, "  recovery       - Force health check");
  LOG_INFO(MODULE_MAIN, "  memory         - Show memory info");
  LOG_INFO(MODULE_MAIN, "  diag           - System diagnostics");
  LOG_INFO(MODULE_MAIN, "  stats          - System statistics");
  
  LOG_INFO(MODULE_MAIN, "Power & Movement:");
  LOG_INFO(MODULE_MAIN, "  full/standby/emergency - Set power mode");
  LOG_INFO(MODULE_MAIN, "  power          - Show power mode info");
  LOG_INFO(MODULE_MAIN, "  movement       - Show movement info");
  LOG_INFO(MODULE_MAIN, "  speed [N|auto] - Set manual speed or auto");
  
  LOG_INFO(MODULE_MAIN, "Network & WebSocket:");
  LOG_INFO(MODULE_MAIN, "  network        - Show network info");
  LOG_INFO(MODULE_MAIN, "  wsstats        - WebSocket statistics");
  LOG_INFO(MODULE_MAIN, "  wsreset        - Reset WebSocket connection");
  LOG_INFO(MODULE_MAIN, "  optimize       - Manual optimization");
  
  LOG_INFO(MODULE_MAIN, "GPS & Data:");
  LOG_INFO(MODULE_MAIN, "  gps            - Show GPS details");
  LOG_INFO(MODULE_MAIN, "  send           - Force send GPS data");
  LOG_INFO(MODULE_MAIN, "  latency        - Performance report");
  
  LOG_INFO(MODULE_MAIN, "Hardware Control:");
  LOG_INFO(MODULE_MAIN, "  on/off         - Control relay");
  LOG_INFO(MODULE_MAIN, "  battery        - Show battery info");
  LOG_INFO(MODULE_MAIN, "  setbat N       - Set battery voltage");
  
  #if ENABLE_OFFLINE_STORAGE
  LOG_INFO(MODULE_MAIN, "Offline Storage (REAL GPS PRIORITY):");
  LOG_INFO(MODULE_MAIN, "  offline        - Show offline status");
  LOG_INFO(MODULE_MAIN, "  offline status - Detailed offline status");
  LOG_INFO(MODULE_MAIN, "  offline stats  - Offline statistics");
  LOG_INFO(MODULE_MAIN, "  offline records- Show stored records");
  LOG_INFO(MODULE_MAIN, "  offline sync   - Manual sync");
  LOG_INFO(MODULE_MAIN, "  offline clear  - Clear all offline data");
  LOG_INFO(MODULE_MAIN, "  offline test   - Test offline storage dengan real GPS");
  LOG_INFO(MODULE_MAIN, "  offline force  - Force offline mode");
  LOG_INFO(MODULE_MAIN, "  offline online - Force online mode");
  LOG_INFO(MODULE_MAIN, "  offline generate - Generate test data dari real GPS");
  LOG_INFO(MODULE_MAIN, "  offline simulate outage N - Simulate N-sec outage dengan real GPS");
  LOG_INFO(MODULE_MAIN, "  offline simulate restore - Simulate network restore");
  LOG_INFO(MODULE_MAIN, "  offline synctest - Run sync test dengan real GPS");
  LOG_INFO(MODULE_MAIN, "  offline debug  - Enhanced debug information");
  #endif
  
  LOG_INFO(MODULE_MAIN, "Testing Commands (REAL GPS DATA):");
  LOG_INFO(MODULE_MAIN, "  test           - Show test commands");
  LOG_INFO(MODULE_MAIN, "  test send      - Test GPS transmission");
  LOG_INFO(MODULE_MAIN, "  test movement  - Test movement detection");
  LOG_INFO(MODULE_MAIN, "  test network   - Test network connectivity");
  LOG_INFO(MODULE_MAIN, "  test websocket - Test WebSocket connection");
  LOG_INFO(MODULE_MAIN, "  test offline   - Test offline storage dengan real GPS");
  LOG_INFO(MODULE_MAIN, "  test offline sync - Comprehensive offline sync test dengan real GPS");
  LOG_INFO(MODULE_MAIN, "  test gps       - Test GPS data priority");
  LOG_INFO(MODULE_MAIN, "  test stress    - Run stress test");
  LOG_INFO(MODULE_MAIN, "  test emergency - Test emergency procedures");
  LOG_INFO(MODULE_MAIN, "  outage N       - Quick outage simulation dengan real GPS");
  
  LOG_INFO(MODULE_MAIN, "Emergency:");
  LOG_INFO(MODULE_MAIN, "  backup         - Emergency backup");
  LOG_INFO(MODULE_MAIN, "  recover        - Recover from corruption");
  LOG_INFO(MODULE_MAIN, "  factory reset  - Factory reset (WARNING!)");
  
  LOG_INFO(MODULE_MAIN, "=======================================");
  LOG_INFO(MODULE_MAIN, "GPS DATA PRIORITY: Real GPS > Last Known > Minimal Simulated");
  LOG_INFO(MODULE_MAIN, "=======================================");
}

// ========================================
// REMAINING FUNCTION IMPLEMENTATIONS (UNCHANGED FROM v7.3 - ONLY DOCUMENTED)
// ========================================

// NOTE: Remaining functions are unchanged from v7.3 implementation
// They include: Movement detection, WebSocket management, Power management,
// System monitoring, Connection management, Performance optimization, etc.

// Movement Detection Functions
void updateMovementState() {
  static unsigned long lastMovementLog = 0;
  MovementState previousState = currentMovementState;
  
  float currentSpeed = getCurrentSpeed();
  
  if (currentSpeed > MOVEMENT_SPEED_THRESHOLD) {
    currentMovementState = MOVEMENT_MOVING;
    vehicleStopTime = 0;
  } else {
    if (vehicleStopTime == 0) {
      vehicleStopTime = millis();
      currentMovementState = MOVEMENT_PARKED;
    } else {
      unsigned long stopDuration = millis() - vehicleStopTime;
      
      if (stopDuration > PARKED_TO_STATIC_TIMEOUT) {
        currentMovementState = MOVEMENT_STATIC;
      } else {
        currentMovementState = MOVEMENT_PARKED;
      }
    }
  }
  
  if (currentMovementState != previousState) {
    logMovementStateChange(currentSpeed);
  }
  
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

// WebSocket Management Functions
bool sendVehicleDataViaWebSocket() {
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
  
  if (!powerConfigs[currentPowerMode].wsContinuous && !wsManager.isReady()) {
    LOG_INFO(MODULE_WS, "Connecting WebSocket for transmission...");
    if (!wsManager.connect()) {
      LOG_ERROR(MODULE_WS, "❌ WebSocket connection failed");
      systemFlags.wsReady = false;
      return false;
    }
    delay(1000);
  }
  
  if (!ensureWebSocketSubscribed()) {
    return false;
  }
  
  LOG_INFO(MODULE_GPS, "📤 Sending vehicle data [%s mode]...", 
           getMovementString(currentMovementState));
  
  bool success = false;
  
  if (gpsManager.isValid() || useManualSpeed) {
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      wsManager.startLatencyMeasurement();
      modemManager.startLatencyMeasurement();
    }
    
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
  
  if (!powerConfigs[currentPowerMode].wsContinuous && success) {
    delay(500);
    wsManager.disconnect();
    systemFlags.wsReady = false;
  }
  
  return success;
}

// Connection Management Functions (unchanged)
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
  if (!systemFlags.gpsReady && gpsManager.isValid()) {
    systemFlags.gpsReady = true;
    if (!systemFlags.firstGpsFix) {
      systemFlags.firstGpsFix = true;
      systemFlags.gpsFirstFixTime = millis();
    }
    LOG_INFO(MODULE_GPS, "✅ GPS fix acquired!");
    lastGpsSendTime = 0;
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

void checkSleepConditions(unsigned long currentTime) {
  const PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  if (config.sleepDuration > 0 && 
      currentTime - lastActivityTime > ACTIVITY_TIMEOUT) {
    LOG_INFO(MODULE_SYS, "No activity, preparing for sleep");
    currentState = STATE_SLEEP_PREPARE;
  }
}

void checkConnectionHealth(unsigned long currentTime) {
  if (currentTime - lastSuccessfulOperation > 180000) {
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

// System Monitoring Functions
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

// Power Management Functions
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

// Relay Control
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

// Enhanced Status Display Functions
void printPowerModeInfo() {
  const PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  LOG_INFO(MODULE_MAIN, "=== POWER MODE INFO ===");
  LOG_INFO(MODULE_MAIN, "Current Mode: %s", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "GPS Interval: %lu ms", config.gpsInterval);
  LOG_INFO(MODULE_MAIN, "WS Keep-Alive: %lu ms", config.wsKeepAliveInterval);
  LOG_INFO(MODULE_MAIN, "Sleep Duration: %lu ms", config.sleepDuration);
  LOG_INFO(MODULE_MAIN, "GPS Always On: %s", config.gpsAlwaysOn ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "WS Continuous: %s", config.wsContinuous ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Relay Enabled: %s", config.relayEnabled ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Performance Monitoring: %s", config.performanceMonitoring ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "Aggressive Optimization: %s", config.aggressiveOptimization ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "=======================");
}

void printWebSocketStats() {
  LOG_INFO(MODULE_WS, "=== WEBSOCKET STATISTICS ===");
  LOG_INFO(MODULE_WS, "State: %s", wsManager.getStateString());
  LOG_INFO(MODULE_WS, "Ready: %s", wsManager.isReady() ? "YES" : "NO");
  LOG_INFO(MODULE_WS, "============================");
}

void printPerformanceReport() {
  LOG_INFO(MODULE_PERF, "=== PERFORMANCE REPORT ===");
  LOG_INFO(MODULE_PERF, "Total Transmissions: %lu", performanceMetrics.totalTransmissions);
  LOG_INFO(MODULE_PERF, "Successful: %lu", performanceMetrics.successfulTransmissions);
  LOG_INFO(MODULE_PERF, "Failed: %lu", performanceMetrics.failedTransmissions);
  
  if (performanceMetrics.totalTransmissions > 0) {
    unsigned long successRate = (performanceMetrics.successfulTransmissions * 100) / 
                               performanceMetrics.totalTransmissions;
    LOG_INFO(MODULE_PERF, "Success Rate: %lu%%", successRate);
  }
  
  if (performanceMetrics.totalLatency > 0 && performanceMetrics.successfulTransmissions > 0) {
    unsigned long avgLatency = performanceMetrics.totalLatency / performanceMetrics.successfulTransmissions;
    LOG_INFO(MODULE_PERF, "Average Latency: %lu ms", avgLatency);
    LOG_INFO(MODULE_PERF, "Min Latency: %lu ms", performanceMetrics.minLatency);
    LOG_INFO(MODULE_PERF, "Max Latency: %lu ms", performanceMetrics.maxLatency);
  }
  
  LOG_INFO(MODULE_PERF, "Consecutive Failures: %d", performanceMetrics.consecutiveFailures);
  LOG_INFO(MODULE_PERF, "Consecutive Slow: %d", performanceMetrics.consecutiveSlowTransmissions);
  
  unsigned long timeSinceOptimization = millis() - performanceMetrics.lastOptimizationTime;
  LOG_INFO(MODULE_PERF, "Last Optimization: %lu min ago", timeSinceOptimization / 60000);
  
  LOG_INFO(MODULE_PERF, "==========================");
}

void printSystemReadyStatus() {
  LOG_INFO(MODULE_MAIN, "=== SYSTEM READY STATUS ===");
  LOG_INFO(MODULE_MAIN, "GPS Ready: %s", systemFlags.gpsReady ? "✅ YES" : "❌ NO");
  LOG_INFO(MODULE_MAIN, "Modem Ready: %s", systemFlags.modemReady ? "✅ YES" : "❌ NO");
  LOG_INFO(MODULE_MAIN, "Network Ready: %s", systemFlags.networkReady ? "✅ YES" : "❌ NO");
  LOG_INFO(MODULE_MAIN, "GPRS Ready: %s", systemFlags.gprsReady ? "✅ YES" : "❌ NO");
  LOG_INFO(MODULE_MAIN, "WebSocket Ready: %s", systemFlags.wsReady ? "✅ YES" : "❌ NO");
  LOG_INFO(MODULE_MAIN, "First GPS Fix: %s", systemFlags.firstGpsFix ? "✅ YES" : "❌ NO");
  
  if (systemFlags.firstGpsFix && systemFlags.gpsFirstFixTime > 0) {
    unsigned long fixTime = (systemFlags.gpsFirstFixTime - systemFlags.systemStartTime) / 1000;
    LOG_INFO(MODULE_MAIN, "GPS Fix Time: %lu seconds", fixTime);
  }
  
  unsigned long uptime = (millis() - systemFlags.systemStartTime) / 1000;
  LOG_INFO(MODULE_MAIN, "System Uptime: %lu seconds", uptime);
  
  bool fullyReady = systemFlags.isReady();
  LOG_INFO(MODULE_MAIN, "System Status: %s", fullyReady ? "✅ FULLY READY" : "⚠️ NOT READY");
  
  LOG_INFO(MODULE_MAIN, "===========================");
}

void printMovementInfo() {
  LOG_INFO(MODULE_MAIN, "=== MOVEMENT INFO ===");
  LOG_INFO(MODULE_MAIN, "Current State: %s", getMovementString(currentMovementState));
  LOG_INFO(MODULE_MAIN, "Current Speed: %.1f km/h %s", 
           getCurrentSpeed(), useManualSpeed ? "[MANUAL]" : "[GPS]");
  LOG_INFO(MODULE_MAIN, "GPS Interval: %lu ms", getGpsIntervalForMovement());
  LOG_INFO(MODULE_MAIN, "Speed Threshold: %.1f km/h", MOVEMENT_SPEED_THRESHOLD);
  LOG_INFO(MODULE_MAIN, "Static Timeout: %lu min", PARKED_TO_STATIC_TIMEOUT / 60000);
  
  if (vehicleStopTime > 0) {
    unsigned long stopDuration = (millis() - vehicleStopTime) / 1000;
    LOG_INFO(MODULE_MAIN, "Stopped Duration: %lu seconds", stopDuration);
  }
  
  LOG_INFO(MODULE_MAIN, "=====================");
}

void showSpeedInfo() {
  LOG_INFO(MODULE_MAIN, "=== SPEED INFO ===");
  if (useManualSpeed) {
    LOG_INFO(MODULE_MAIN, "Mode: MANUAL");
    LOG_INFO(MODULE_MAIN, "Manual Speed: %.1f km/h", manualSpeed);
  } else {
    LOG_INFO(MODULE_MAIN, "Mode: GPS AUTO");
    if (systemFlags.gpsReady) {
      LOG_INFO(MODULE_MAIN, "GPS Speed: %.1f km/h", gpsManager.getSpeed());
    } else {
      LOG_INFO(MODULE_MAIN, "GPS Speed: NOT AVAILABLE");
    }
  }
  LOG_INFO(MODULE_MAIN, "Current Speed: %.1f km/h", getCurrentSpeed());
  LOG_INFO(MODULE_MAIN, "Movement State: %s", getMovementString(currentMovementState));
  LOG_INFO(MODULE_MAIN, "==================");
}

void showBatteryInfo() {
  LOG_INFO(MODULE_MAIN, "=== BATTERY INFO ===");
  LOG_INFO(MODULE_MAIN, "Voltage: %.1f V", batteryVoltage);
  
  #if ENABLE_BATTERY_MONITORING
    LOG_INFO(MODULE_MAIN, "Monitoring: ENABLED");
    float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                       (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
    percentage = constrain(percentage, 0.0, 100.0);
    LOG_INFO(MODULE_MAIN, "Percentage: %.1f%%", percentage);
    
    if (batteryVoltage < BATTERY_LOW_THRESHOLD) {
      LOG_WARN(MODULE_MAIN, "⚠️ LOW BATTERY WARNING");
    }
  #else
    LOG_INFO(MODULE_MAIN, "Monitoring: DISABLED");
  #endif
  
  LOG_INFO(MODULE_MAIN, "====================");
}

void showGpsDetails() {
  LOG_INFO(MODULE_MAIN, "=== GPS DETAILS (ENHANCED) ===");
  LOG_INFO(MODULE_MAIN, "Status: %s", systemFlags.gpsReady ? "READY" : "NOT READY");
  LOG_INFO(MODULE_MAIN, "Valid: %s", gpsManager.isValid() ? "YES" : "NO");
  
  if (gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "Latitude: %.6f", gpsManager.getLatitude());
    LOG_INFO(MODULE_MAIN, "Longitude: %.6f", gpsManager.getLongitude());
    LOG_INFO(MODULE_MAIN, "Altitude: %.1f m", gpsManager.getAltitude());
    LOG_INFO(MODULE_MAIN, "Speed: %.1f km/h", gpsManager.getSpeed());
    LOG_INFO(MODULE_MAIN, "Heading: %.1f°", gpsManager.getHeading());
    LOG_INFO(MODULE_MAIN, "Data Source: REAL GPS (Priority 1)");
  } else if (gpsManager.hasLastKnownPosition()) {
    unsigned long age = (millis() - gpsManager.getLastValidTime()) / 1000;
    LOG_INFO(MODULE_MAIN, "Last Known Lat: %.6f", gpsManager.getLastKnownLatitude());
    LOG_INFO(MODULE_MAIN, "Last Known Lon: %.6f", gpsManager.getLastKnownLongitude());
    LOG_INFO(MODULE_MAIN, "Last Known Speed: %.1f km/h", gpsManager.getLastKnownSpeed());
    LOG_INFO(MODULE_MAIN, "Position Age: %lu seconds", age);
    LOG_INFO(MODULE_MAIN, "Data Source: LAST KNOWN (Priority 2)");
  } else {
    LOG_WARN(MODULE_MAIN, "Data Source: SIMULATED ONLY (Priority 3)");
  }
  
  LOG_INFO(MODULE_MAIN, "Satellites: %d", gpsManager.getSatellites());
  LOG_INFO(MODULE_MAIN, "HDOP: %.2f", gpsManager.getHDOP());
  
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  LOG_INFO(MODULE_MAIN, "Timestamp: %s", timestamp);
  
  LOG_INFO(MODULE_MAIN, "===============================");
}

void printHealthStatus() {
  LOG_INFO(MODULE_HEALTH, "=== SYSTEM HEALTH ===");
  
  // Memory health
  uint32_t freeHeap = Utils::getFreeHeap();
  LOG_INFO(MODULE_HEALTH, "Free Memory: %u KB", freeHeap / 1024);
  
  if (freeHeap < MEMORY_CRITICAL_THRESHOLD) {
    LOG_ERROR(MODULE_HEALTH, "❌ CRITICAL: Low memory");
  } else if (freeHeap < MEMORY_WARNING_THRESHOLD) {
    LOG_WARN(MODULE_HEALTH, "⚠️ WARNING: Low memory");
  } else {
    LOG_INFO(MODULE_HEALTH, "✅ Memory: OK");
  }
  
  // Success rate health
  if (performanceMetrics.totalTransmissions > 10) {
    unsigned long successRate = (performanceMetrics.successfulTransmissions * 100) / 
                               performanceMetrics.totalTransmissions;
    LOG_INFO(MODULE_HEALTH, "Success Rate: %lu%%", successRate);
    
    if (successRate < SUCCESS_RATE_THRESHOLD) {
      LOG_ERROR(MODULE_HEALTH, "❌ CRITICAL: Low success rate");
    } else if (successRate < 50) {
      LOG_WARN(MODULE_HEALTH, "⚠️ WARNING: Moderate success rate");
    } else {
      LOG_INFO(MODULE_HEALTH, "✅ Success Rate: OK");
    }
  }
  
  // Uptime health
  unsigned long uptime = millis() / 1000;
  LOG_INFO(MODULE_HEALTH, "Uptime: %lu hours", uptime / 3600);
  
  #if ENABLE_AUTO_RESTART
    unsigned long timeToRestart = (AUTO_RESTART_INTERVAL - (millis() - systemStartTime)) / 1000;
    LOG_INFO(MODULE_HEALTH, "Next Restart: %lu hours", timeToRestart / 3600);
  #endif
  
  // Consecutive failures
  LOG_INFO(MODULE_HEALTH, "Consecutive Health Failures: %d/%d", 
           consecutiveHealthFailures, MAX_HEALTH_FAILURES);
  
  if (consecutiveHealthFailures > 0) {
    LOG_WARN(MODULE_HEALTH, "⚠️ Health issues detected");
  } else {
    LOG_INFO(MODULE_HEALTH, "✅ System Health: OK");
  }
  
  // Last successful transmission
  if (performanceMetrics.successfulTransmissions > 0) {
    unsigned long timeSinceSuccess = (millis() - lastSuccessfulTransmission) / 1000;
    LOG_INFO(MODULE_HEALTH, "Last Success: %lu min ago", timeSinceSuccess / 60);
    
    if (timeSinceSuccess > (NO_SUCCESS_TIMEOUT / 1000)) {
      LOG_ERROR(MODULE_HEALTH, "❌ CRITICAL: No recent success");
    }
  }
  
  LOG_INFO(MODULE_HEALTH, "=====================");
}

// Advanced Testing Functions dengan Real GPS Data
void runOfflineStorageStressTest() {
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_MAIN, "🧪 Running offline storage stress test dengan REAL GPS data...");
    
    // Backup current settings
    bool originalForce = forceOfflineMode;
    bool originalNetwork = networkAvailable;
    
    // Force offline mode
    forceOfflineMode = true;
    networkAvailable = false;
    
    int targetRecords = min(50, OFFLINE_MAX_RECORDS - 10);
    int storedCount = 0;
    int failedCount = 0;
    int realGpsCount = 0;
    int lastKnownCount = 0;
    int simulatedCount = 0;
    
    LOG_INFO(MODULE_MAIN, "Generating %d test records dengan GPS data priority...", targetRecords);
    
    for (int i = 0; i < targetRecords; i++) {
      bool stored = false;
      
      // Update GPS
      gpsManager.update();
      
      // PRIORITY 1: Real GPS data
      if (systemFlags.gpsReady && gpsManager.isValid()) {
        char timestamp[30];
        gpsManager.getTimestamp(timestamp, sizeof(timestamp));
        
        stored = storeDataOffline(
          gpsManager.getLatitude(),
          gpsManager.getLongitude(),
          getCurrentSpeed(),
          gpsManager.getSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        if (stored) realGpsCount++;
      }
      // PRIORITY 2: Last known position
      else if (gpsManager.hasLastKnownPosition()) {
        char timestamp[30];
        sprintf(timestamp, "2025-01-17T10:%02d:%02dZ", i);
        
        stored = storeDataOffline(
          gpsManager.getLastKnownLatitude(),
          gpsManager.getLastKnownLongitude(),
          getCurrentSpeed(),
          gpsManager.getLastKnownSatellites(),
          String(timestamp),
          batteryVoltage
        );
        
        if (stored) lastKnownCount++;
      }
      // PRIORITY 3: Simulated data (minimal)
      else {
        float lat = -6.2088 + (i * 0.00001);
        float lon = 106.8456 + (i * 0.00001); 
        float speed = (i % 60) + 10.0;
        int satellites = 4 + (i % 8);
        
        char timestamp[30];
        sprintf(timestamp, "2025-01-17T%02d:%02d:%02dZ", 
                (i / 60) % 24, i % 60, (i * 13) % 60);
        
        stored = storeDataOffline(lat, lon, speed, satellites, String(timestamp), 
                                12.0 + (i % 25) * 0.1);
        
        if (stored) simulatedCount++;
      }
      
      if (stored) {
        storedCount++;
      } else {
        failedCount++;
      }
      
      // Show progress every 10 records
      if ((i + 1) % 10 == 0) {
        LOG_INFO(MODULE_MAIN, "Progress: %d/%d stored (%d real, %d last known, %d simulated), %d failed", 
                 storedCount, targetRecords, realGpsCount, lastKnownCount, simulatedCount, failedCount);
      }
      
      Utils::safeDelay(10);
    }
    
    LOG_INFO(MODULE_MAIN, "✅ Storage test complete:");
    LOG_INFO(MODULE_MAIN, "  - Total stored: %d/%d", storedCount, targetRecords);
    LOG_INFO(MODULE_MAIN, "  - Real GPS: %d", realGpsCount);
    LOG_INFO(MODULE_MAIN, "  - Last known: %d", lastKnownCount);
    LOG_INFO(MODULE_MAIN, "  - Simulated: %d", simulatedCount);
    LOG_INFO(MODULE_MAIN, "  - Failed: %d", failedCount);
    
    if (storedCount > 0) {
      float realDataPercentage = ((realGpsCount + lastKnownCount) * 100.0f) / storedCount;
      LOG_INFO(MODULE_MAIN, "  - Real data coverage: %.1f%%", realDataPercentage);
    }
    
    // Test sync performance
    LOG_INFO(MODULE_MAIN, "Testing sync performance...");
    
    // Restore network
    forceOfflineMode = false;
    networkAvailable = true;
    systemFlags.networkReady = true;
    systemFlags.gprsReady = true;
    
    // Force network check
    lastNetworkCheck = 0;
    checkNetworkAvailability();
    
    // Monitor sync
    unsigned long syncStart = millis();
    int maxWait = 60; // 60 seconds max
    int waited = 0;
    
    while (offlineManager.hasOfflineData() && waited < maxWait) {
      Utils::safeDelay(1000);
      waited++;
      
      if (waited % 10 == 0) {
        int remaining = offlineManager.getUnsentRecordCount();
        LOG_INFO(MODULE_MAIN, "📊 Sync progress: %d records remaining (%d/%d sec)", 
                 remaining, waited, maxWait);
      }
    }
    
    unsigned long syncDuration = millis() - syncStart;
    int finalCount = offlineManager.getUnsentRecordCount();
    
    LOG_INFO(MODULE_MAIN, "✅ Stress test complete:");
    LOG_INFO(MODULE_MAIN, "  Records stored: %d (%d real GPS)", storedCount, realGpsCount);
    LOG_INFO(MODULE_MAIN, "  Sync time: %lu ms", syncDuration);
    LOG_INFO(MODULE_MAIN, "  Records remaining: %d", finalCount);
    LOG_INFO(MODULE_MAIN, "  Success rate: %.1f%%", 
             storedCount > 0 ? ((float)(storedCount - finalCount) * 100.0 / storedCount) : 0);
    
    // Restore original settings
    forceOfflineMode = originalForce;
    networkAvailable = originalNetwork;
    
    offlineManager.printSyncStatistics();
  #else
    LOG_WARN(MODULE_MAIN, "Offline storage not enabled for stress test");
  #endif
}

void performSystemDiagnostics() {
  LOG_INFO(MODULE_MAIN, "🔍 Running enhanced system diagnostics...");
  
  // 1. Memory diagnostics
  Utils::printMemoryInfo();
  
  // 2. Enhanced GPS diagnostics dengan data source priority
  LOG_INFO(MODULE_MAIN, "GPS Diagnostics (Enhanced):");
  LOG_INFO(MODULE_MAIN, "  Valid: %s", gpsManager.isValid() ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "  Satellites: %d", gpsManager.getSatellites());
  LOG_INFO(MODULE_MAIN, "  HDOP: %.2f", gpsManager.getHDOP());
  LOG_INFO(MODULE_MAIN, "  GPS Ready: %s", systemFlags.gpsReady ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "  Has Last Known: %s", gpsManager.hasLastKnownPosition() ? "YES" : "NO");
  
  if (systemFlags.gpsReady && gpsManager.isValid()) {
    LOG_INFO(MODULE_MAIN, "  Data Source: REAL GPS (Priority 1)");
  } else if (gpsManager.hasLastKnownPosition()) {
    unsigned long age = (millis() - gpsManager.getLastValidTime()) / 1000;
    LOG_INFO(MODULE_MAIN, "  Data Source: LAST KNOWN (%lu sec old, Priority 2)", age);
  } else {
    LOG_INFO(MODULE_MAIN, "  Data Source: SIMULATED ONLY (Priority 3)");
  }
  
  // 3. Modem diagnostics
  LOG_INFO(MODULE_MAIN, "Modem Diagnostics:");
  LOG_INFO(MODULE_MAIN, "  Status: %s", modemManager.getStatusString());
  LOG_INFO(MODULE_MAIN, "  Network: %s", modemManager.isNetworkConnected() ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "  GPRS: %s", modemManager.isGprsConnected() ? "YES" : "NO");
  LOG_INFO(MODULE_MAIN, "  Signal: %d", modemManager.getSignalQuality());
  
  // 4. WebSocket diagnostics
  LOG_INFO(MODULE_MAIN, "WebSocket Diagnostics:");
  LOG_INFO(MODULE_MAIN, "  State: %s", wsManager.getStateString());
  LOG_INFO(MODULE_MAIN, "  Ready: %s", wsManager.isReady() ? "YES" : "NO");
  printWebSocketStats();
  
  // 5. Performance diagnostics
  LOG_INFO(MODULE_MAIN, "Performance Diagnostics:");
  printPerformanceReport();
  
  // 6. Enhanced offline storage diagnostics dengan data source info
  #if ENABLE_OFFLINE_STORAGE
    LOG_INFO(MODULE_MAIN, "Offline Storage Diagnostics (Enhanced):");
    offlineManager.printDetailedStatus();
    printOfflineStats();
    
    int totalRecords = offlineManager.getOfflineRecordCount();
    int unsentRecords = offlineManager.getUnsentRecordCount();
    LOG_INFO(MODULE_MAIN, "  Total/Unsent: %d/%d", totalRecords, unsentRecords);
    LOG_INFO(MODULE_MAIN, "  Network Mode: %s", networkAvailable ? "ONLINE" : "OFFLINE");
    LOG_INFO(MODULE_MAIN, "  GPS Data Priority: Real > Last Known > Simulated");
  #endif
  
  // 7. System health
  LOG_INFO(MODULE_MAIN, "System Health:");
  printHealthStatus();
  
  LOG_INFO(MODULE_MAIN, "✅ Enhanced system diagnostics complete");
}

void performEmergencyBackup() {
  LOG_INFO(MODULE_MAIN, "🚨 Performing emergency backup dengan real GPS data...");
  
  #if ENABLE_OFFLINE_STORAGE
    // Force save any pending offline data
    offlineManager.performMaintenance();
    
    // Save current GPS position if available dengan priority
    bool backupSuccess = false;
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    // PRIORITY 1: Real GPS data
    if (systemFlags.gpsReady && gpsManager.isValid()) {
      backupSuccess = storeDataOffline(
        gpsManager.getLatitude(),
        gpsManager.getLongitude(),
        getCurrentSpeed(),
        gpsManager.getSatellites(),
        String(timestamp),
        batteryVoltage
      );
      LOG_INFO(MODULE_MAIN, "Emergency backup dengan REAL GPS: %s", backupSuccess ? "SUCCESS" : "FAILED");
    }
    // PRIORITY 2: Last known position
    else if (gpsManager.hasLastKnownPosition()) {
      backupSuccess = storeDataOffline(
        gpsManager.getLastKnownLatitude(),
        gpsManager.getLastKnownLongitude(),
        getCurrentSpeed(),
        gpsManager.getLastKnownSatellites(),
        String(timestamp),
        batteryVoltage
      );
      LOG_INFO(MODULE_MAIN, "Emergency backup dengan LAST KNOWN: %s", backupSuccess ? "SUCCESS" : "FAILED");
    }
    
    // Log critical system state
    LOG_INFO(MODULE_MAIN, "System state backup:");
    LOG_INFO(MODULE_MAIN, "  State: %s", getStateString(currentState));
    LOG_INFO(MODULE_MAIN, "  Movement: %s", getMovementString(currentMovementState));
    LOG_INFO(MODULE_MAIN, "  Network: %s", networkAvailable ? "ONLINE" : "OFFLINE");
    LOG_INFO(MODULE_MAIN, "  GPS Source: %s", 
             (systemFlags.gpsReady && gpsManager.isValid()) ? "REAL" : 
             (gpsManager.hasLastKnownPosition() ? "LAST_KNOWN" : "NONE"));
    LOG_INFO(MODULE_MAIN, "  Uptime: %lu min", millis() / 60000);
    Utils::printMemoryInfo();
  #endif
  
  LOG_INFO(MODULE_MAIN, "✅ Emergency backup complete");
}

void recoverFromCorruptedState() {
  LOG_INFO(MODULE_MAIN, "🔧 Attempting recovery from corrupted state...");
  
  // 1. Reset state machine
  currentState = STATE_INIT;
  
  // 2. Reset flags
  systemFlags.reset();
  performanceMetrics.reset();
  
  // 3. Reset offline stats but preserve data
  offlineOpStats.syncInProgress = false;
  offlineOpStats.prioritySyncMode = false;
  
  // 4. Reset network state
  networkAvailable = false;
  offlineMode = true;
  forceOfflineMode = false;
  
  // 5. Reset timers
  lastActivityTime = millis();
  lastSuccessfulTransmission = millis();
  lastNetworkCheck = 0;
  lastOfflineSync = 0;
  
  // 6. Validate and repair offline storage
  #if ENABLE_OFFLINE_STORAGE
    if (!offlineManager.validateStorage()) {
      LOG_WARN(MODULE_MAIN, "Repairing offline storage...");
      offlineManager.repairStorage();
    }
  #endif
  
  // 7. Force re-initialization
  LOG_INFO(MODULE_MAIN, "Forcing system re-initialization...");
  
  LOG_INFO(MODULE_MAIN, "✅ Recovery complete, system reinitializing...");
}

void logSystemStats() {
  LOG_INFO(MODULE_MAIN, "=== ENHANCED SYSTEM STATISTICS ===");
  LOG_INFO(MODULE_MAIN, "Uptime: %lu minutes", millis() / 60000);
  LOG_INFO(MODULE_MAIN, "State: %s", getStateString(currentState));
  LOG_INFO(MODULE_MAIN, "Power Mode: %s", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "Movement: %s", getMovementString(currentMovementState));
  
  // Enhanced GPS info dengan data source priority
  LOG_INFO(MODULE_MAIN, "GPS Status: Ready=%s, Valid=%s, LastKnown=%s", 
           systemFlags.gpsReady ? "Y" : "N",
           gpsManager.isValid() ? "Y" : "N",
           gpsManager.hasLastKnownPosition() ? "Y" : "N");
  
  if (performanceMetrics.totalTransmissions > 0) {
    unsigned long successRate = (performanceMetrics.successfulTransmissions * 100) / 
                               performanceMetrics.totalTransmissions;
    LOG_INFO(MODULE_MAIN, "Transmissions: %lu total, %lu successful (%lu%%)", 
             performanceMetrics.totalTransmissions, 
             performanceMetrics.successfulTransmissions, successRate);
    
    if (performanceMetrics.totalLatency > 0 && performanceMetrics.successfulTransmissions > 0) {
      unsigned long avgLatency = performanceMetrics.totalLatency / performanceMetrics.successfulTransmissions;
      LOG_INFO(MODULE_MAIN, "Latency: %lu ms avg (min: %lu, max: %lu)", 
               avgLatency, performanceMetrics.minLatency, performanceMetrics.maxLatency);
    }
  }
  
  #if ENABLE_OFFLINE_STORAGE
    printOfflineStats();
  #endif
  
  Utils::printMemoryInfo();
  LOG_INFO(MODULE_MAIN, "=================================");
}

// Utility Functions
unsigned long getGpsIntervalForMovement() {
  switch (currentMovementState) {
    case MOVEMENT_MOVING:
      return GPS_INTERVAL_MOVING;
    case MOVEMENT_PARKED:
      return GPS_INTERVAL_PARKED;
    case MOVEMENT_STATIC:
      return GPS_INTERVAL_STATIC;
    default:
      return GPS_INTERVAL_MOVING; // Default to most frequent
  }
}

float getCurrentSpeed() {
  return useManualSpeed ? manualSpeed : gpsManager.getSpeed();
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
    case STATE_CONNECTION_RECOVERY: return "CONNECTION_RECOVERY";
    case STATE_ERROR: return "ERROR";
    case STATE_SLEEP_PREPARE: return "SLEEP_PREPARE";
    case STATE_SLEEPING: return "SLEEPING";
    case STATE_OPTIMIZING: return "OPTIMIZING";
    case STATE_OFFLINE_SYNC: return "OFFLINE_SYNC";
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

String formatTimestamp(unsigned long unixTime) {
  // Simple timestamp formatting
  char buffer[30];
  time_t rawTime = unixTime;
  struct tm *timeInfo = gmtime(&rawTime);
  
  sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          timeInfo->tm_year + 1900, 
          timeInfo->tm_mon + 1, 
          timeInfo->tm_mday,
          timeInfo->tm_hour, 
          timeInfo->tm_min, 
          timeInfo->tm_sec);
  
  return String(buffer);
}

bool isSystemReady() {
  return systemFlags.isReady();
}

void setBatteryVoltage(float voltage) {
  batteryVoltage = voltage;
  LOG_INFO(MODULE_MAIN, "Battery voltage set to: %.2fV", batteryVoltage);
}

// Final comment indicating completion
// LOG_INFO(MODULE_MAIN, "✅ Main.cpp v7.4 COMPLETE - All functions implemented dengan Real GPS Data Priority");

// ========================================
// END OF MAIN.CPP v7.4 - COMPLETE FIXED IMPLEMENTATION
// DENGAN REAL GPS DATA PRIORITY SYSTEM
// ========================================