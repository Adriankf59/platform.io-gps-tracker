// ========================================
// Main.cpp - Updated for Database Compliance and Signal Monitoring
// ========================================

/**
 * ESP32 Vehicle GPS Tracking dengan Database Compliance
 * - Database-compliant payload format
 * - RSRQ/RSRP signal monitoring
 * - End-to-end latency calculation
 * - Enhanced performance monitoring
 * - Movement state detection (MOVING/PARKED/STATIC)
 * 
 * Versi: 8.0 - Database Compliance with Signal Monitoring
 * Update: Integrated signal monitoring and database format compliance
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
#include "SignalAnalysis.h"

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

// ========================================
// STRUKTUR DATA
// ========================================

// ----- ENHANCED PERFORMANCE METRICS -----
struct PerformanceMetrics {
  unsigned long totalTransmissions;
  unsigned long successfulTransmissions;
  unsigned long failedTransmissions;
  unsigned long totalLatency;
  unsigned long minLatency;
  unsigned long maxLatency;
  unsigned long totalEndToEndLatency;    // NEW: End-to-end latency
  unsigned long minEndToEndLatency;      // NEW: Min end-to-end latency
  unsigned long maxEndToEndLatency;      // NEW: Max end-to-end latency
  unsigned long endToEndSamples;         // NEW: Number of end-to-end samples
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
    totalEndToEndLatency = 0;
    minEndToEndLatency = UINT32_MAX;
    maxEndToEndLatency = 0;
    endToEndSamples = 0;
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
  bool signalMonitoring;        // NEW: Signal monitoring flag
};

// Konfigurasi untuk setiap mode power (UPDATED)
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

// Control Variables
bool relayState = true;
bool waitForSubscription = true;
float batteryVoltage = 12.6;

// Testing Variables
float manualSpeed = -1.0;
bool useManualSpeed = false;

// Signal Monitoring Variables (NEW)
float lastRSRQ = RSRQ_INVALID_VALUE;
float lastRSRP = RSRP_INVALID_VALUE;
unsigned long lastSignalUpdate = 0;

// Latency Tracking Variables (NEW)
String lastTransmissionId = "";
unsigned long lastTransmissionStart = 0;
bool waitingForConfirmation = false;

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
void processSignalCommand(const String& cmd);  // NEW

// Status and Info Functions
void printStatus();
void printHelp();
void printPowerModeInfo();
void printWebSocketStats();
void printPerformanceReport();
void printSystemReadyStatus();
void printMovementInfo();
void printSignalInfo();           // NEW
void showSpeedInfo();
void showBatteryInfo();
void showGpsDetails();

// Data Transmission (UPDATED)
bool sendVehicleDataViaWebSocket();
void onRelayUpdate(bool newState);
void onDataConfirmation(const String& id, bool success, unsigned long latency);  // NEW
void forceSendGpsData();

// Power Management
void setPowerMode(PowerMode mode);
void enterLightSleep(unsigned long duration);
void enterDeepSleep(unsigned long duration);
void disableUnusedPeripherals();
void enablePeripherals();
void setRelay(bool state);

// System Monitoring (UPDATED)
float readBatteryVoltage();
void updateBatteryStatus();
void checkEmergencyMode();
void updateMovementState();
void checkSignalQuality();
void updateSignalMonitoring();    // NEW
void performanceOptimizationCheck();
void updatePerformanceMetrics(bool success, unsigned long latency, unsigned long endToEndLatency = 0);  // UPDATED
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

// Signal Quality Functions (NEW)
void logSignalMetrics();
String getSignalQualityString(float rsrq, float rsrp);
bool isSignalGoodForTransmission();

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
  LOG_INFO(MODULE_MAIN, "Database Compliance with Signal Monitoring");
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
  wsManager.setOnDataConfirmation(onDataConfirmation);  // NEW
  
  // Enable signal monitoring if supported
  if (ENABLE_SIGNAL_MONITORING) {
    modemManager.setSignalMonitoring(true);
    LOG_INFO(MODULE_SIGNAL, "Signal monitoring enabled (RSRQ/RSRP)");
  }
  
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
    updateSignalMonitoring();  // NEW
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
// NEW SIGNAL MONITORING FUNCTIONS
// ========================================

void updateSignalMonitoring() {
  if (!ENABLE_SIGNAL_MONITORING || !powerConfigs[currentPowerMode].signalMonitoring) {
    return;
  }
  
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < SIGNAL_MONITORING_INTERVAL) {
    return;
  }
  
  // Update signal metrics
  modemManager.updateSignalInfo();
  
  // Get current values
  float currentRSRQ = modemManager.getRSRQ();
  float currentRSRP = modemManager.getRSRP();
  
  // Check for significant changes
  bool rsrqChanged = abs(currentRSRQ - lastRSRQ) > 2.0;  // 2 dB threshold
  bool rsrpChanged = abs(currentRSRP - lastRSRP) > 5.0;  // 5 dBm threshold
  
  if (rsrqChanged || rsrpChanged) {
    LOG_INFO(MODULE_SIGNAL, "📶 Signal change: RSRQ=%.1f dB, RSRP=%.1f dBm", 
             currentRSRQ, currentRSRP);
    
    // Log quality assessment
    if (currentRSRQ != RSRQ_INVALID_VALUE && currentRSRP != RSRP_INVALID_VALUE) {
      String quality = getSignalQualityString(currentRSRQ, currentRSRP);
      LOG_INFO(MODULE_SIGNAL, "📊 Signal quality: %s", quality.c_str());
    }
  }
  
  // Update last known values
  lastRSRQ = currentRSRQ;
  lastRSRP = currentRSRP;
  lastUpdate = millis();
}

void logSignalMetrics() {
  if (!ENABLE_SIGNAL_MONITORING) {
    LOG_INFO(MODULE_SIGNAL, "Signal monitoring disabled");
    return;
  }
  
  const SignalInfo& signal = modemManager.getSignalInfo();
  
  LOG_INFO(MODULE_SIGNAL, "=== SIGNAL METRICS ===");
  LOG_INFO(MODULE_SIGNAL, "CSQ: %d (%s)", 
           signal.csq, 
           SignalAnalysis::getSignalQualityDescription(signal.csq).c_str());
  
  if (signal.rsrqValid) {
    LOG_INFO(MODULE_SIGNAL, "RSRQ: %.1f dB (%s)", 
             signal.rsrq,
             SignalAnalysis::getRSRQQualityDescription(signal.rsrq).c_str());
  } else {
    LOG_INFO(MODULE_SIGNAL, "RSRQ: INVALID");
  }
  
  if (signal.rsrpValid) {
    LOG_INFO(MODULE_SIGNAL, "RSRP: %.1f dBm (%s)", 
             signal.rsrp,
             SignalAnalysis::getRSRPQualityDescription(signal.rsrp).c_str());
  } else {
    LOG_INFO(MODULE_SIGNAL, "RSRP: INVALID");
  }
  
  if (signal.hasLteMetrics()) {
    float score = SignalAnalysis::calculateSignalScore(signal);
    LOG_INFO(MODULE_SIGNAL, "Signal Score: %.1f/100", score);
    
    if (isSignalGoodForTransmission()) {
      LOG_INFO(MODULE_SIGNAL, "Status: ✅ GOOD FOR TRANSMISSION");
    } else {
      LOG_INFO(MODULE_SIGNAL, "Status: ⚠️ POOR SIGNAL");
    }
  }
  
  unsigned long age = (millis() - signal.lastUpdate) / 1000;
  LOG_INFO(MODULE_SIGNAL, "Last Update: %lu seconds ago", age);
}

String getSignalQualityString(float rsrq, float rsrp) {
  String quality = "";
  
  if (rsrq != RSRQ_INVALID_VALUE) {
    if (rsrq > RSRQ_EXCELLENT_THRESHOLD) quality += "EXCELLENT";
    else if (rsrq > RSRQ_GOOD_THRESHOLD) quality += "GOOD";
    else if (rsrq > RSRQ_FAIR_THRESHOLD) quality += "FAIR";
    else quality += "POOR";
  }
  
  if (rsrp != RSRP_INVALID_VALUE) {
    if (!quality.isEmpty()) quality += "/";
    
    if (rsrp > RSRP_EXCELLENT_THRESHOLD) quality += "EXCELLENT";
    else if (rsrp > RSRP_GOOD_THRESHOLD) quality += "GOOD";
    else if (rsrp > RSRP_FAIR_THRESHOLD) quality += "FAIR";
    else quality += "POOR";
  }
  
  return quality.isEmpty() ? "UNKNOWN" : quality;
}

bool isSignalGoodForTransmission() {
  const SignalInfo& signal = modemManager.getSignalInfo();
  
  // Check CSQ first
  if (signal.csq < 5) return false;
  
  // Check LTE metrics if available
  if (signal.hasLteMetrics()) {
    if (signal.rsrq < RSRQ_POOR_THRESHOLD || signal.rsrp < RSRP_POOR_THRESHOLD) {
      return false;
    }
  }
  
  return true;
}

// ========================================
// ENHANCED DATA TRANSMISSION
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
  
  // Check signal quality before transmission
  if (ENABLE_SIGNAL_MONITORING && !isSignalGoodForTransmission()) {
    LOG_WARN(MODULE_SIGNAL, "⚠️ Poor signal quality, transmission may be slow");
  }
  
  // Send data with enhanced monitoring
  bool success = false;
  
  if (gpsManager.isValid() || useManualSpeed) {
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    // Start latency measurement
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      wsManager.startLatencyMeasurement();
      modemManager.startLatencyMeasurement();
    }
    
    // Get signal metrics if available
    float rsrq = RSRQ_INVALID_VALUE;
    float rsrp = RSRP_INVALID_VALUE;
    
    if (ENABLE_SIGNAL_MONITORING && powerConfigs[currentPowerMode].signalMonitoring) {
      rsrq = modemManager.getRSRQ();
      rsrp = modemManager.getRSRP();
    }
    
    // Include manual speed in data if testing
    float displaySpeed = useManualSpeed ? manualSpeed : gpsManager.getSpeed();
    
    // Send data with signal monitoring (UPDATED)
    success = wsManager.sendVehicleData(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      displaySpeed,
      gpsManager.getSatellites(),
      timestamp,
      rsrq,  // NEW: RSRQ value
      rsrp   // NEW: RSRP value
    );
    
    if (success) {
      if (powerConfigs[currentPowerMode].performanceMonitoring) {
        modemManager.endLatencyMeasurement();
      }
      
      LOG_INFO(MODULE_GPS, "✅ Data sent successfully");
      LOG_INFO(MODULE_GPS, "📍 Pos: %.6f, %.6f | 🚗 %.1f km/h | 🛰️ %d sats",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               displaySpeed, gpsManager.getSatellites());
      
      // Log signal metrics if available
      if (ENABLE_SIGNAL_MONITORING && rsrq != RSRQ_INVALID_VALUE && rsrp != RSRP_INVALID_VALUE) {
        LOG_INFO(MODULE_SIGNAL, "📶 Signal: RSRQ=%.1f dB, RSRP=%.1f dBm", rsrq, rsrp);
      }
      
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

// NEW: Callback for data confirmation
void onDataConfirmation(const String& id, bool success, unsigned long latency) {
  if (success) {
    LOG_INFO(MODULE_PERF, "✅ Data confirmed in database: %s (latency: %lu ms)", 
             id.c_str(), latency);
    
    // Update end-to-end performance metrics
    updatePerformanceMetrics(true, 0, latency);
  } else {
    LOG_WARN(MODULE_PERF, "❌ Data confirmation failed: %s", id.c_str());
    updatePerformanceMetrics(false, 0, 0);
  }
  
  // Reset waiting state
  waitingForConfirmation = false;
  lastTransmissionId = "";
}

// ========================================
// ENHANCED SERIAL COMMAND HANDLERS
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
  // Signal monitoring commands (NEW)
  else if (cmd.startsWith("signal")) {
    processSignalCommand(cmd);
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

// NEW: Process signal monitoring commands
void processSignalCommand(const String& cmd) {
  if (cmd == "signal") {
    // Show current signal info
    printSignalInfo();
  } else if (cmd == "signal update") {
    // Force signal update
    if (ENABLE_SIGNAL_MONITORING) {
      LOG_INFO(MODULE_SIGNAL, "Updating signal metrics...");
      modemManager.updateSignalInfo();
      printSignalInfo();
    } else {
      LOG_WARN(MODULE_SIGNAL, "Signal monitoring disabled");
    }
  } else if (cmd == "signal enable") {
    // Enable signal monitoring
    if (ENABLE_SIGNAL_MONITORING) {
      modemManager.setSignalMonitoring(true);
      LOG_INFO(MODULE_SIGNAL, "Signal monitoring enabled");
    } else {
      LOG_WARN(MODULE_SIGNAL, "Signal monitoring not supported in this build");
    }
  } else if (cmd == "signal disable") {
    // Disable signal monitoring
    modemManager.setSignalMonitoring(false);
    LOG_INFO(MODULE_SIGNAL, "Signal monitoring disabled");
  } else if (cmd == "signal report") {
    // Detailed signal report
    if (ENABLE_SIGNAL_MONITORING) {
      LOG_INFO(MODULE_SIGNAL, modemManager.getSignalQualityReport().c_str());
    } else {
      LOG_WARN(MODULE_SIGNAL, "Signal monitoring disabled");
    }
  }
}

void printSignalInfo() {
  if (!ENABLE_SIGNAL_MONITORING) {
    LOG_INFO(MODULE_SIGNAL, "Signal monitoring disabled");
    return;
  }
  
  logSignalMetrics();
}

// ========================================
// ENHANCED STATUS DISPLAY FUNCTIONS
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
  
  SerialMon.println("\n=== SIGNAL MONITORING ===");  // NEW
  SerialMon.println("signal       - Show signal info");
  SerialMon.println("signal update- Force signal update");
  SerialMon.println("signal report- Detailed signal report");
  SerialMon.println("signal enable- Enable signal monitoring");
  SerialMon.println("signal disable- Disable signal monitoring");
  
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
  
  // Network info with signal monitoring
  SerialMon.printf("Modem        : %s", modemManager.getStatusString());
  if (modemManager.areOptimizationsApplied()) {
    SerialMon.print(" [OPT]");
  }
  SerialMon.println();
  
  SerialMon.printf("Signal       : %d (%s)\n", 
                   modemManager.getSignalQuality(),
                   Utils::getSignalQualityString(modemManager.getSignalQuality()));
  
  // Signal monitoring info (NEW)
  if (ENABLE_SIGNAL_MONITORING && powerConfigs[currentPowerMode].signalMonitoring) {
    float rsrq = modemManager.getRSRQ();
    float rsrp = modemManager.getRSRP();
    
    if (rsrq != RSRQ_INVALID_VALUE && rsrp != RSRP_INVALID_VALUE) {
      SerialMon.printf("RSRQ/RSRP    : %.1f dB / %.1f dBm (%s)\n", 
                       rsrq, rsrp, getSignalQualityString(rsrq, rsrp).c_str());
    } else {
      SerialMon.println("RSRQ/RSRP    : INVALID");
    }
  }
  
  SerialMon.printf("WebSocket    : %s\n", wsManager.getStateString());
  
  // Battery
  float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                     (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  percentage = constrain(percentage, 0.0, 100.0);
  SerialMon.printf("Battery      : %.2fV (%.0f%%)\n", batteryVoltage, percentage);
  
  // Performance with end-to-end latency
  if (performanceMetrics.totalTransmissions > 0) {
    SerialMon.printf("Transmissions: %lu/%lu (%.1f%%)\n",
                     performanceMetrics.successfulTransmissions,
                     performanceMetrics.totalTransmissions,
                     (performanceMetrics.successfulTransmissions * 100.0 / 
                      performanceMetrics.totalTransmissions));
    
    if (performanceMetrics.endToEndSamples > 0) {
      unsigned long avgEndToEnd = performanceMetrics.totalEndToEndLatency / 
                                 performanceMetrics.endToEndSamples;
      SerialMon.printf("DB Latency   : %lu ms avg (%lu samples)\n", 
                       avgEndToEnd, performanceMetrics.endToEndSamples);
    }
  }
  
  SerialMon.printf("Uptime       : %s\n", Utils::formatUptime(millis()).c_str());
  SerialMon.printf("Free Memory  : %u KB\n", Utils::getFreeHeap() / 1024);
  
  SerialMon.println("===================================\n");
}

// ========================================
// ENHANCED PERFORMANCE MONITORING
// ========================================

void updatePerformanceMetrics(bool success, unsigned long latency, unsigned long endToEndLatency) {
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
    
    // NEW: End-to-end latency tracking
    if (endToEndLatency > 0) {
      performanceMetrics.endToEndSamples++;
      performanceMetrics.totalEndToEndLatency += endToEndLatency;
      
      if (endToEndLatency < performanceMetrics.minEndToEndLatency) {
        performanceMetrics.minEndToEndLatency = endToEndLatency;
      }
      if (endToEndLatency > performanceMetrics.maxEndToEndLatency) {
        performanceMetrics.maxEndToEndLatency = endToEndLatency;
      }
    }
  } else {
    performanceMetrics.failedTransmissions++;
  }
}

void printPerformanceReport() {
  LOG_INFO(MODULE_PERF, "=== ENHANCED PERFORMANCE REPORT ===");
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
  
  // NEW: End-to-end latency report
  if (performanceMetrics.endToEndSamples > 0) {
    unsigned long avgEndToEnd = performanceMetrics.totalEndToEndLatency / 
                               performanceMetrics.endToEndSamples;
    LOG_INFO(MODULE_PERF, "=== END-TO-END LATENCY ===");
    LOG_INFO(MODULE_PERF, "DB Samples   : %lu", performanceMetrics.endToEndSamples);
    LOG_INFO(MODULE_PERF, "DB Avg       : %lu ms", avgEndToEnd);
    LOG_INFO(MODULE_PERF, "DB Min       : %lu ms", performanceMetrics.minEndToEndLatency);
    LOG_INFO(MODULE_PERF, "DB Max       : %lu ms", performanceMetrics.maxEndToEndLatency);
    LOG_INFO(MODULE_PERF, "DB Performance: %s", 
             avgEndToEnd <= MAX_ACCEPTABLE_LATENCY ? "✅ GOOD" : "⚠️ NEEDS IMPROVEMENT");
  }
  
  LOG_INFO(MODULE_PERF, modemManager.getPerformanceReport().c_str());
  LOG_INFO(MODULE_PERF, wsManager.getPerformanceReport().c_str());
  
  // NEW: Signal quality impact on performance
  if (ENABLE_SIGNAL_MONITORING) {
    const SignalInfo& signal = modemManager.getSignalInfo();
    if (signal.hasLteMetrics()) {
      float score = SignalAnalysis::calculateSignalScore(signal);
      LOG_INFO(MODULE_PERF, "=== SIGNAL IMPACT ===");
      LOG_INFO(MODULE_PERF, "Signal Score : %.1f/100", score);
      LOG_INFO(MODULE_PERF, "Signal Impact: %s", 
               score > 70 ? "✅ MINIMAL" : score > 40 ? "⚠️ MODERATE" : "❌ HIGH");
    }
  }
}

// ========================================
// REST OF THE IMPLEMENTATION
// ========================================
// [Keep all the remaining functions from the original implementation]
// [Just ensure they integrate with the new signal monitoring features]

// State handlers remain largely the same...
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
    
    // Initialize signal monitoring if enabled
    if (ENABLE_SIGNAL_MONITORING && powerConfigs[currentPowerMode].signalMonitoring) {
      modemManager.setSignalMonitoring(true);
      LOG_INFO(MODULE_SIGNAL, "Signal monitoring initialized");
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

// [Include all other state handlers and functions from the original implementation]
// [The rest of the code remains the same, with signal monitoring integration]

// ========================================
// STATE MACHINE IMPLEMENTATION
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
// STATE HANDLERS IMPLEMENTATION
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
      
      // Re-enable signal monitoring if needed
      if (ENABLE_SIGNAL_MONITORING && powerConfigs[currentPowerMode].signalMonitoring) {
        modemManager.setSignalMonitoring(true);
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
    
    // Re-enable signal monitoring
    if (ENABLE_SIGNAL_MONITORING && powerConfigs[currentPowerMode].signalMonitoring) {
      modemManager.setSignalMonitoring(true);
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
// MOVEMENT DETECTION IMPLEMENTATION
// ========================================

float getCurrentSpeed() {
  return useManualSpeed ? manualSpeed : gpsManager.getSpeed();
}

void updateMovementState() {
  static unsigned long lastMovementLog = 0;
  MovementState previousState = currentMovementState;
  
  float currentSpeed = getCurrentSpeed();
  
  // Determine movement state
  if (currentSpeed > MOVEMENT_SPEED_THRESHOLD) {
    // Speed > 1 km/h = MOVING
    currentMovementState = MOVEMENT_MOVING;
    vehicleStopTime = 0;
  } else {
    // Speed <= 1 km/h
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
      LOG_INFO(MODULE_GPS, "📡 GPS interval: %d seconds", GPS_INTERVAL_STATIC/1000);
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
// CONNECTION MANAGEMENT IMPLEMENTATION
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
    updatePerformanceMetrics(true, latency, 0);  // End-to-end latency will be updated via callback
    
    lastGpsSendTime = currentTime;
    lastSuccessfulOperation = currentTime;
    lastActivityTime = currentTime;
    
    wsManager.endLatencyMeasurement();
    
    if (powerConfigs[currentPowerMode].performanceMonitoring && 
        DEBUG_LATENCY_TRACKING) {
      LOG_DEBUG(MODULE_PERF, "📊 Transmission completed in %lu ms", latency);
    }
  } else {
    updatePerformanceMetrics(false, 0, 0);
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
// ENHANCED SERIAL COMMAND PROCESSING
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
  } else if (cmd == "testsignal") {
    // Test signal monitoring
    if (ENABLE_SIGNAL_MONITORING) {
      LOG_INFO(MODULE_MAIN, "🧪 Testing signal monitoring...");
      modemManager.updateSignalInfo();
      printSignalInfo();
    } else {
      LOG_WARN(MODULE_MAIN, "Signal monitoring not enabled");
    }
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
// MONITORING FUNCTIONS IMPLEMENTATION
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
  
  // Re-enable signal monitoring after optimization
  if (ENABLE_SIGNAL_MONITORING && powerConfigs[currentPowerMode].signalMonitoring) {
    modemManager.setSignalMonitoring(true);
  }
  
  performanceMetrics.consecutiveSlowTransmissions = 0;
  performanceMetrics.consecutiveFailures = 0;
  
  modemManager.logOptimizationDetails();
  
  LOG_INFO(MODULE_PERF, "✅ Optimizations complete");
}

// ========================================
// POWER MANAGEMENT IMPLEMENTATION
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
  
  // Update signal monitoring based on power mode
  if (ENABLE_SIGNAL_MONITORING && systemFlags.modemReady) {
    modemManager.setSignalMonitoring(config.signalMonitoring);
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
// RELAY CONTROL IMPLEMENTATION
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
// ENHANCED STATUS DISPLAY FUNCTIONS
// ========================================

void printSystemReadyStatus() {
  LOG_INFO(MODULE_SYS, "=== SYSTEM READY STATUS ===");
  LOG_INFO(MODULE_SYS, "GPS Ready    : %s", systemFlags.gpsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Modem Ready  : %s", systemFlags.modemReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Network Ready: %s", systemFlags.networkReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "GPRS Ready   : %s", systemFlags.gprsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "WebSocket    : %s", systemFlags.wsReady ? "YES" : "NO");
  LOG_INFO(MODULE_SYS, "Overall      : %s", isSystemReady() ? "READY ✅" : "NOT READY ⚠️");
  
  // Show signal monitoring status
  if (ENABLE_SIGNAL_MONITORING) {
    LOG_INFO(MODULE_SIGNAL, "Signal Mon   : %s", 
             powerConfigs[currentPowerMode].signalMonitoring ? "ENABLED" : "DISABLED");
  }
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
  SerialMon.printf("Signal Mon   : %s\n", config.signalMonitoring ? "Yes" : "No");  // NEW
  
  SerialMon.println("\n=== MOVEMENT INTERVALS ===");
  SerialMon.printf("MOVING       : %d seconds\n", GPS_INTERVAL_MOVING/1000);
  SerialMon.printf("PARKED       : %d seconds\n", GPS_INTERVAL_PARKED/1000);
  SerialMon.printf("STATIC       : %d seconds\n", GPS_INTERVAL_STATIC/1000);
  SerialMon.printf("Speed Thresh : %.1f km/h\n", float(MOVEMENT_SPEED_THRESHOLD));
  SerialMon.printf("Park→Static  : %d minutes\n", PARKED_TO_STATIC_TIMEOUT/60000);
  
  SerialMon.println("=============================\n");
}

void printWebSocketStats() {
  LOG_INFO(MODULE_WS, wsManager.getPerformanceReport().c_str());
}

// ========================================
// HELPER FUNCTIONS IMPLEMENTATION
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

String formatTimestamp(unsigned long unixTime) {
  time_t rawTime = unixTime;
  struct tm *timeInfo = gmtime(&rawTime);
  
  char timestamp[32];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          timeInfo->tm_year + 1900, 
          timeInfo->tm_mon + 1, 
          timeInfo->tm_mday,
          timeInfo->tm_hour, 
          timeInfo->tm_min, 
          timeInfo->tm_sec);
  
  return String(timestamp);
}