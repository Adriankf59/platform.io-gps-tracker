// ========================================
// Main.cpp - Sistem GPS Tracker dengan Performance Optimization
// ========================================

/**
 * ESP32 Vehicle GPS Tracking dengan Low Latency Optimization
 * - Performance monitoring terintegrasi
 * - Network optimization otomatis
 * - Adaptive transmission intervals
 * - Fast error recovery
 * 
 * Versi: 6.0 - Performance Optimized Edition
 * Update: Full integration dengan modul yang dioptimasi untuk latensi <1.5 detik
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

// ----- MODE POWER -----
enum PowerMode {
  POWER_MODE_FULL,      // Mode Aktif Penuh - optimized for low latency
  POWER_MODE_STANDBY,   // Mode Standby - balanced performance
  POWER_MODE_EMERGENCY  // Mode Darurat - ultra hemat daya
};

// ----- STATE SISTEM -----
enum SystemState {
  STATE_INIT,                 // Inisialisasi sistem
  STATE_OPERATIONAL,          // Operasional normal
  STATE_MODEM_RESET,         // Reset modem dalam proses
  STATE_CONNECTION_RECOVERY,  // Pemulihan koneksi
  STATE_ERROR,               // Error state
  STATE_SLEEP_PREPARE,       // Persiapan sleep
  STATE_SLEEPING,            // Dalam mode sleep
  STATE_OPTIMIZING           // Applying network optimizations
};

// ----- STATE PERGERAKAN -----
enum MovementState {
  MOVEMENT_UNKNOWN,   // Status belum diketahui
  MOVEMENT_STATIC,    // Kendaraan diam
  MOVEMENT_MOVING     // Kendaraan bergerak
};

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
} performanceMetrics = {0, 0, 0, 0, UINT32_MAX, 0, 0, 0, 0, 0};

// ----- OBJEK GLOBAL -----
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
GpsManager gpsManager(gps, SerialGPS);
ModemManager modemManager(modem, SerialAT);
WebSocketManager wsManager(&gsmClient);

// ----- VARIABEL STATE -----
SystemState currentState = STATE_INIT;
PowerMode currentPowerMode = POWER_MODE_FULL;
MovementState currentMovementState = MOVEMENT_UNKNOWN;
unsigned long lastGpsSendTime = 0;
unsigned long lastSuccessfulOperation = 0;
unsigned long lastActivityTime = 0;
unsigned long lastSignalCheck = 0;
unsigned long lastMaintenanceCheck = 0;
unsigned long lastPerformanceOptimization = 0;
bool relayState = true;

// ----- TESTING FLAGS -----
bool waitForSubscription = true;  // Flag untuk menunggu WebSocket subscription sebelum kirim data

// ----- OPTIMIZED POWER CONFIGURATIONS -----
struct PowerModeConfig {
  unsigned long gpsInterval;            // Interval update GPS
  unsigned long wsKeepAliveInterval;     // Interval keepalive WebSocket
  unsigned long sleepDuration;          // Durasi sleep mode
  bool gpsAlwaysOn;                    // GPS selalu aktif
  bool wsContinuous;                   // WebSocket selalu terhubung
  bool relayEnabled;                   // Kontrol relay diizinkan
  bool performanceMonitoring;          // Enable performance monitoring
  bool aggressiveOptimization;         // Apply aggressive optimizations
};

// Konfigurasi untuk setiap mode power (OPTIMIZED)
PowerModeConfig powerConfigs[3] = {
  // POWER_MODE_FULL - Optimized for testing realtime
  {
    .gpsInterval = 2000,                     // 2 seconds for testing (was 5000)
    .wsKeepAliveInterval = WS_PING_INTERVAL, // Use optimized 20s interval
    .sleepDuration = 0,                      // No sleep
    .gpsAlwaysOn = true,
    .wsContinuous = true,
    .relayEnabled = true,
    .performanceMonitoring = true,           // Full monitoring
    .aggressiveOptimization = true           // Maximum optimization
  },
  // POWER_MODE_STANDBY - Balanced performance
  {
    .gpsInterval = 30000,                    // 30 seconds (was 60000)
    .wsKeepAliveInterval = 60000,            // 60 seconds
    .sleepDuration = 0,                      // Disable sleep for testing (was 50000)
    .gpsAlwaysOn = true,                     // Keep GPS on for testing
    .wsContinuous = true,                    // Keep WS connected for testing
    .relayEnabled = true,
    .performanceMonitoring = false,
    .aggressiveOptimization = false
  },
  // POWER_MODE_EMERGENCY - Maximum power saving (but disabled for testing)
  {
    .gpsInterval = 300000,                   // 5 minutes (was 1800000)
    .wsKeepAliveInterval = 0,                // No keepalive
    .sleepDuration = 0,                      // Disable sleep for testing (was 1740000)
    .gpsAlwaysOn = true,                     // Keep GPS on for testing
    .wsContinuous = false,
    .relayEnabled = false,
    .performanceMonitoring = false,
    .aggressiveOptimization = false
  }
};

// Battery voltage untuk testing
float batteryVoltage = 12.6;

// ----- PROTOTYPE FUNGSI -----
void handleInitState();
void handleOperationalState();
void handleModemResetState();
void handleConnectionRecoveryState();
void handleSleepPrepareState();
void handleOptimizingState();
void handleSerialCommands();
void printStatus();
void printHelp();
void printPowerModeInfo();
void printWebSocketStats();
void printPerformanceReport();
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
void updateMovementState();
void checkSignalQuality();
void performanceOptimizationCheck();
void updatePerformanceMetrics(bool success, unsigned long latency);
void checkNetworkHealth();
void applyPerformanceOptimizations();
unsigned long getGpsIntervalForMovement();
const char* getPowerModeString(PowerMode mode);
const char* getStateString(SystemState state);
const char* getMovementString(MovementState movement);
String formatTimestamp(unsigned long unixTime);

// ----- SETUP -----
void setup() {
  // Inisialisasi Serial dengan increased buffer
  SerialMon.begin(115200);
  SerialMon.setRxBufferSize(1024);
  delay(100);
  
  // Inisialisasi Logger
  #ifdef DEBUG_MODE
    Logger::init(&SerialMon, LOG_DEBUG);
  #else
    Logger::init(&SerialMon, LOG_INFO);
  #endif
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS Tracker - Performance Optimized ===");
  LOG_INFO(MODULE_MAIN, "Versi: 6.0 (Low Latency Edition)");
  
  #if TESTING_MODE
    LOG_WARN(MODULE_MAIN, "⚠️ TESTING MODE ACTIVE - Sleep/Emergency disabled");
    LOG_INFO(MODULE_MAIN, "Battery thresholds: Low=%.1fV, Recovery=%.1fV", 
             BATTERY_LOW_THRESHOLD, BATTERY_RECOVERY_THRESHOLD);
    LOG_INFO(MODULE_MAIN, "Activity timeout: %lu ms", ACTIVITY_TIMEOUT);
  #endif
  
  LOG_INFO(MODULE_MAIN, "Target Latency: <%d ms", MAX_ACCEPTABLE_LATENCY);
  LOG_INFO(MODULE_MAIN, "Mode Power: %s", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "Device ID: %s", GPS_ID);
  LOG_INFO(MODULE_MAIN, "Payload Mode: %s", 
           DEFAULT_PAYLOAD_MODE == PAYLOAD_MODE_ESSENTIAL ? "ESSENTIAL" :
           DEFAULT_PAYLOAD_MODE == PAYLOAD_MODE_FULL ? "FULL" : "MINIMAL");
  LOG_INFO(MODULE_MAIN, "Compiled: %s %s", __DATE__, __TIME__);
  
  // Info about optimizations
  LOG_INFO(MODULE_MAIN, "=== OPTIMIZATION FEATURES ===");
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    LOG_INFO(MODULE_MAIN, "✅ Dynamic GPS Interval: ENABLED");
    LOG_INFO(MODULE_MAIN, "  - Moving: %d ms", GPS_SEND_INTERVAL_MOVING);
    LOG_INFO(MODULE_MAIN, "  - Static: %d ms", GPS_SEND_INTERVAL_STATIC);
  #endif
  
  #if ENABLE_LATENCY_MONITORING
    LOG_INFO(MODULE_MAIN, "✅ Latency Monitoring: ENABLED");
  #endif
  
  #if ENABLE_ADAPTIVE_OPTIMIZATION
    LOG_INFO(MODULE_MAIN, "✅ Adaptive Optimization: ENABLED");
  #endif
  
  // Inisialisasi watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  LOG_INFO(MODULE_SYS, "Watchdog timer: %d detik", WATCHDOG_TIMEOUT/1000);
  
  // Inisialisasi hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  LOG_INFO(MODULE_RELAY, "Relay diinisialisasi: ON");
  
  // Inisialisasi battery monitoring jika enabled
  #if ENABLE_BATTERY_MONITORING
    pinMode(BATTERY_ADC_PIN, INPUT);
    LOG_INFO(MODULE_SYS, "Battery monitoring: ENABLED");
  #endif
  
  // Inisialisasi manajer dengan optimized settings
  LOG_INFO(MODULE_MAIN, "Menginisialisasi modul...");
  gpsManager.begin();
  
  // Enable high update rate GPS jika didukung
  #ifdef GPS_UPDATE_RATE
    gpsManager.enableHighUpdateRate(GPS_UPDATE_RATE);
    LOG_INFO(MODULE_GPS, "GPS update rate: %d Hz", GPS_UPDATE_RATE);
  #endif
  
  modemManager.begin();
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  // Reset performance metrics
  performanceMetrics = {0, 0, 0, 0, UINT32_MAX, 0, 0, 0, 0, 0};
  
  // Tampilkan bantuan awal
  printHelp();
  printPowerModeInfo();
  
  // Tampilkan info memory awal
  Utils::printMemoryInfo();
  
  // Set state awal
  currentState = STATE_INIT;
  lastSuccessfulOperation = millis();
  lastActivityTime = millis();
  lastSignalCheck = millis();
  lastMaintenanceCheck = millis();
  lastPerformanceOptimization = millis();
  
  LOG_INFO(MODULE_MAIN, "Setup selesai, memulai operasi...");
}

// ----- LOOP UTAMA -----
void loop() {
  // Feed watchdog
  Utils::feedWatchdog();
  
  // Update status baterai
  updateBatteryStatus();
  
  // Cek apakah perlu mode darurat
  checkEmergencyMode();
  
  // Monitor kualitas sinyal dan network health
  checkSignalQuality();
  checkNetworkHealth();
  
  // Performance optimization check (every 5 minutes or when needed)
  performanceOptimizationCheck();
  
  // Update GPS berdasarkan konfigurasi mode power
  if (powerConfigs[currentPowerMode].gpsAlwaysOn || 
      (millis() - lastGpsSendTime > powerConfigs[currentPowerMode].gpsInterval - 30000)) {
    gpsManager.update();
    
    // Update movement state setelah GPS update
    updateMovementState();
  }
  
  // Update WebSocket dengan maintenance jika dalam mode continuous
  if (currentState == STATE_OPERATIONAL && powerConfigs[currentPowerMode].wsContinuous) {
    wsManager.update();
    
    // Maintain connection health
    static unsigned long lastWsMaintenance = 0;
    if (millis() - lastWsMaintenance > CONNECTION_HEALTH_CHECK_INTERVAL) {
      wsManager.maintainConnection();
      lastWsMaintenance = millis();
    }
  }
  
  // Maintain modem connection
  if (currentState == STATE_OPERATIONAL) {
    static unsigned long lastModemMaintenance = 0;
    if (millis() - lastModemMaintenance > ModemManager::MAINTENANCE_INTERVAL) {
      modemManager.maintainConnection();
      lastModemMaintenance = millis();
    }
  }
  
  // Handle perintah serial
  handleSerialCommands();
  
  // State machine
  switch (currentState) {
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
      LOG_ERROR(MODULE_SYS, "Sistem dalam kondisi error, mencoba recovery...");
      modemManager.startReset();
      currentState = STATE_MODEM_RESET;
      break;
      
    case STATE_SLEEPING:
      // Tidak seharusnya sampai ke sini
      LOG_WARN(MODULE_SYS, "State SLEEPING tidak valid dalam loop");
      currentState = STATE_OPERATIONAL;
      break;
  }
  
  delay(5); // Reduced from 10ms for faster response
}

// ----- HANDLER STATE -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "🚀 Menginisialisasi sistem dengan optimasi...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "✅ Inisialisasi sistem berhasil");
    
    // Apply network optimizations immediately
    if (!modemManager.areOptimizationsApplied()) {
      LOG_INFO(MODULE_SYS, "🔧 Menerapkan optimasi jaringan...");
      if (modemManager.applyNetworkOptimizations()) {
        LOG_INFO(MODULE_SYS, "✅ Optimasi jaringan berhasil diterapkan");
      }
    }
    
    // Log network info
    LOG_INFO(MODULE_MODEM, modemManager.getNetworkInfo().c_str());
    
    // Koneksi WebSocket jika mode continuous
    if (powerConfigs[currentPowerMode].wsContinuous) {
      if (wsManager.connect()) {
        LOG_INFO(MODULE_SYS, "✅ WebSocket terhubung");
      } else {
        LOG_WARN(MODULE_SYS, "⚠️ Koneksi WebSocket gagal, akan dicoba lagi...");
      }
    }
    
    currentState = STATE_OPERATIONAL;
    lastSuccessfulOperation = millis();
    
    // Print initial performance report
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      LOG_INFO(MODULE_PERF, "Performance monitoring aktif");
    }
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Inisialisasi gagal, memulai reset modem");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

void handleOperationalState() {
  unsigned long currentTime = millis();
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  // Gunakan dynamic GPS interval jika di mode FULL dan movement detection aktif
  unsigned long gpsInterval = config.gpsInterval;
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    if (currentPowerMode == POWER_MODE_FULL) {
      gpsInterval = getGpsIntervalForMovement();
    }
  #endif
  
  // Cek apakah waktunya kirim data GPS
  if (currentTime - lastGpsSendTime >= gpsInterval) {
    #if TESTING_MODE && DEBUG_LATENCY_TRACKING
      LOG_INFO(MODULE_GPS, "⏱️ GPS send trigger: interval=%lu ms, movement=%s", 
               gpsInterval, getMovementString(currentMovementState));
    #endif
    
    LOG_DEBUG(MODULE_GPS, "Waktu untuk mengirim data GPS (interval: %lu ms)", gpsInterval);
    
    // Start performance measurement
    unsigned long transmissionStart = millis();
    
    bool success = sendVehicleDataViaWebSocket();
    
    if (success) {
      // Calculate latency
      unsigned long latency = millis() - transmissionStart;
      updatePerformanceMetrics(true, latency);
      
      lastGpsSendTime = currentTime;
      lastSuccessfulOperation = currentTime;
      lastActivityTime = currentTime;
      
      // End latency measurement in WebSocket manager
      wsManager.endLatencyMeasurement();
      
      // Log performance if monitoring enabled
      if (config.performanceMonitoring && DEBUG_LATENCY_TRACKING) {
        LOG_DEBUG(MODULE_PERF, "📊 Transmission completed in %lu ms", latency);
        
        if (latency > LATENCY_WARNING_THRESHOLD) {
          LOG_WARN(MODULE_PERF, "⚠️ Latency tinggi terdeteksi: %lu ms", latency);
          performanceMetrics.consecutiveSlowTransmissions++;
        } else {
          performanceMetrics.consecutiveSlowTransmissions = 0;
        }
      }
    } else {
      updatePerformanceMetrics(false, 0);
      performanceMetrics.consecutiveFailures++;
      
      // Trigger optimization if too many failures
      if (performanceMetrics.consecutiveFailures >= MAX_CONNECTION_FAILURES) {
        LOG_WARN(MODULE_PERF, "🔧 Multiple failures detected (%d), triggering optimization", 
                 performanceMetrics.consecutiveFailures);
        currentState = STATE_OPTIMIZING;
      }
    }
  }
  
  // Masuk mode sleep jika dikonfigurasi dan tidak ada aktivitas
  if (config.sleepDuration > 0 && 
      currentTime - lastActivityTime > ACTIVITY_TIMEOUT) { // Use testing constant
    LOG_INFO(MODULE_SYS, "Tidak ada aktivitas, bersiap untuk sleep");
    currentState = STATE_SLEEP_PREPARE;
  }
  
  // Cek kesehatan koneksi dengan reduced timeout
  if (currentTime - lastSuccessfulOperation > 180000) { // Reduced from 300000 to 180000 (3 menit)
    LOG_WARN(MODULE_SYS, "⚠️ Tidak ada operasi berhasil dalam 3 menit, memulai recovery");
    currentState = STATE_CONNECTION_RECOVERY;
  }
}

void handleOptimizingState() {
  LOG_INFO(MODULE_SYS, "🔧 Menerapkan optimasi performance...");
  
  // Apply all optimizations
  applyPerformanceOptimizations();
  
  // Reset consecutive counters
  performanceMetrics.consecutiveSlowTransmissions = 0;
  performanceMetrics.consecutiveFailures = 0;
  
  // Return to operational
  currentState = STATE_OPERATIONAL;
  LOG_INFO(MODULE_SYS, "✅ Optimasi selesai, kembali ke operasional");
}

void handleSleepPrepareState() {
  LOG_INFO(MODULE_SYS, "😴 Mempersiapkan mode sleep...");
  
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  // Log performance stats sebelum sleep
  if (config.performanceMonitoring) {
    printPerformanceReport();
  }
  
  // Log WebSocket stats sebelum disconnect
  if (wsManager.getState() != WS_DISCONNECTED) {
    printWebSocketStats();
  }
  
  // Putuskan WebSocket jika tidak continuous
  if (!config.wsContinuous) {
    wsManager.disconnect();
    LOG_INFO(MODULE_WS, "WebSocket diputuskan untuk mode sleep");
  }
  
  // Matikan GPS jika tidak always on
  if (!config.gpsAlwaysOn) {
    LOG_INFO(MODULE_GPS, "Mematikan GPS untuk hemat daya");
    // Implementasi power down GPS tergantung hardware
  }
  
  // Nonaktifkan peripheral yang tidak digunakan
  disableUnusedPeripherals();
  
  // Masuk mode sleep yang sesuai
  if (currentPowerMode == POWER_MODE_EMERGENCY) {
    LOG_INFO(MODULE_SYS, "Memasuki deep sleep untuk %lu ms", config.sleepDuration);
    enterDeepSleep(config.sleepDuration);
  } else {
    LOG_INFO(MODULE_SYS, "Memasuki light sleep untuk %lu ms", config.sleepDuration);
    enterLightSleep(config.sleepDuration);
  }
  
  // Setelah bangun dari sleep
  LOG_INFO(MODULE_SYS, "⏰ Bangun dari sleep mode");
  enablePeripherals();
  currentState = STATE_OPERATIONAL;
  lastActivityTime = millis();
  
  // Re-apply optimizations after wake
  if (powerConfigs[currentPowerMode].aggressiveOptimization) {
    modemManager.forceOptimizationReapply();
  }
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
    // Reset selesai, coba setup lagi
    if (modemManager.setup()) {
      LOG_INFO(MODULE_SYS, "✅ Reset modem berhasil");
      
      // Apply optimizations after reset
      if (powerConfigs[currentPowerMode].aggressiveOptimization) {
        modemManager.applyNetworkOptimizations();
      }
      
      // Reset WebSocket reconnect attempts
      wsManager.resetReconnectAttempts();
      
      // Reset performance counters
      performanceMetrics.consecutiveFailures = 0;
      
      currentState = STATE_OPERATIONAL;
    } else {
      LOG_ERROR(MODULE_SYS, "❌ Reset modem gagal");
      currentState = STATE_ERROR;
    }
  }
}

void handleConnectionRecoveryState() {
  LOG_INFO(MODULE_SYS, "🔄 Mencoba pemulihan koneksi cepat...");
  
  // Disconnect semua koneksi
  wsManager.disconnect();
  modemManager.disconnectGprs();
  Utils::safeDelay(1000); // Reduced from 2000
  
  // Coba koneksi ulang GPRS
  if (modemManager.connectGprs()) {
    LOG_INFO(MODULE_SYS, "✅ Pemulihan GPRS berhasil");
    
    // Re-apply optimizations
    if (!modemManager.areOptimizationsApplied()) {
      modemManager.forceOptimizationReapply();
    }
    
    // Reset WebSocket reconnect attempts
    wsManager.resetReconnectAttempts();
    
    // Reconnect WebSocket jika mode continuous
    if (powerConfigs[currentPowerMode].wsContinuous) {
      if (wsManager.connect()) {
        LOG_INFO(MODULE_SYS, "✅ WebSocket terhubung kembali");
        currentState = STATE_OPERATIONAL;
        lastSuccessfulOperation = millis();
        performanceMetrics.consecutiveFailures = 0;
      } else {
        LOG_ERROR(MODULE_SYS, "❌ Koneksi ulang WebSocket gagal");
        currentState = STATE_MODEM_RESET;
      }
    } else {
      currentState = STATE_OPERATIONAL;
    }
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Pemulihan koneksi gagal, mencoba reset modem");
    modemManager.startReset();
    currentState = STATE_MODEM_RESET;
  }
}

// ----- PERFORMANCE OPTIMIZATION -----
void performanceOptimizationCheck() {
  unsigned long now = millis();
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  // Skip if not enabled
  if (!config.aggressiveOptimization) {
    return;
  }
  
  // Check every 5 minutes
  if (now - lastPerformanceOptimization < AUTO_OPTIMIZATION_INTERVAL) {
    return;
  }
  
  lastPerformanceOptimization = now;
  
  // Check if optimizations are still applied
  if (!modemManager.areOptimizationsApplied()) {
    LOG_WARN(MODULE_PERF, "🔧 Optimasi tidak aktif, menerapkan ulang...");
    modemManager.reapplyOptimizations();
  }
  
  // Signal-based optimization
  int signal = modemManager.getSignalQuality();
  if (signal < SIGNAL_WEAK_THRESHOLD && signal != 99) {
    LOG_WARN(MODULE_PERF, "🔧 Sinyal lemah (%d), trigger optimasi", signal);
    modemManager.reapplyOptimizations();
  }
  
  // Performance-based optimization
  if (performanceMetrics.totalTransmissions > 10) {
    unsigned long avgLatency = performanceMetrics.totalLatency / performanceMetrics.totalTransmissions;
    if (avgLatency > MAX_ACCEPTABLE_LATENCY) {
      LOG_WARN(MODULE_PERF, "🔧 Rata-rata latensi tinggi (%lu ms), trigger optimasi", avgLatency);
      applyPerformanceOptimizations();
    }
  }
  
  // Check WebSocket performance
  if (!wsManager.isPerformanceGood()) {
    LOG_WARN(MODULE_PERF, "🔧 WebSocket performance buruk, trigger optimasi");
    applyPerformanceOptimizations();
  }
}

void applyPerformanceOptimizations() {
  LOG_INFO(MODULE_PERF, "🚀 Menerapkan optimasi performance menyeluruh...");
  
  // 1. Modem optimizations
  if (!modemManager.areOptimizationsApplied()) {
    modemManager.forceOptimizationReapply();
  }
  
  // 2. Ensure GPRS is optimized
  if (modemManager.isGprsConnected()) {
    modemManager.connectGprs(); // Re-connect with optimizations
  }
  
  // 3. Reset performance counters
  performanceMetrics.consecutiveSlowTransmissions = 0;
  performanceMetrics.consecutiveFailures = 0;
  
  // 4. Log current optimization status
  modemManager.logOptimizationDetails();
  
  LOG_INFO(MODULE_PERF, "✅ Optimasi performance selesai");
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
    }
  } else {
    performanceMetrics.failedTransmissions++;
  }
}

void checkNetworkHealth() {
  // Quick health check every 30 seconds
  if (millis() - lastMaintenanceCheck < CONNECTION_HEALTH_CHECK_INTERVAL) {
    return;
  }
  
  lastMaintenanceCheck = millis();
  
  // Check modem connection
  if (!modemManager.isGprsConnected() && currentState == STATE_OPERATIONAL) {
    LOG_WARN(MODULE_SYS, "⚠️ GPRS terputus, trigger recovery");
    currentState = STATE_CONNECTION_RECOVERY;
  }
  
  // Check if need network diagnostic
  if (performanceMetrics.consecutiveSlowTransmissions >= 3) {
    if (modemManager.performNetworkDiagnostic()) {
      LOG_INFO(MODULE_SYS, "✅ Network diagnostic passed");
    } else {
      LOG_WARN(MODULE_SYS, "❌ Network diagnostic failed");
      currentState = STATE_OPTIMIZING;
    }
  }
}

// ----- MONITORING SINYAL -----
void checkSignalQuality() {
  // Cek sinyal setiap SIGNAL_QUALITY_CHECK_INTERVAL
  if (millis() - lastSignalCheck > SIGNAL_QUALITY_CHECK_INTERVAL) {
    int signal = modemManager.getSignalQuality();
    
    if (modemManager.isSignalWeak()) {
      LOG_WARN(MODULE_MODEM, "⚠️ Sinyal lemah: %d (%s)", 
               signal, Utils::getSignalQualityString(signal));
      
      // Trigger optimization if signal too weak
      if (signal < SIGNAL_WEAK_THRESHOLD && powerConfigs[currentPowerMode].aggressiveOptimization) {
        LOG_WARN(MODULE_MODEM, "🔧 Sinyal sangat lemah, trigger optimasi");
        currentState = STATE_OPTIMIZING;
      }
    } else if (modemManager.isSignalStrong()) {
      LOG_DEBUG(MODULE_MODEM, "📶 Signal excellent: %d", signal);
    }
    
    lastSignalCheck = millis();
  }
}

// ----- MANAJEMEN MODE POWER -----
void setPowerMode(PowerMode mode) {
  if (mode == currentPowerMode) return;
  
  LOG_INFO(MODULE_SYS, "🔄 Mengganti mode power: %s → %s", 
           getPowerModeString(currentPowerMode), 
           getPowerModeString(mode));
  
  currentPowerMode = mode;
  PowerModeConfig& config = powerConfigs[mode];
  
  // Sesuaikan konfigurasi
  if (config.performanceMonitoring) {
    LOG_INFO(MODULE_PERF, "Performance monitoring: ENABLED");
    wsManager.resetPerformanceStats();
    modemManager.resetPerformanceStats();
  }
  
  if (config.aggressiveOptimization) {
    LOG_INFO(MODULE_PERF, "Aggressive optimization: ENABLED");
    applyPerformanceOptimizations();
  }
  
  // Perkiraan konsumsi daya
  float expectedCurrent = 0;
  switch(mode) {
    case POWER_MODE_FULL:
      expectedCurrent = 280; // ~280mA with optimizations
      break;
    case POWER_MODE_STANDBY:
      expectedCurrent = 100;  // ~100mA rata-rata
      break;
    case POWER_MODE_EMERGENCY:
      expectedCurrent = 25;   // ~25mA rata-rata
      break;
  }
  
  LOG_INFO(MODULE_SYS, "Perkiraan konsumsi arus: ~%.0f mA", expectedCurrent);
  printPowerModeInfo();
}

void enterLightSleep(unsigned long duration) {
  LOG_TRACE(MODULE_SYS, "Konfigurasi light sleep");
  
  // Konfigurasi timer wakeup
  esp_sleep_enable_timer_wakeup(duration * 1000); // Konversi ke microseconds
  
  // Pertahankan RTC memory
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  
  // Masuk light sleep
  esp_light_sleep_start();
}

void enterDeepSleep(unsigned long duration) {
  LOG_TRACE(MODULE_SYS, "Konfigurasi deep sleep");
  
  // Konfigurasi timer wakeup
  esp_sleep_enable_timer_wakeup(duration * 1000); // Konversi ke microseconds
  
  // Masuk deep sleep
  esp_deep_sleep_start();
  // Kode setelah ini tidak akan dieksekusi
}

void disableUnusedPeripherals() {
  // Nonaktifkan WiFi
  esp_wifi_stop();
  
  // Nonaktifkan Bluetooth
  esp_bt_controller_disable();
  
  LOG_DEBUG(MODULE_SYS, "Peripheral dinonaktifkan untuk hemat daya");
}

void enablePeripherals() {
  // Re-enable peripheral yang diperlukan setelah sleep
  LOG_DEBUG(MODULE_SYS, "Peripheral diaktifkan kembali");
}

// ----- FUNGSI BATERAI -----
float readBatteryVoltage() {
  #if ENABLE_BATTERY_MONITORING
    int adcValue = analogRead(BATTERY_ADC_PIN);
    float voltage = (adcValue / 4095.0) * 3.3 * BATTERY_VOLTAGE_DIVIDER_RATIO * BATTERY_CALIBRATION_FACTOR;
    return voltage;
  #else
    // Untuk testing, return nilai simulasi
    return batteryVoltage;
  #endif
}

void updateBatteryStatus() {
  static unsigned long lastBatteryCheck = 0;
  
  #if ENABLE_BATTERY_MONITORING
    if (millis() - lastBatteryCheck > BATTERY_READ_INTERVAL) {
      batteryVoltage = readBatteryVoltage();
      lastBatteryCheck = millis();
      
      // Calculate battery percentage
      float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                         (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
      percentage = constrain(percentage, 0.0, 100.0);
      
      LOG_DEBUG(MODULE_SYS, "Battery: %.2fV (%.0f%%) - %s", 
                batteryVoltage, percentage, Utils::getBatteryStatus(batteryVoltage));
    }
  #endif
}

void checkEmergencyMode() {
  // Auto-switch ke mode darurat jika baterai rendah
  if (batteryVoltage < BATTERY_LOW_THRESHOLD && currentPowerMode != POWER_MODE_EMERGENCY) {
    LOG_WARN(MODULE_SYS, "⚠️ Baterai rendah terdeteksi! Beralih ke mode darurat");
    setPowerMode(POWER_MODE_EMERGENCY);
  }
  
  // Kembali ke mode standby jika baterai pulih
  if (batteryVoltage > BATTERY_RECOVERY_THRESHOLD && currentPowerMode == POWER_MODE_EMERGENCY) {
    LOG_INFO(MODULE_SYS, "✅ Baterai pulih, beralih ke mode standby");
    setPowerMode(POWER_MODE_STANDBY);
  }
}

// ----- FUNGSI PENGIRIMAN DATA -----
bool sendVehicleDataViaWebSocket() {
  // Cek koneksi GPRS
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "❌ Tidak ada koneksi GPRS");
    return false;
  }
  
  // Koneksi WebSocket jika diperlukan (mode non-continuous)
  if (!powerConfigs[currentPowerMode].wsContinuous) {
    if (!wsManager.isReady()) {
      LOG_INFO(MODULE_WS, "Menghubungkan WebSocket untuk transmisi data...");
      if (!wsManager.connect()) {
        LOG_ERROR(MODULE_WS, "❌ Gagal menghubungkan WebSocket");
        return false;
      }
      delay(1000); // Reduced from 2000
    }
  }
  
  // FIX RACE CONDITION: Wait for SUBSCRIBED state
  if (powerConfigs[currentPowerMode].wsContinuous && waitForSubscription) {
    int retries = 0;
    while (wsManager.getState() != WS_SUBSCRIBED && retries < 30) { // 3 second timeout
      delay(100);
      wsManager.update(); // Process incoming messages
      retries++;
      
      if (retries % 10 == 0) {
        LOG_DEBUG(MODULE_WS, "Waiting for subscription... (%d/30)", retries);
      }
    }
    
    if (wsManager.getState() != WS_SUBSCRIBED) {
      LOG_ERROR(MODULE_WS, "❌ Timeout waiting for subscription");
      return false;
    }
  }
  
  LOG_INFO(MODULE_GPS, "📤 Mengirim data kendaraan [mode %s]...", 
           getPowerModeString(currentPowerMode));
  
  bool success = false;
  
  // Kirim data jika GPS valid
  if (gpsManager.isValid()) {
    // Get timestamp untuk compact transmission
    char timestamp[30];
    gpsManager.getTimestamp(timestamp, sizeof(timestamp));
    
    // Convert to unix time manually if needed
    // For now, use current millis as approximation
    unsigned long unixTime = millis() / 1000; // Simple unix timestamp
    
    // Start latency measurement
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      wsManager.startLatencyMeasurement();
      modemManager.startLatencyMeasurement();
    }
    
    // Use optimized compact transmission for better performance
    // Use sendVehicleData dengan string timestamp dari GPS
    success = wsManager.sendVehicleData(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      gpsManager.getSpeed(),
      gpsManager.getSatellites(),
      timestamp,  // <-- Gunakan string timestamp yang sudah diambil dari GPS
      batteryVoltage
    );
    
    if (success) {
      // End latency measurement in modem
      if (powerConfigs[currentPowerMode].performanceMonitoring) {
        modemManager.endLatencyMeasurement();
      }
      
      LOG_INFO(MODULE_GPS, "✅ Data kendaraan berhasil dikirim");
      LOG_INFO(MODULE_GPS, "📍 Posisi: %.6f, %.6f | 🚗 %.1f km/h | 🛰️ %d satelit",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               gpsManager.getSpeed(), gpsManager.getSatellites());
      LOG_DEBUG(MODULE_GPS, "📏 Altitude: %.1f m | 🧭 Heading: %.1f° | HDOP: %.1f",
                gpsManager.getAltitude(), gpsManager.getHeading(), gpsManager.getHDOP());
      
      // Log payload size if debug enabled
      if (DEBUG_PAYLOAD_SIZE) {
        LOG_DEBUG(MODULE_GPS, "📦 Payload size: %d bytes", wsManager.getLastPayloadSize());
      }
      
      // Log jika ada fix baru
      if (gpsManager.hasNewFix()) {
        LOG_INFO(MODULE_GPS, "🆕 GPS fix baru terdeteksi");
      }
    } else {
      LOG_ERROR(MODULE_GPS, "❌ Gagal mengirim data kendaraan");
    }
  } else {
    LOG_WARN(MODULE_GPS, "⚠️ Data GPS tidak valid, menunggu fix...");
    LOG_DEBUG(MODULE_GPS, "Satelit: %d, HDOP: %.1f", 
              gpsManager.getSatellites(), gpsManager.getHDOP());
  }
  
  // Putuskan WebSocket jika mode non-continuous
  if (!powerConfigs[currentPowerMode].wsContinuous && success) {
    delay(500); // Reduced from 1000
    LOG_INFO(MODULE_WS, "Memutuskan WebSocket (mode non-continuous)");
    wsManager.disconnect();
  }
  
  return success;
}

// ----- CALLBACK RELAY -----
void onRelayUpdate(bool newState) {
  // Cek apakah kontrol relay diizinkan
  if (!powerConfigs[currentPowerMode].relayEnabled) {
    LOG_WARN(MODULE_RELAY, "⚠️ Kontrol relay dinonaktifkan pada mode %s", 
             getPowerModeString(currentPowerMode));
    return;
  }
  
  // Update relay jika berubah
  if (newState != relayState) {
    LOG_INFO(MODULE_RELAY, "🔄 Update relay: %s → %s", 
             relayState ? "ON" : "OFF", newState ? "ON" : "OFF");
    
    digitalWrite(RELAY_PIN, newState ? RELAY_ON : RELAY_OFF);
    relayState = newState;
    
    LOG_INFO(MODULE_RELAY, "✅ Relay fisik diupdate ke: %s", newState ? "ON" : "OFF");
    
    // Reset activity timer karena ada perubahan relay
    lastActivityTime = millis();
  }
}

// ----- FUNGSI HELPER -----
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
    case MOVEMENT_MOVING: return "MOVING";
    default: return "INVALID";
  }
}

// ----- DETEKSI PERGERAKAN -----
void updateMovementState() {
  static unsigned long lastMovementLog = 0;
  MovementState previousState = currentMovementState;
  
  if (gpsManager.isValid()) {
    // Gunakan threshold dari config atau default
    #ifdef MOVEMENT_SPEED_THRESHOLD
      float speedThreshold = MOVEMENT_SPEED_THRESHOLD;
    #else
      float speedThreshold = 3.0; // Default 3 km/h
    #endif
    
    if (gpsManager.isMoving(speedThreshold)) {
      currentMovementState = MOVEMENT_MOVING;
    } else {
      currentMovementState = MOVEMENT_STATIC;
    }
    
    // Log perubahan state
    if (currentMovementState != previousState) {
      switch (currentMovementState) {
        case MOVEMENT_MOVING:
          LOG_INFO(MODULE_GPS, "🚗 Kendaraan mulai bergerak (%.1f km/h)", 
                   gpsManager.getSpeed());
          break;
        case MOVEMENT_STATIC:
          LOG_INFO(MODULE_GPS, "🛑 Kendaraan berhenti (%.1f km/h)", 
                   gpsManager.getSpeed());
          if (gpsManager.getDistanceFromLastPosition() > 0) {
            LOG_INFO(MODULE_GPS, "📏 Jarak dari posisi terakhir: %.1f meter", 
                     gpsManager.getDistanceFromLastPosition());
          }
          break;
        default:
          break;
      }
    }
    
    // Log periodik status movement (setiap 30 detik)
    if (millis() - lastMovementLog > 30000) {
      if (currentMovementState == MOVEMENT_MOVING) {
        LOG_DEBUG(MODULE_GPS, "Status: %s | Kecepatan: %.1f km/h | Heading: %.1f°",
                  getMovementString(currentMovementState),
                  gpsManager.getSpeed(),
                  gpsManager.getHeading());
      } else {
        LOG_DEBUG(MODULE_GPS, "Status: %s | Waktu diam: %lu detik",
                  getMovementString(currentMovementState),
                  gpsManager.getTimeSinceLastMovement() / 1000);
      }
      lastMovementLog = millis();
    }
  } else {
    currentMovementState = MOVEMENT_UNKNOWN;
  }
}

// Dapatkan interval GPS berdasarkan movement state
unsigned long getGpsIntervalForMovement() {
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    switch (currentMovementState) {
      case MOVEMENT_MOVING:
        return GPS_SEND_INTERVAL_MOVING;   // 1 detik saat bergerak
      case MOVEMENT_STATIC:
        return GPS_SEND_INTERVAL_STATIC;   // 10 detik saat diam
      default:
        return powerConfigs[currentPowerMode].gpsInterval; // Default
    }
  #else
    return powerConfigs[currentPowerMode].gpsInterval;
  #endif
}

// ----- WEBSOCKET STATISTICS -----
void printWebSocketStats() {
  LOG_INFO(MODULE_WS, wsManager.getPerformanceReport().c_str());
}

// ----- PERFORMANCE REPORT -----
void printPerformanceReport() {
  LOG_INFO(MODULE_PERF, "=== PERFORMANCE REPORT ===");
  LOG_INFO(MODULE_PERF, "Total Transmissions: %lu", performanceMetrics.totalTransmissions);
  LOG_INFO(MODULE_PERF, "Successful: %lu (%.1f%%)", 
           performanceMetrics.successfulTransmissions,
           (performanceMetrics.totalTransmissions > 0) ? 
           (performanceMetrics.successfulTransmissions * 100.0 / performanceMetrics.totalTransmissions) : 0);
  LOG_INFO(MODULE_PERF, "Failed: %lu", performanceMetrics.failedTransmissions);
  
  if (performanceMetrics.successfulTransmissions > 0) {
    unsigned long avgLatency = performanceMetrics.totalLatency / performanceMetrics.successfulTransmissions;
    LOG_INFO(MODULE_PERF, "Average Latency: %lu ms", avgLatency);
    LOG_INFO(MODULE_PERF, "Min Latency: %lu ms", performanceMetrics.minLatency);
    LOG_INFO(MODULE_PERF, "Max Latency: %lu ms", performanceMetrics.maxLatency);
    
    if (avgLatency <= MAX_ACCEPTABLE_LATENCY) {
      LOG_INFO(MODULE_PERF, "Performance: ✅ EXCELLENT (target: <%d ms)", MAX_ACCEPTABLE_LATENCY);
    } else {
      LOG_WARN(MODULE_PERF, "Performance: ⚠️ NEEDS IMPROVEMENT (target: <%d ms)", MAX_ACCEPTABLE_LATENCY);
    }
  }
  
  // Modem performance
  LOG_INFO(MODULE_PERF, modemManager.getPerformanceReport().c_str());
  
  // WebSocket performance
  LOG_INFO(MODULE_PERF, wsManager.getPerformanceReport().c_str());
}

// ----- PERINTAH SERIAL -----
void handleSerialCommands() {
  if (SerialMon.available()) {
    String cmd = SerialMon.readStringUntil('\n');
    cmd.trim();
    
    LOG_DEBUG(MODULE_MAIN, "Perintah diterima: %s", cmd.c_str());
    
    // Basic commands
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
      float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                         (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
      percentage = constrain(percentage, 0.0, 100.0);
      LOG_INFO(MODULE_MAIN, "Battery: %.2f V (%.0f%%) - %s", 
               batteryVoltage, percentage, Utils::getBatteryStatus(batteryVoltage));
    } else if (cmd.startsWith("setbat ")) {
      // Untuk testing: set tegangan baterai simulasi
      batteryVoltage = cmd.substring(7).toFloat();
      LOG_INFO(MODULE_MAIN, "Tegangan baterai diset ke: %.2f V", batteryVoltage);
    } else if (cmd == "memory") {
      Utils::printMemoryInfo();
    } 
    // Network commands
    else if (cmd == "network") {
      LOG_INFO(MODULE_MAIN, modemManager.getNetworkInfo().c_str());
    } else if (cmd == "sim") {
      const SimInfo& sim = modemManager.getSimInfo();
      LOG_INFO(MODULE_MAIN, "=== SIM CARD INFO ===");
      LOG_INFO(MODULE_MAIN, "SIM Ready: %s", sim.isReady ? "Ya" : "Tidak");
      LOG_INFO(MODULE_MAIN, "IMSI: %s", sim.imsi.c_str());
      LOG_INFO(MODULE_MAIN, "ICCID: %s", sim.iccid.c_str());
      if (sim.phoneNumber.length() > 0) {
        LOG_INFO(MODULE_MAIN, "Phone: %s", sim.phoneNumber.c_str());
      }
    } 
    // WebSocket commands
    else if (cmd == "wsstats") {
      printWebSocketStats();
    } else if (cmd == "wsreset") {
      LOG_INFO(MODULE_MAIN, "Mereset koneksi WebSocket...");
      wsManager.disconnect();
      delay(500); // Reduced delay
      wsManager.resetReconnectAttempts();
      if (powerConfigs[currentPowerMode].wsContinuous) {
        if (wsManager.connect()) {
          LOG_INFO(MODULE_MAIN, "✅ WebSocket berhasil direconnect");
        } else {
          LOG_ERROR(MODULE_MAIN, "❌ WebSocket gagal reconnect");
        }
      } else {
        LOG_INFO(MODULE_MAIN, "WebSocket dalam mode non-continuous");
      }
    } else if (cmd == "wsping") {
      // Force WebSocket ping untuk testing
      if (wsManager.isReady()) {
        wsManager.forcePing();
        LOG_INFO(MODULE_MAIN, "🏓 WebSocket ping sent");
      } else {
        LOG_WARN(MODULE_MAIN, "WebSocket tidak terhubung");
      }
    }
    // Relay commands
    else if (cmd == "on") {
      if (powerConfigs[currentPowerMode].relayEnabled) {
        onRelayUpdate(true);
      } else {
        LOG_WARN(MODULE_MAIN, "Kontrol relay dinonaktifkan pada mode %s", 
                 getPowerModeString(currentPowerMode));
      }
    } else if (cmd == "off") {
      if (powerConfigs[currentPowerMode].relayEnabled) {
        onRelayUpdate(false);
      } else {
        LOG_WARN(MODULE_MAIN, "Kontrol relay dinonaktifkan pada mode %s", 
                 getPowerModeString(currentPowerMode));
      }
    } 
    // System commands
    else if (cmd == "reset") {
      LOG_WARN(MODULE_MAIN, "Restart sistem diminta...");
      printPerformanceReport();
      delay(1000);
      ESP.restart();
    } else if (cmd == "loglevel") {
      LOG_INFO(MODULE_MAIN, "Level log saat ini: %d", Logger::getLevel());
    } else if (cmd.startsWith("loglevel ")) {
      int level = cmd.substring(9).toInt();
      if (level >= 0 && level <= 4) {
        Logger::setLevel((LogLevel)level);
      } else {
        LOG_WARN(MODULE_MAIN, "Level log tidak valid (0-4)");
      }
    } 
    // Movement commands
    else if (cmd == "movement") {
      LOG_INFO(MODULE_MAIN, "Movement State: %s", getMovementString(currentMovementState));
      if (gpsManager.isValid()) {
        LOG_INFO(MODULE_MAIN, "Kecepatan: %.1f km/h", gpsManager.getSpeed());
        LOG_INFO(MODULE_MAIN, "Jarak dari posisi terakhir: %.1f m", 
                 gpsManager.getDistanceFromLastPosition());
        LOG_INFO(MODULE_MAIN, "Waktu sejak movement terakhir: %lu detik", 
                 gpsManager.getTimeSinceLastMovement() / 1000);
      }
    } 
    // Performance/Optimization commands
    else if (cmd == "optimize") {
      LOG_INFO(MODULE_MAIN, "🔧 Memulai optimasi manual...");
      currentState = STATE_OPTIMIZING;
    } else if (cmd == "latency") {
      printPerformanceReport();
    } else if (cmd == "perftest") {
      // Test current configuration
      LOG_INFO(MODULE_MAIN, "=== PERFORMANCE TEST INFO ===");
      LOG_INFO(MODULE_MAIN, "Optimizations Applied: %s", 
               modemManager.areOptimizationsApplied() ? "YES" : "NO");
      LOG_INFO(MODULE_MAIN, "WebSocket Performance: %s",
               wsManager.isPerformanceGood() ? "GOOD" : "NEEDS IMPROVEMENT");
      LOG_INFO(MODULE_MAIN, "Payload Mode: %d (size: ~%d bytes)", 
               wsManager.getPayloadMode(),
               wsManager.getPayloadMode() == PAYLOAD_MODE_ESSENTIAL ? 140 :
               wsManager.getPayloadMode() == PAYLOAD_MODE_FULL ? 180 : 90);
      LOG_INFO(MODULE_MAIN, "GPS Interval: %lu ms", getGpsIntervalForMovement());
      
      NetworkOptimization netOpt = modemManager.getOptimizationStatus();
      LOG_INFO(MODULE_MAIN, "LTE-Only Mode: %s", netOpt.lteOnlyMode ? "YES" : "NO");
      LOG_INFO(MODULE_MAIN, "TCP Optimized: %s", netOpt.tcpOptimized ? "YES" : "NO");
      LOG_INFO(MODULE_MAIN, "Compression: %s", netOpt.compressionEnabled ? "YES" : "NO");
    } else if (cmd == "nettech") {
      // Get current network technology
      String tech = modemManager.getCurrentNetworkTechnology();
      LOG_INFO(MODULE_MAIN, "Network Technology: %s", tech.c_str());
      int band = modemManager.getBandInfo();
      if (band > 0) {
        LOG_INFO(MODULE_MAIN, "LTE Band: %d", band);
      }
    } else if (cmd == "resetperf") {
      // Reset performance statistics
      performanceMetrics = {0, 0, 0, 0, UINT32_MAX, 0, 0, 0, 0, 0};
      wsManager.resetPerformanceStats();
      modemManager.resetPerformanceStats();
      LOG_INFO(MODULE_MAIN, "✅ Performance statistics reset");
    } else if (cmd == "forceopt") {
      // Force reapply optimizations
      modemManager.forceOptimizationReapply();
      LOG_INFO(MODULE_MAIN, "✅ Optimizations reapplied");
    } else if (cmd == "testdata") {
      // Quick data connection test
      LOG_INFO(MODULE_MAIN, "Testing data connection...");
      if (modemManager.testDataConnection()) {
        LOG_INFO(MODULE_MAIN, "✅ Data connection test PASSED");
      } else {
        LOG_ERROR(MODULE_MAIN, "❌ Data connection test FAILED");
      }
    }
    // Testing commands
    else if (cmd == "send") {
      // Force immediate GPS data send for testing
      LOG_INFO(MODULE_MAIN, "🚀 Force sending GPS data...");
      if (sendVehicleDataViaWebSocket()) {
        LOG_INFO(MODULE_MAIN, "✅ Data sent successfully");
        // Print latency info
        if (powerConfigs[currentPowerMode].performanceMonitoring) {
          LOG_INFO(MODULE_MAIN, "WebSocket avg latency: %lu ms", wsManager.getAverageLatency());
          LOG_INFO(MODULE_MAIN, "Modem avg latency: %lu ms", modemManager.getAverageLatency());
        }
      } else {
        LOG_ERROR(MODULE_MAIN, "❌ Failed to send data");
      }
    } else if (cmd == "interval") {
      // Show current GPS interval
      LOG_INFO(MODULE_MAIN, "Current GPS interval: %lu ms", getGpsIntervalForMovement());
      LOG_INFO(MODULE_MAIN, "Movement state: %s", getMovementString(currentMovementState));
    } else if (cmd.startsWith("interval ")) {
      // Set custom GPS interval for testing
      unsigned long newInterval = cmd.substring(9).toInt();
      if (newInterval >= 1000 && newInterval <= 300000) {
        powerConfigs[currentPowerMode].gpsInterval = newInterval;
        LOG_INFO(MODULE_MAIN, "GPS interval set to: %lu ms", newInterval);
      } else {
        LOG_WARN(MODULE_MAIN, "Invalid interval (1000-300000 ms)");
      }
    } else if (cmd == "nowait") {
      // Toggle wait for subscription flag
      waitForSubscription = !waitForSubscription;
      LOG_INFO(MODULE_MAIN, "Wait for subscription: %s", waitForSubscription ? "ON" : "OFF");
    } else if (cmd == "move") {
      // Force movement state for testing
      currentMovementState = MOVEMENT_MOVING;
      LOG_INFO(MODULE_MAIN, "🚗 Forced movement state: MOVING");
      LOG_INFO(MODULE_MAIN, "GPS interval now: %lu ms", getGpsIntervalForMovement());
    } else if (cmd == "stop") {
      // Force static state for testing
      currentMovementState = MOVEMENT_STATIC;
      LOG_INFO(MODULE_MAIN, "🛑 Forced movement state: STATIC");
      LOG_INFO(MODULE_MAIN, "GPS interval now: %lu ms", getGpsIntervalForMovement());
    }
    // Debug commands
    else if (cmd.startsWith("at ")) {
      String atCmd = cmd.substring(3);
      modemManager.sendATCommand(atCmd);
      delay(500); // Reduced delay
      String response = modemManager.readATResponse(1000); // Reduced timeout
      LOG_INFO(MODULE_MAIN, "AT Response: %s", response.c_str());
    } else {
      LOG_WARN(MODULE_MAIN, "Perintah tidak dikenal: %s", cmd.c_str());
    }
    
    lastActivityTime = millis(); // Reset activity timer
  }
}

void printHelp() {
  SerialMon.println("\n========== BANTUAN PERINTAH ==========");
  SerialMon.println("=== BASIC COMMANDS ===");
  SerialMon.println("help         - Tampilkan bantuan ini");
  SerialMon.println("status       - Tampilkan status sistem lengkap");
  SerialMon.println("power        - Info mode power saat ini");
  SerialMon.println("full         - Beralih ke mode FULL (low latency)");
  SerialMon.println("standby      - Beralih ke mode STANDBY");
  SerialMon.println("emergency    - Beralih ke mode EMERGENCY");
  SerialMon.println("battery      - Tampilkan tegangan baterai");
  SerialMon.println("setbat X.X   - Set tegangan baterai (testing)");
  
  SerialMon.println("\n=== TESTING COMMANDS ===");
  SerialMon.println("send         - Force send GPS data immediately");
  SerialMon.println("interval     - Show current GPS interval");
  SerialMon.println("interval N   - Set GPS interval (1000-300000 ms)");
  SerialMon.println("nowait       - Toggle wait for WS subscription");
  SerialMon.println("move         - Force MOVING state");
  SerialMon.println("stop         - Force STATIC state");
  
  SerialMon.println("\n=== NETWORK COMMANDS ===");
  SerialMon.println("network      - Tampilkan info network lengkap");
  SerialMon.println("sim          - Tampilkan info SIM card");
  SerialMon.println("nettech      - Network technology saat ini");
  SerialMon.println("optimize     - Terapkan optimasi jaringan");
  SerialMon.println("forceopt     - Force reapply optimizations");
  SerialMon.println("testdata     - Test data connection");
  
  SerialMon.println("\n=== PERFORMANCE COMMANDS ===");
  SerialMon.println("latency      - Tampilkan laporan performance");
  SerialMon.println("perftest     - Info konfigurasi optimasi");
  SerialMon.println("resetperf    - Reset statistik performance");
  
  SerialMon.println("\n=== WEBSOCKET COMMANDS ===");
  SerialMon.println("wsstats      - Tampilkan WebSocket statistics");
  SerialMon.println("wsreset      - Reset koneksi WebSocket");
  SerialMon.println("wsping       - Send WebSocket ping");
  
  SerialMon.println("\n=== CONTROL COMMANDS ===");
  SerialMon.println("movement     - Tampilkan info pergerakan");
  SerialMon.println("on/off       - Kontrol relay");
  SerialMon.println("memory       - Tampilkan info memory");
  SerialMon.println("loglevel     - Tampilkan level log");
  SerialMon.println("loglevel N   - Set level log (0-4)");
  SerialMon.println("at <cmd>     - Kirim AT command");
  SerialMon.println("reset        - Restart sistem");
  
  #if TESTING_MODE
    SerialMon.println("\n⚠️ TESTING MODE ACTIVE - Sleep/Emergency disabled");
  #endif
  
  SerialMon.println("=====================================\n");
}

void printStatus() {
  SerialMon.println("\n========== STATUS SISTEM ==========");
  
  #if TESTING_MODE
    SerialMon.println("⚠️ TESTING MODE ACTIVE");
    SerialMon.printf("Wait for subscription: %s\n", waitForSubscription ? "ON" : "OFF");
  #endif
  
  SerialMon.printf("Mode Power   : %s", getPowerModeString(currentPowerMode));
  if (powerConfigs[currentPowerMode].performanceMonitoring) {
    SerialMon.print(" [PERF MON]");
  }
  if (powerConfigs[currentPowerMode].aggressiveOptimization) {
    SerialMon.print(" [AGGRESSIVE OPT]");
  }
  SerialMon.println();
  
  SerialMon.printf("State Sistem : %s\n", getStateString(currentState));
  SerialMon.printf("Modem Status : %s", modemManager.getStatusString());
  if (modemManager.areOptimizationsApplied()) {
    SerialMon.print(" [OPTIMIZED]");
  }
  SerialMon.println();
  
  if (modemManager.getResetRetries() > 0) {
    SerialMon.printf("Reset Count  : %d\n", modemManager.getResetRetries());
  }
  
  // Battery status
  float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                     (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  percentage = constrain(percentage, 0.0, 100.0);
  SerialMon.printf("Baterai      : %.2f V (%.0f%%) - %s\n", 
                   batteryVoltage, percentage, Utils::getBatteryStatus(batteryVoltage));
  
  SerialMon.printf("Uptime       : %s\n", Utils::formatUptime(millis()).c_str());
  
  // Status GPS
  if (gpsManager.isValid()) {
    SerialMon.printf("GPS          : Valid (%.6f, %.6f)\n",
                     gpsManager.getLatitude(), gpsManager.getLongitude());
    SerialMon.printf("Koordinat    : %s, %s\n",
                     Utils::formatCoordinate(gpsManager.getLatitude(), true).c_str(),
                     Utils::formatCoordinate(gpsManager.getLongitude(), false).c_str());
    SerialMon.printf("Kecepatan    : %.1f km/h\n", gpsManager.getSpeed());
    SerialMon.printf("Altitude     : %.1f m\n", gpsManager.getAltitude());
    SerialMon.printf("Heading      : %.1f°\n", gpsManager.getHeading());
    SerialMon.printf("Satelit      : %d\n", gpsManager.getSatellites());
    SerialMon.printf("HDOP         : %.1f (%s)\n", 
                     gpsManager.getHDOP(),
                     gpsManager.isHighAccuracy() ? "High Accuracy" : "Normal");
    SerialMon.printf("Movement     : %s\n", getMovementString(currentMovementState));
    
    if (currentMovementState == MOVEMENT_STATIC) {
      SerialMon.printf("Waktu Diam   : %lu detik\n", 
                       gpsManager.getTimeSinceLastMovement() / 1000);
    }
  } else {
    SerialMon.println("GPS          : Tidak ada fix");
    SerialMon.printf("Satelit      : %d\n", gpsManager.getSatellites());
  }
  
  // Status Relay
  SerialMon.printf("Relay        : %s\n", relayState ? "ON" : "OFF");
  
  // Status Koneksi
  SerialMon.printf("GPRS         : %s\n", 
                   modemManager.isGprsConnected() ? "Terhubung" : "Terputus");
  SerialMon.printf("WebSocket    : %s", wsManager.getStateString());
  
  // WebSocket performance inline
  if (wsManager.getState() != WS_DISCONNECTED) {
    const WSStats& stats = wsManager.getStats();
    SerialMon.printf(" (Msgs: %lu", stats.totalMessages);
    if (powerConfigs[currentPowerMode].performanceMonitoring) {
      unsigned long avgLatency = wsManager.getAverageLatency();
      if (avgLatency > 0) {
        SerialMon.printf(", Avg: %lums", avgLatency);
      }
    }
    SerialMon.print(")");
  }
  SerialMon.println();
  
  SerialMon.printf("Operator     : %s\n", modemManager.getOperator().c_str());
  SerialMon.printf("Signal       : %d (%s)\n", 
                   modemManager.getSignalQuality(),
                   Utils::getSignalQualityString(modemManager.getSignalQuality()));
                   
  // Memory Info
  SerialMon.printf("Free Memory  : %u KB (Min: %u KB)\n", 
                   Utils::getFreeHeap() / 1024, 
                   Utils::getMinFreeHeap() / 1024);
                   
  // GPS Interval info
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    if (currentPowerMode == POWER_MODE_FULL) {
      SerialMon.printf("GPS Interval : %lu ms (dynamic)\n", getGpsIntervalForMovement());
    } else {
      SerialMon.printf("GPS Interval : %lu ms\n", powerConfigs[currentPowerMode].gpsInterval);
    }
  #else
    SerialMon.printf("GPS Interval : %lu ms\n", powerConfigs[currentPowerMode].gpsInterval);
  #endif
  
  // Performance metrics summary
  if (performanceMetrics.totalTransmissions > 0) {
    SerialMon.printf("Performance  : %lu/%lu transmissions (%.1f%% success)\n",
                     performanceMetrics.successfulTransmissions,
                     performanceMetrics.totalTransmissions,
                     (performanceMetrics.successfulTransmissions * 100.0 / 
                      performanceMetrics.totalTransmissions));
    
    if (performanceMetrics.successfulTransmissions > 0) {
      unsigned long avgLatency = performanceMetrics.totalLatency / 
                                performanceMetrics.successfulTransmissions;
      SerialMon.printf("Avg Latency  : %lu ms ", avgLatency);
      if (avgLatency <= MAX_ACCEPTABLE_LATENCY) {
        SerialMon.print("✅");
      } else {
        SerialMon.print("⚠️");
      }
      SerialMon.println();
    }
  }
  
  SerialMon.println("===================================\n");
}

void printPowerModeInfo() {
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  SerialMon.println("\n====== INFO MODE POWER ======");
  SerialMon.printf("Mode         : %s\n", getPowerModeString(currentPowerMode));
  
  #if TESTING_MODE
    SerialMon.println("⚠️ TESTING MODE - Optimized for realtime");
  #endif
  
  SerialMon.printf("GPS Interval : %lu ms (%lu detik)\n", 
                   config.gpsInterval, config.gpsInterval/1000);
  
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    if (currentPowerMode == POWER_MODE_FULL) {
      SerialMon.println("             : Dynamic berdasarkan movement");
      SerialMon.printf("             : Moving: %d ms, Static: %d ms\n",
                       GPS_SEND_INTERVAL_MOVING, GPS_SEND_INTERVAL_STATIC);
    }
  #endif
  
  SerialMon.printf("WS KeepAlive : %lu ms\n", config.wsKeepAliveInterval);
  SerialMon.printf("Sleep Durasi : %lu ms", config.sleepDuration);
  if (config.sleepDuration == 0) {
    SerialMon.print(" (DISABLED)");
  }
  SerialMon.println();
  
  SerialMon.printf("GPS Always On: %s\n", config.gpsAlwaysOn ? "Ya" : "Tidak");
  SerialMon.printf("WS Continuous: %s\n", config.wsContinuous ? "Ya" : "Tidak");
  SerialMon.printf("Relay Enable : %s\n", config.relayEnabled ? "Ya" : "Tidak");
  SerialMon.printf("Perf Monitor : %s\n", config.performanceMonitoring ? "Ya" : "Tidak");
  SerialMon.printf("Aggressive   : %s\n", config.aggressiveOptimization ? "Ya" : "Tidak");
  
  SerialMon.println("\n=== OPTIMIZATION STATUS ===");
  SerialMon.printf("Target Latency: <%d ms\n", MAX_ACCEPTABLE_LATENCY);
  SerialMon.printf("Payload Mode  : %s (~%d bytes)\n",
                   DEFAULT_PAYLOAD_MODE == PAYLOAD_MODE_ESSENTIAL ? "ESSENTIAL" :
                   DEFAULT_PAYLOAD_MODE == PAYLOAD_MODE_FULL ? "FULL" : "MINIMAL",
                   DEFAULT_PAYLOAD_MODE == PAYLOAD_MODE_ESSENTIAL ? 140 :
                   DEFAULT_PAYLOAD_MODE == PAYLOAD_MODE_FULL ? 180 : 90);
  
  NetworkOptimization netOpt = modemManager.getOptimizationStatus();
  SerialMon.printf("Network Opt   : %s\n", modemManager.areOptimizationsApplied() ? "APPLIED" : "NOT APPLIED");
  if (modemManager.areOptimizationsApplied()) {
    SerialMon.printf("  LTE-Only    : %s\n", netOpt.lteOnlyMode ? "YES" : "NO");
    SerialMon.printf("  All Bands   : %s\n", netOpt.allBandsEnabled ? "YES" : "NO");
    SerialMon.printf("  TCP Optimized: %s\n", netOpt.tcpOptimized ? "YES" : "NO");
    SerialMon.printf("  Compression : %s\n", netOpt.compressionEnabled ? "YES" : "NO");
    SerialMon.printf("  Keep-Alive  : %s\n", netOpt.keepAliveConfigured ? "YES" : "NO");
  }
  
  #if TESTING_MODE
    SerialMon.println("\n=== TESTING PARAMETERS ===");
    SerialMon.printf("Battery Low   : %.1f V\n", BATTERY_LOW_THRESHOLD);
    SerialMon.printf("Activity TO   : %lu ms (%lu min)\n", 
                     ACTIVITY_TIMEOUT, ACTIVITY_TIMEOUT/60000);
    SerialMon.printf("WS Reconnect  : %d ms\n", WS_RECONNECT_DELAY);
    SerialMon.printf("Max Failures  : %d\n", MAX_CONNECTION_FAILURES);
  #endif
  
  SerialMon.println("=============================\n");
}