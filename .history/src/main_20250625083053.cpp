// ========================================
// Main.cpp - Sistem GPS Tracker dengan Power Management
// ========================================

/**
 * ESP32 Vehicle GPS Tracking dengan Mode Power Management
 * - Mode Aktif Penuh: Real-time tracking
 * - Mode Sleep/Standby: Power saving dengan update periodik
 * - Mode Darurat: Minimal power untuk kondisi baterai rendah
 * 
 * Versi: 5.2 - Enhanced Edition
 * Update: Integrasi penuh dengan ModemManager yang diperbaiki
 */

// ----- FRAMEWORK ARDUINO -----
#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>

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
  POWER_MODE_FULL,      // Mode Aktif Penuh - tracking realtime
  POWER_MODE_STANDBY,   // Mode Standby - hemat daya
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
  STATE_SLEEPING             // Dalam mode sleep
};

// ----- STATE PERGERAKAN -----
enum MovementState {
  MOVEMENT_UNKNOWN,   // Status belum diketahui
  MOVEMENT_STATIC,    // Kendaraan diam
  MOVEMENT_MOVING     // Kendaraan bergerak
};

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
bool relayState = true;

// ----- KONFIGURASI MODE POWER -----
struct PowerModeConfig {
  unsigned long gpsInterval;        // Interval update GPS
  unsigned long wsKeepAliveInterval; // Interval keepalive WebSocket
  unsigned long sleepDuration;      // Durasi sleep mode
  bool gpsAlwaysOn;                // GPS selalu aktif
  bool wsContinuous;               // WebSocket selalu terhubung
  bool relayEnabled;               // Kontrol relay diizinkan
};

// Konfigurasi untuk setiap mode power
PowerModeConfig powerConfigs[3] = {
  // POWER_MODE_FULL - Mode penuh untuk tracking realtime
  {
    .gpsInterval = 10000,           // 10 detik
    .wsKeepAliveInterval = 45000,   // 45 detik
    .sleepDuration = 0,             // Tidak ada sleep
    .gpsAlwaysOn = true,
    .wsContinuous = true,
    .relayEnabled = true
  },
  // POWER_MODE_STANDBY - Mode hemat dengan update berkala
  {
    .gpsInterval = 300000,          // 5 menit
    .wsKeepAliveInterval = 120000,  // 2 menit
    .sleepDuration = 240000,        // 4 menit light sleep
    .gpsAlwaysOn = false,
    .wsContinuous = false,
    .relayEnabled = true
  },
  // POWER_MODE_EMERGENCY - Mode darurat ultra hemat
  {
    .gpsInterval = 1800000,         // 30 menit
    .wsKeepAliveInterval = 0,       // Tidak ada keepalive
    .sleepDuration = 1740000,       // 29 menit deep sleep
    .gpsAlwaysOn = false,
    .wsContinuous = false,
    .relayEnabled = false
  }
};

// Tegangan baterai saat ini (simulasi untuk testing)
float batteryVoltage = 12.6;

// ----- PROTOTYPE FUNGSI -----
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
void updateMovementState();
void checkSignalQuality();
unsigned long getGpsIntervalForMovement();
const char* getPowerModeString(PowerMode mode);
const char* getStateString(SystemState state);
const char* getMovementString(MovementState movement);

// ----- SETUP -----
void setup() {
  // Inisialisasi Serial
  SerialMon.begin(115200);
  delay(100);
  
  // Inisialisasi Logger dengan level INFO untuk produksi
  #ifdef DEBUG_MODE
    Logger::init(&SerialMon, LOG_DEBUG);
  #else
    Logger::init(&SerialMon, LOG_INFO);
  #endif
  
  LOG_INFO(MODULE_MAIN, "=== ESP32 GPS Tracker dengan Power Management ===");
  LOG_INFO(MODULE_MAIN, "Versi: 5.2 (Enhanced Edition)");
  LOG_INFO(MODULE_MAIN, "Mode Power: %s", getPowerModeString(currentPowerMode));
  LOG_INFO(MODULE_MAIN, "Device ID: %s", GPS_ID);
  
  // Info tentang dynamic GPS interval jika aktif
  #ifdef ENABLE_DYNAMIC_GPS_INTERVAL
    LOG_INFO(MODULE_MAIN, "Dynamic GPS Interval: AKTIF");
    LOG_INFO(MODULE_MAIN, "- Bergerak: %d detik", GPS_SEND_INTERVAL_MOVING/1000);
    LOG_INFO(MODULE_MAIN, "- Diam: %d detik", GPS_SEND_INTERVAL_STATIC/1000);
  #endif
  
  // Inisialisasi watchdog
  Utils::initWatchdog(WATCHDOG_TIMEOUT);
  LOG_INFO(MODULE_SYS, "Watchdog timer diaktifkan: %d detik", WATCHDOG_TIMEOUT/1000);
  
  // Inisialisasi hardware
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON);
  LOG_INFO(MODULE_RELAY, "Relay diinisialisasi: ON");
  
  // Inisialisasi pin ADC baterai jika ada
  // pinMode(BATTERY_ADC_PIN, INPUT);
  
  // Inisialisasi manajer
  LOG_INFO(MODULE_MAIN, "Menginisialisasi modul...");
  gpsManager.begin();
  
  // Enable high update rate GPS jika didukung
  #ifdef GPS_UPDATE_RATE
    gpsManager.enableHighUpdateRate(GPS_UPDATE_RATE);
    LOG_INFO(MODULE_GPS, "GPS update rate diset ke %d Hz", GPS_UPDATE_RATE);
  #endif
  
  modemManager.begin();
  wsManager.begin();
  wsManager.setOnRelayUpdate(onRelayUpdate);
  
  // Tampilkan bantuan awal
  printHelp();
  printPowerModeInfo();
  
  // Set state awal
  currentState = STATE_INIT;
  lastSuccessfulOperation = millis();
  lastActivityTime = millis();
  lastSignalCheck = millis();
  
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
  
  // Monitor kualitas sinyal
  checkSignalQuality();
  
  // Update GPS berdasarkan konfigurasi mode power
  if (powerConfigs[currentPowerMode].gpsAlwaysOn || 
      (millis() - lastGpsSendTime > powerConfigs[currentPowerMode].gpsInterval - 30000)) {
    gpsManager.update();
    
    // Update movement state setelah GPS update
    updateMovementState();
  }
  
  // Update WebSocket jika dalam mode continuous
  if (currentState == STATE_OPERATIONAL && powerConfigs[currentPowerMode].wsContinuous) {
    wsManager.update();
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
  
  delay(10);
}

// ----- HANDLER STATE -----
void handleInitState() {
  LOG_INFO(MODULE_SYS, "Menginisialisasi sistem...");
  
  if (modemManager.setup()) {
    LOG_INFO(MODULE_SYS, "✅ Inisialisasi sistem berhasil");
    
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
    LOG_DEBUG(MODULE_GPS, "Waktu untuk mengirim data GPS (interval: %lu ms)", gpsInterval);
    
    if (sendVehicleDataViaWebSocket()) {
      lastGpsSendTime = currentTime;
      lastSuccessfulOperation = currentTime;
      lastActivityTime = currentTime;
    }
  }
  
  // Masuk mode sleep jika dikonfigurasi dan tidak ada aktivitas
  if (config.sleepDuration > 0 && 
      currentTime - lastActivityTime > 60000) { // 1 menit tidak aktif
    LOG_INFO(MODULE_SYS, "Tidak ada aktivitas, bersiap untuk sleep");
    currentState = STATE_SLEEP_PREPARE;
  }
  
  // Cek kesehatan koneksi
  if (currentTime - lastSuccessfulOperation > 300000) { // 5 menit tanpa sukses
    LOG_WARN(MODULE_SYS, "⚠️ Tidak ada operasi berhasil dalam 5 menit, memulai recovery");
    currentState = STATE_CONNECTION_RECOVERY;
  }
}

void handleSleepPrepareState() {
  LOG_INFO(MODULE_SYS, "Mempersiapkan mode sleep...");
  
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
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
  LOG_INFO(MODULE_SYS, "Bangun dari sleep mode");
  enablePeripherals();
  currentState = STATE_OPERATIONAL;
  lastActivityTime = millis();
}

void handleModemResetState() {
  if (!modemManager.continueReset()) {
    // Reset selesai, coba setup lagi
    if (modemManager.setup()) {
      LOG_INFO(MODULE_SYS, "✅ Reset modem berhasil");
      currentState = STATE_OPERATIONAL;
    } else {
      LOG_ERROR(MODULE_SYS, "❌ Reset modem gagal");
      currentState = STATE_ERROR;
    }
  }
}

void handleConnectionRecoveryState() {
  LOG_INFO(MODULE_SYS, "Mencoba pemulihan koneksi...");
  
  // Disconnect semua koneksi
  wsManager.disconnect();
  modemManager.disconnectGprs();
  Utils::safeDelay(2000);
  
  // Coba koneksi ulang GPRS
  if (modemManager.connectGprs()) {
    LOG_INFO(MODULE_SYS, "✅ Pemulihan GPRS berhasil");
    
    // Reconnect WebSocket jika mode continuous
    if (powerConfigs[currentPowerMode].wsContinuous) {
      if (wsManager.connect()) {
        LOG_INFO(MODULE_SYS, "✅ WebSocket terhubung kembali");
        currentState = STATE_OPERATIONAL;
        lastSuccessfulOperation = millis();
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

// ----- MONITORING SINYAL -----
void checkSignalQuality() {
  // Cek sinyal setiap menit
  if (millis() - lastSignalCheck > 60000) {
    int signal = modemManager.getSignalQuality();
    
    if (modemManager.isSignalWeak()) {
      LOG_WARN(MODULE_MODEM, "⚠️ Sinyal lemah: %d (%s)", 
               signal, Utils::getSignalQualityString(signal));
      
      // Jika sinyal terlalu lemah, mungkin perlu recovery
      if (signal < 5 && currentState == STATE_OPERATIONAL) {
        LOG_WARN(MODULE_MODEM, "⚠️ Sinyal sangat lemah, monitoring koneksi ketat");
      }
    } else {
      LOG_DEBUG(MODULE_MODEM, "Signal quality: %d (%s)", 
                signal, Utils::getSignalQualityString(signal));
    }
    
    lastSignalCheck = millis();
  }
}

// ----- MANAJEMEN MODE POWER -----
void setPowerMode(PowerMode mode) {
  if (mode == currentPowerMode) return;
  
  LOG_INFO(MODULE_SYS, "Mengganti mode power: %s → %s", 
           getPowerModeString(currentPowerMode), 
           getPowerModeString(mode));
  
  currentPowerMode = mode;
  PowerModeConfig& config = powerConfigs[mode];
  
  // Sesuaikan konfigurasi WebSocket
  if (config.wsContinuous) {
    LOG_INFO(MODULE_WS, "WebSocket keepalive: %lu ms", config.wsKeepAliveInterval);
  }
  
  // Perkiraan konsumsi daya
  float expectedCurrent = 0;
  switch(mode) {
    case POWER_MODE_FULL:
      expectedCurrent = 250; // ~250mA rata-rata
      break;
    case POWER_MODE_STANDBY:
      expectedCurrent = 80;  // ~80mA rata-rata
      break;
    case POWER_MODE_EMERGENCY:
      expectedCurrent = 25;  // ~25mA rata-rata
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
  
  // Set GPIO yang tidak digunakan ke input dengan pulldown
  // (Tambahkan konfigurasi GPIO spesifik sesuai hardware)
  
  LOG_DEBUG(MODULE_SYS, "Peripheral dinonaktifkan untuk hemat daya");
}

void enablePeripherals() {
  // Re-enable peripheral yang diperlukan setelah sleep
  LOG_DEBUG(MODULE_SYS, "Peripheral diaktifkan kembali");
}

// ----- FUNGSI BATERAI -----
float readBatteryVoltage() {
  // Implementasi pembacaan tegangan baterai
  // Ini adalah placeholder - implementasi sesuai hardware
  /*
  int adcValue = analogRead(BATTERY_ADC_PIN);
  float voltage = (adcValue / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
  return voltage;
  */
  
  // Untuk testing, return nilai simulasi
  return batteryVoltage;
}

void updateBatteryStatus() {
  static unsigned long lastBatteryCheck = 0;
  
  if (millis() - lastBatteryCheck > 60000) { // Cek setiap menit
    batteryVoltage = readBatteryVoltage();
    lastBatteryCheck = millis();
    
    LOG_DEBUG(MODULE_SYS, "Tegangan baterai: %.2f V", batteryVoltage);
  }
}

void checkEmergencyMode() {
  // Auto-switch ke mode darurat jika baterai rendah
  if (batteryVoltage < 11.5 && currentPowerMode != POWER_MODE_EMERGENCY) {
    LOG_WARN(MODULE_SYS, "⚠️ Baterai rendah terdeteksi! Beralih ke mode darurat");
    setPowerMode(POWER_MODE_EMERGENCY);
  }
  
  // Kembali ke mode standby jika baterai pulih
  if (batteryVoltage > 12.0 && currentPowerMode == POWER_MODE_EMERGENCY) {
    LOG_INFO(MODULE_SYS, "✅ Baterai pulih, beralih ke mode standby");
    setPowerMode(POWER_MODE_STANDBY);
  }
}

// ----- FUNGSI PENGIRIMAN DATA -----
bool sendVehicleDataViaWebSocket() {
  // Cek koneksi GPRS
  if (!modemManager.ensureConnection()) {
    LOG_ERROR(MODULE_GPS, "Tidak ada koneksi GPRS");
    return false;
  }
  
  // Koneksi WebSocket jika diperlukan (mode non-continuous)
  if (!powerConfigs[currentPowerMode].wsContinuous) {
    if (!wsManager.isReady()) {
      LOG_INFO(MODULE_WS, "Menghubungkan WebSocket untuk transmisi data...");
      if (!wsManager.connect()) {
        LOG_ERROR(MODULE_WS, "Gagal menghubungkan WebSocket");
        return false;
      }
      delay(2000); // Tunggu koneksi stabil
    }
  }
  
  LOG_INFO(MODULE_GPS, "Mengirim data kendaraan [mode %s]...", 
           getPowerModeString(currentPowerMode));
  
  // Siapkan timestamp
  char timestamp[30];
  gpsManager.getTimestamp(timestamp, sizeof(timestamp));
  
  // Konversi ke format ISO
  String isoTimestamp = String(timestamp);
  if (isoTimestamp.endsWith("Z")) {
    isoTimestamp.replace("Z", ".000Z");
  }
  
  bool success = false;
  
  // Kirim data jika GPS valid
  if (gpsManager.isValid()) {
    success = wsManager.sendVehicleData(
      gpsManager.getLatitude(),
      gpsManager.getLongitude(),
      gpsManager.getSpeed(),
      gpsManager.getSatellites(),
      isoTimestamp
    );
    
    if (success) {
      LOG_INFO(MODULE_GPS, "✅ Data kendaraan berhasil dikirim");
      LOG_INFO(MODULE_GPS, "📍 Posisi: %.6f, %.6f | 🚗 %.1f km/h | 🛰️ %d satelit",
               gpsManager.getLatitude(), gpsManager.getLongitude(),
               gpsManager.getSpeed(), gpsManager.getSatellites());
      LOG_DEBUG(MODULE_GPS, "📏 Altitude: %.1f m | 🧭 Heading: %.1f° | HDOP: %.1f",
                gpsManager.getAltitude(), gpsManager.getHeading(), gpsManager.getHDOP());
      
      // Log jika ada fix baru
      if (gpsManager.hasNewFix()) {
        LOG_INFO(MODULE_GPS, "🆕 GPS fix baru terdeteksi");
      }
    }
  } else {
    LOG_WARN(MODULE_GPS, "⚠️ Data GPS tidak valid, menunggu fix...");
    LOG_DEBUG(MODULE_GPS, "Satelit: %d, HDOP: %.1f", 
              gpsManager.getSatellites(), gpsManager.getHDOP());
  }
  
  // Putuskan WebSocket jika mode non-continuous
  if (!powerConfigs[currentPowerMode].wsContinuous && success) {
    delay(1000); // Tunggu data terkirim
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

// ----- PERINTAH SERIAL -----
void handleSerialCommands() {
  if (SerialMon.available()) {
    String cmd = SerialMon.readStringUntil('\n');
    cmd.trim();
    
    LOG_DEBUG(MODULE_MAIN, "Perintah diterima: %s", cmd.c_str());
    
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
      LOG_INFO(MODULE_MAIN, "Tegangan baterai: %.2f V (%s)", 
               batteryVoltage, Utils::getBatteryStatus(batteryVoltage));
    } else if (cmd.startsWith("setbat ")) {
      // Untuk testing: set tegangan baterai simulasi
      batteryVoltage = cmd.substring(7).toFloat();
      LOG_INFO(MODULE_MAIN, "Tegangan baterai diset ke: %.2f V", batteryVoltage);
    } else if (cmd == "memory") {
      Utils::printMemoryInfo();
    } else if (cmd == "network") {
      // Tampilkan info network
      LOG_INFO(MODULE_MAIN, modemManager.getNetworkInfo().c_str());
    } else if (cmd == "sim") {
      // Tampilkan info SIM card
      const SimInfo& sim = modemManager.getSimInfo();
      LOG_INFO(MODULE_MAIN, "=== SIM CARD INFO ===");
      LOG_INFO(MODULE_MAIN, "SIM Ready: %s", sim.isReady ? "Ya" : "Tidak");
      LOG_INFO(MODULE_MAIN, "IMSI: %s", sim.imsi.c_str());
      LOG_INFO(MODULE_MAIN, "ICCID: %s", sim.iccid.c_str());
      if (sim.phoneNumber.length() > 0) {
        LOG_INFO(MODULE_MAIN, "Phone: %s", sim.phoneNumber.c_str());
      }
    } else if (cmd == "on") {
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
    } else if (cmd == "reset") {
      LOG_WARN(MODULE_MAIN, "Restart sistem diminta...");
      delay(1000);
      ESP.restart();
    } else if (cmd == "loglevel") {
      // Tampilkan level log saat ini
      LOG_INFO(MODULE_MAIN, "Level log saat ini: %d", Logger::getLevel());
    } else if (cmd.startsWith("loglevel ")) {
      // Set level log
      int level = cmd.substring(9).toInt();
      if (level >= 0 && level <= 4) {
        Logger::setLevel((LogLevel)level);
      } else {
        LOG_WARN(MODULE_MAIN, "Level log tidak valid (0-4)");
      }
    } else if (cmd == "movement") {
      // Tampilkan info movement
      LOG_INFO(MODULE_MAIN, "Movement State: %s", getMovementString(currentMovementState));
      if (gpsManager.isValid()) {
        LOG_INFO(MODULE_MAIN, "Kecepatan: %.1f km/h", gpsManager.getSpeed());
        LOG_INFO(MODULE_MAIN, "Jarak dari posisi terakhir: %.1f m", 
                 gpsManager.getDistanceFromLastPosition());
        LOG_INFO(MODULE_MAIN, "Waktu sejak movement terakhir: %lu detik", 
                 gpsManager.getTimeSinceLastMovement() / 1000);
      }
    } else if (cmd.startsWith("at ")) {
      // Kirim AT command langsung untuk debugging
      String atCmd = cmd.substring(3);
      modemManager.sendATCommand(atCmd);
      delay(1000);
      String response = modemManager.readATResponse(2000);
      LOG_INFO(MODULE_MAIN, "AT Response: %s", response.c_str());
    } else {
      LOG_WARN(MODULE_MAIN, "Perintah tidak dikenal: %s", cmd.c_str());
    }
    
    lastActivityTime = millis(); // Reset activity timer
  }
}

void printHelp() {
  SerialMon.println("\n========== BANTUAN PERINTAH ==========");
  SerialMon.println("help         - Tampilkan bantuan ini");
  SerialMon.println("status       - Tampilkan status sistem");
  SerialMon.println("power        - Info mode power saat ini");
  SerialMon.println("full         - Beralih ke mode FULL");
  SerialMon.println("standby      - Beralih ke mode STANDBY");
  SerialMon.println("emergency    - Beralih ke mode EMERGENCY");
  SerialMon.println("battery      - Tampilkan tegangan baterai");
  SerialMon.println("setbat X.X   - Set tegangan baterai (testing)");
  SerialMon.println("memory       - Tampilkan info memory");
  SerialMon.println("network      - Tampilkan info network");
  SerialMon.println("sim          - Tampilkan info SIM card");
  SerialMon.println("movement     - Tampilkan info pergerakan");
  SerialMon.println("on/off       - Kontrol relay");
  SerialMon.println("loglevel     - Tampilkan level log");
  SerialMon.println("loglevel N   - Set level log (0-4)");
  SerialMon.println("at <cmd>     - Kirim AT command");
  SerialMon.println("reset        - Restart sistem");
  SerialMon.println("=====================================\n");
}

void printStatus() {
  SerialMon.println("\n========== STATUS SISTEM ==========");
  SerialMon.printf("Mode Power   : %s\n", getPowerModeString(currentPowerMode));
  SerialMon.printf("State Sistem : %s\n", getStateString(currentState));
  SerialMon.printf("Modem Status : %s\n", modemManager.getStatusString());
  if (modemManager.getResetRetries() > 0) {
    SerialMon.printf("Reset Count  : %d\n", modemManager.getResetRetries());
  }
  SerialMon.printf("Baterai      : %.2f V (%s)\n", 
                   batteryVoltage, Utils::getBatteryStatus(batteryVoltage));
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
  SerialMon.printf("WebSocket    : %s\n", wsManager.getStateString());
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
    }
  #endif
  
  SerialMon.println("===================================\n");
}

void printPowerModeInfo() {
  PowerModeConfig& config = powerConfigs[currentPowerMode];
  
  SerialMon.println("\n====== INFO MODE POWER ======");
  SerialMon.printf("Mode         : %s\n", getPowerModeString(currentPowerMode));
  SerialMon.printf("GPS Interval : %lu ms (%lu detik)\n", 
                   config.gpsInterval, config.gpsInterval/1000);
  SerialMon.printf("WS KeepAlive : %lu ms\n", config.wsKeepAliveInterval);
  SerialMon.printf("Sleep Durasi : %lu ms\n", config.sleepDuration);
  SerialMon.printf("GPS Always On: %s\n", config.gpsAlwaysOn ? "Ya" : "Tidak");
  SerialMon.printf("WS Continuous: %s\n", config.wsContinuous ? "Ya" : "Tidak");
  SerialMon.printf("Relay Enable : %s\n", config.relayEnabled ? "Ya" : "Tidak");
  SerialMon.println("=============================\n");
}