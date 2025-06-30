// ModemManager.cpp - Implementasi Manajer Modem
#include "ModemManager.h"

// Konstruktor
ModemManager::ModemManager(TinyGsm& modemInstance, HardwareSerial& serial)
  : modem(modemInstance),
    serialAT(serial),
    resetStartTime(0),
    resetStage(0),
    resetInProgress(false),
    resetRetries(0),
    currentStatus(MODEM_STATUS_OFF),
    lastStatusCheck(0),
    lastSignalQuality(99),
    lastOperator("Unknown") {
  
  // Inisialisasi SIM info
  simInfo.isReady = false;
  simInfo.imsi = "";
  simInfo.iccid = "";
  simInfo.phoneNumber = "";
}

// Inisialisasi hardware modem
void ModemManager::begin() {
  LOG_INFO(MODULE_MODEM, "Menginisialisasi modem GSM A7670C...");
  
  currentStatus = MODEM_STATUS_INITIALIZING;
  
  // Sequensi power on untuk A7670C
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  delay(100);
  digitalWrite(POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(POWER_PIN, LOW);
  
  // Tunggu modem siap (dengan watchdog feed)
  LOG_INFO(MODULE_MODEM, "Menunggu modem siap...");
  for (int i = 0; i < 5; i++) {
    Utils::safeDelay(1000);
    LOG_DEBUG(MODULE_MODEM, "Power on delay %d/5", i+1);
  }
  
  // Inisialisasi komunikasi serial
  serialAT.begin(MODEM_BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
  delay(500);
  
  LOG_INFO(MODULE_MODEM, "Hardware modem diinisialisasi");
}

// Tunggu response AT command
bool ModemManager::waitForATResponse(unsigned long timeout) {
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    if (serialAT.available()) {
      response += serialAT.readString();
      if (response.indexOf("OK") >= 0) {
        return true;
      }
      // Cek error response
      if (response.indexOf("ERROR") >= 0) {
        LOG_WARN(MODULE_MODEM, "AT command error: %s", response.c_str());
        return false;
      }
    }
    Utils::safeDelay(100);
  }
  
  LOG_DEBUG(MODULE_MODEM, "AT response timeout. Response: %s", response.c_str());
  return false;
}

// Tunggu koneksi network
bool ModemManager::waitForNetwork(unsigned long timeout) {
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (modem.isNetworkConnected()) {
      currentStatus = MODEM_STATUS_NETWORK_CONNECTED;
      return true;
    }
    Utils::safeDelay(500);
    
    // Update signal quality selama menunggu
    int csq = modem.getSignalQuality();
    if (csq != 99) {
      LOG_DEBUG(MODULE_MODEM, "Signal quality: %d", csq);
    }
  }
  
  return false;
}

// Cek status SIM card
bool ModemManager::checkSimCard() {
  LOG_DEBUG(MODULE_MODEM, "Memeriksa status SIM card...");
  
  serialAT.println("AT+CPIN?");
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < 5000) {
    if (serialAT.available()) {
      response += serialAT.readString();
      
      if (response.indexOf("+CPIN: READY") >= 0) {
        LOG_INFO(MODULE_MODEM, "✅ SIM card siap");
        simInfo.isReady = true;
        return true;
      } else if (response.indexOf("SIM failure") >= 0) {
        LOG_ERROR(MODULE_MODEM, "❌ SIM card gagal terdeteksi");
        LOG_INFO(MODULE_MODEM, "Tips: 1) Cek SIM terpasang benar 2) Bersihkan kontak SIM 3) Coba SIM lain");
        return false;
      } else if (response.indexOf("SIM PIN") >= 0) {
        LOG_ERROR(MODULE_MODEM, "❌ SIM memerlukan PIN");
        return false;
      } else if (response.indexOf("ERROR") >= 0) {
        LOG_ERROR(MODULE_MODEM, "❌ Error SIM: %s", response.c_str());
        return false;
      }
    }
    Utils::safeDelay(100);
  }
  
  LOG_ERROR(MODULE_MODEM, "⏱️ Timeout menunggu status SIM");
  return false;
}

// Setup lengkap modem
bool ModemManager::setup() {
  LOG_INFO(MODULE_MODEM, "Memulai setup modem...");
  
  // Clear buffer serial AT
  Utils::clearSerialBuffer(serialAT);
  
  // Test AT command dengan multiple attempts
  bool atResponse = false;
  for (int i = 0; i < 5; i++) {
    LOG_DEBUG(MODULE_MODEM, "AT command attempt %d/5", i+1);
    serialAT.println("AT");
    atResponse = waitForATResponse(3000);
    if (atResponse) break;
    Utils::safeDelay(1000 * (i+1)); // Delay incremental
  }
  
  if (!atResponse) {
    LOG_ERROR(MODULE_MODEM, "❌ Modem tidak merespon AT command");
    currentStatus = MODEM_STATUS_ERROR;
    return false;
  }
  
  LOG_DEBUG(MODULE_MODEM, "✅ Modem merespon AT command");
  
  // Cek SIM card
  if (!checkSimCard()) {
    currentStatus = MODEM_STATUS_ERROR;
    return false;
  }
  
  // Set full functionality
  serialAT.println("AT+CFUN=1");
  Utils::safeDelay(2000);
  Utils::clearSerialBuffer(serialAT);
  
  // Inisialisasi modem dengan TinyGSM
  if (!modem.init()) {
    LOG_ERROR(MODULE_MODEM, "❌ Gagal inisialisasi modem");
    currentStatus = MODEM_STATUS_ERROR;
    return false;
  }
  
  currentStatus = MODEM_STATUS_READY;
  
  // Get modem info
  String modemInfo = modem.getModemInfo();
  LOG_INFO(MODULE_MODEM, "Modem Info: %s", modemInfo.c_str());
  
  // Get IMEI
  String imei = modem.getIMEI();
  if (imei.length() > 0) {
    LOG_INFO(MODULE_MODEM, "IMEI: %s", imei.c_str());
  }
  
  // Update SIM info
  updateSimInfo();
  
  // Tunggu network
  LOG_INFO(MODULE_MODEM, "Menunggu koneksi jaringan...");
  
  bool networkConnected = Utils::retryOperation(
    MODULE_MODEM, 
    "koneksi jaringan",
    [this]() { return waitForNetwork(15000); },
    3, 
    3000
  );
  
  if (!networkConnected) {
    LOG_ERROR(MODULE_MODEM, "❌ Gagal terhubung ke jaringan");
    currentStatus = MODEM_STATUS_ERROR;
    return false;
  }
  
  LOG_INFO(MODULE_MODEM, "✅ Terhubung ke jaringan");
  
  // Update network info
  updateNetworkStatus();
  
  // Koneksi GPRS
  LOG_INFO(MODULE_MODEM, "Menghubungkan ke GPRS (APN: %s)...", APN);
  
  bool gprsConnected = Utils::retryOperation(
    MODULE_MODEM,
    "koneksi GPRS",
    [this]() {
      if (modem.isGprsConnected()) {
        modem.gprsDisconnect();
        Utils::safeDelay(1000);
      }
      return modem.gprsConnect(APN, "", "");
    },
    3,
    3000
  );
  
  if (!gprsConnected) {
    LOG_ERROR(MODULE_MODEM, "❌ Gagal koneksi GPRS");
    currentStatus = MODEM_STATUS_NETWORK_CONNECTED;
    return false;
  }
  
  currentStatus = MODEM_STATUS_GPRS_CONNECTED;
  LOG_INFO(MODULE_MODEM, "✅ GPRS terhubung");
  
  // Log final network info
  LOG_INFO(MODULE_MODEM, getNetworkInfo().c_str());
  
  return true;
}

// Pastikan koneksi GPRS aktif
bool ModemManager::ensureConnection() {
  if (!modem.isGprsConnected()) {
    LOG_WARN(MODULE_MODEM, "⚠️ GPRS terputus, mencoba reconnect...");
    
    // Disconnect dulu untuk clear state
    modem.gprsDisconnect();
    Utils::safeDelay(1000);
    
    // Coba reconnect
    bool reconnected = Utils::retryOperation(
      MODULE_MODEM,
      "GPRS reconnection",
      [this]() { return connectGprs(); },
      3,
      2000
    );
    
    if (!reconnected) {
      LOG_ERROR(MODULE_MODEM, "❌ Gagal reconnect GPRS");
      return false;
    }
    
    Utils::safeDelay(1000); // Stabilisasi
  }
  
  // Verifikasi dengan AT command
  LOG_DEBUG(MODULE_MODEM, "Verifikasi status GPRS...");
  serialAT.println("AT+CGACT?");
  delay(500);
  
  if (serialAT.available()) {
    String response = serialAT.readString();
    LOG_TRACE(MODULE_MODEM, "CGACT Response: %s", response.c_str());
    
    if (response.indexOf("+CGACT: 1,1") < 0) {
      LOG_WARN(MODULE_MODEM, "⚠️ PDP context tidak aktif, mengaktifkan...");
      modem.gprsDisconnect();
      Utils::safeDelay(1000);
      return connectGprs();
    }
  }
  
  return true;
}

// Mulai reset modem (non-blocking)
bool ModemManager::startReset() {
  LOG_INFO(MODULE_MODEM, "🔄 Memulai proses reset modem...");
  
  resetStage = 0;
  resetStartTime = millis();
  resetInProgress = true;
  
  // Disconnect GPRS jika masih terhubung
  if (modem.isGprsConnected()) {
    modem.gprsDisconnect();
  }
  
  // Coba shutdown modem dengan clean
  Utils::clearSerialBuffer(serialAT);
  serialAT.println("AT+CFUN=0"); // Set minimum functionality
  Utils::safeDelay(1000);
  
  currentStatus = MODEM_STATUS_INITIALIZING;
  
  return true;
}

// Lanjutkan proses reset (non-blocking)
bool ModemManager::continueReset() {
  if (!resetInProgress) {
    return false;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - resetStartTime;
  
  Utils::feedWatchdog();
  
  switch (resetStage) {
    case 0: // Power cycle start
      if (elapsedTime < 1500) {
        digitalWrite(POWER_PIN, HIGH);
      } else {
        digitalWrite(POWER_PIN, LOW);
        resetStage = 1;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 1: Power key HIGH");
      }
      break;
      
    case 1: // Wait 1.5 seconds
      if (elapsedTime >= 1500) {
        digitalWrite(POWER_PIN, HIGH);
        resetStage = 2;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 2: Power key LOW");
      }
      break;
      
    case 2: // Wait 1.5 more seconds
      if (elapsedTime >= 1500) {
        digitalWrite(POWER_PIN, LOW);
        resetStage = 3;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 3: Menunggu modem siap");
      }
      break;
      
    case 3: // Wait for modem ready
      if (elapsedTime >= 12000) { // 12 detik
        LOG_INFO(MODULE_MODEM, "Modem reset selesai, mencoba reinisialisasi");
        resetStage = 4;
      }
      break;
      
    case 4: // Test AT command
      Utils::clearSerialBuffer(serialAT);
      
      static int atAttempts = 0;
      serialAT.println("AT");
      
      if (waitForATResponse(1000)) {
        LOG_INFO(MODULE_MODEM, "✅ Modem merespon setelah reset");
        resetStage = 5;
        atAttempts = 0;
      } else {
        atAttempts++;
        if (atAttempts >= 5) {
          LOG_WARN(MODULE_MODEM, "⚠️ Modem belum respon setelah 5 percobaan");
          resetStage = 5;
          atAttempts = 0;
        } else {
          LOG_DEBUG(MODULE_MODEM, "Modem belum respon (attempt %d/5)...", atAttempts);
          Utils::safeDelay(1000 * atAttempts);
        }
      }
      break;
      
    case 5: // Stabilization commands
      LOG_DEBUG(MODULE_MODEM, "Mengirim command stabilisasi");
      Utils::clearSerialBuffer(serialAT);
      
      serialAT.println("AT+CFUN=1"); // Full functionality
      Utils::safeDelay(2000);
      serialAT.println("AT+CMEE=2"); // Verbose error
      Utils::safeDelay(500);
      serialAT.println("AT+CPIN?"); // Check SIM
      Utils::safeDelay(1000);
      
      resetStage = 6;
      resetStartTime = currentTime;
      break;
      
    case 6: // Try full setup
      if (elapsedTime >= 3000) {
        LOG_INFO(MODULE_MODEM, "Mencoba setup lengkap setelah reset");
        
        if (setup()) {
          resetRetries++;
          LOG_INFO(MODULE_MODEM, "✅ Reset modem berhasil (percobaan #%d)", resetRetries);
          resetInProgress = false;
          return false; // Reset complete
        } else {
          LOG_ERROR(MODULE_MODEM, "❌ Setup gagal setelah reset");
          
          if (resetRetries >= MAX_RESET_RETRIES) {
            LOG_ERROR(MODULE_MODEM, "❌ Melebihi maksimal reset (%d)", MAX_RESET_RETRIES);
            LOG_INFO(MODULE_MODEM, "Tips: 1) Cek power supply 2) Cek SIM card 3) Cek antenna 4) Cek wiring");
            resetInProgress = false;
            currentStatus = MODEM_STATUS_ERROR;
            return false;
          }
          
          resetRetries++;
          resetStage = 0; // Restart reset process
          resetStartTime = currentTime;
        }
      }
      break;
      
    default:
      LOG_ERROR(MODULE_MODEM, "Invalid reset stage: %d", resetStage);
      resetInProgress = false;
      return false;
  }
  
  return true; // Still in progress
}

// Update informasi SIM card
bool ModemManager::updateSimInfo() {
  // Get IMSI
  String imsi = modem.getIMSI();
  if (imsi.length() > 0) {
    simInfo.imsi = imsi;
    LOG_DEBUG(MODULE_MODEM, "IMSI: %s", imsi.c_str());
  }
  
  // Get ICCID
  String iccid = modem.getSimCCID();
  if (iccid.length() > 0) {
    simInfo.iccid = iccid;
    LOG_DEBUG(MODULE_MODEM, "ICCID: %s", iccid.c_str());
  }
  
  // Get phone number (tidak selalu tersedia)
  serialAT.println("AT+CNUM");
  delay(500);
  if (serialAT.available()) {
    String response = serialAT.readString();
    int idx = response.indexOf("+CNUM:");
    if (idx >= 0) {
      int start = response.indexOf("\"", idx) + 1;
      int end = response.indexOf("\"", start);
      if (start > 0 && end > start) {
        simInfo.phoneNumber = response.substring(start, end);
        LOG_DEBUG(MODULE_MODEM, "Phone: %s", simInfo.phoneNumber.c_str());
      }
    }
  }
  
  return simInfo.isReady;
}

// Update status network
void ModemManager::updateNetworkStatus() {
  // Update signal quality
  lastSignalQuality = modem.getSignalQuality();
  
  // Update operator
  lastOperator = modem.getOperator();
  
  lastStatusCheck = millis();
}

// Get signal quality dengan caching
int ModemManager::getSignalQuality() {
  // Update jika data lama (> 30 detik)
  if (millis() - lastStatusCheck > 30000) {
    updateNetworkStatus();
  }
  return lastSignalQuality;
}

// Get operator dengan caching
String ModemManager::getOperator() {
  // Update jika data lama (> 30 detik)
  if (millis() - lastStatusCheck > 30000) {
    updateNetworkStatus();
  }
  return lastOperator;
}

// Get status string
const char* ModemManager::getStatusString() const {
  switch (currentStatus) {
    case MODEM_STATUS_OFF: return "OFF";
    case MODEM_STATUS_INITIALIZING: return "INITIALIZING";
    case MODEM_STATUS_READY: return "READY";
    case MODEM_STATUS_NETWORK_CONNECTED: return "NETWORK_CONNECTED";
    case MODEM_STATUS_GPRS_CONNECTED: return "GPRS_CONNECTED";
    case MODEM_STATUS_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// Get formatted network info
String ModemManager::getNetworkInfo() {
  updateNetworkStatus();
  
  String info = "=== NETWORK INFO ===\n";
  info += "Status: " + String(getStatusString()) + "\n";
  info += "Operator: " + lastOperator + "\n";
  info += "Signal: " + String(lastSignalQuality) + " (" + Utils::getSignalQualityString(lastSignalQuality) + ")\n";
  
  if (simInfo.isReady) {
    info += "IMSI: " + simInfo.imsi + "\n";
    if (simInfo.phoneNumber.length() > 0) {
      info += "Phone: " + simInfo.phoneNumber + "\n";
    }
  }
  
  return info;
}

// Send AT command untuk debugging
void ModemManager::sendATCommand(const String& command) {
  LOG_DEBUG(MODULE_MODEM, "AT Command: %s", command.c_str());
  serialAT.println(command);
}

// Read AT response untuk debugging
String ModemManager::readATResponse(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (serialAT.available()) {
      response += serialAT.readString();
    }
    delay(10);
  }
  
  return response;
}