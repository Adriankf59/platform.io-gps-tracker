// ModemManager.cpp - Implementasi Manajer Modem (Optimized for Low Latency)
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
    lastOperator("Unknown"),
    optimizationsApplied(false) {
  
  // Inisialisasi SIM info
  simInfo.isReady = false;
  simInfo.imsi = "";
  simInfo.iccid = "";
  simInfo.phoneNumber = "";
  
  // Inisialisasi optimization status
  netOptStatus = {false, false, false, false, false, 0};
  
  // Inisialisasi performance stats
  perfStats = {0, 0, UINT32_MAX, 0, 0, false};
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
  
  // Inisialisasi komunikasi serial dengan buffer optimization
  serialAT.begin(MODEM_BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
  serialAT.setRxBufferSize(2048); // Increase RX buffer for better performance
  delay(500);
  
  LOG_INFO(MODULE_MODEM, "Hardware modem diinisialisasi");
}

// Apply network optimizations for low latency
bool ModemManager::applyNetworkOptimizations() {
  if (optimizationsApplied) {
    return true;
  }
  
  LOG_INFO(MODULE_MODEM, "🚀 Menerapkan optimasi jaringan untuk latensi rendah...");
  
  // 1. Force LTE-Only Mode
  LOG_DEBUG(MODULE_MODEM, "Setting LTE-only mode...");
  sendATCommand("AT+CNMP=38");
  if (!waitForATResponse(3000)) {
    LOG_WARN(MODULE_MODEM, "⚠️ Gagal set LTE-only mode");
  } else {
    LOG_INFO(MODULE_MODEM, "✅ LTE-only mode diaktifkan");
    netOptStatus.lteOnlyMode = true;
  }
  
  Utils::safeDelay(1000);
  
  // 2. Enable All LTE Bands
  LOG_DEBUG(MODULE_MODEM, "Enabling all LTE bands...");
  sendATCommand("AT+CBAND=\"ALL\"");
  if (!waitForATResponse(2000)) {
    LOG_WARN(MODULE_MODEM, "⚠️ Gagal enable all LTE bands");
  } else {
    LOG_INFO(MODULE_MODEM, "✅ All LTE bands enabled");
    netOptStatus.allBandsEnabled = true;
  }
  
  Utils::safeDelay(500);
  
  // 3. Optimize TCP Settings for Low Latency
  LOG_DEBUG(MODULE_MODEM, "Optimizing TCP settings...");
  
  // Enable TCP Fast Open dan Quick ACK
  sendATCommand("AT+CIPCCFG=10,0,0,0,1,0,1200"); // Smaller MSS for faster transmission
  if (waitForATResponse(2000)) {
    netOptStatus.tcpOptimized = true;
  }
  
  // Set TCP keep-alive for persistent connections
  sendATCommand("AT+CIPCCFG=3,7200,75,9"); // Keep-alive: 2h interval, 75s probe, 9 retries
  if (waitForATResponse(2000)) {
    netOptStatus.keepAliveConfigured = true;
  }
  
  // 4. Optimize GPRS Settings
  LOG_DEBUG(MODULE_MODEM, "Optimizing GPRS settings...");
  
  // Set faster PDP context activation
  sendATCommand("AT+CGQREQ=1,2,4,3,6,31"); // QoS profile for low latency
  waitForATResponse(2000);
  
  // Enable header compression
  sendATCommand("AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\",\"0.0.0.0\",0,1"); // Enable header compression
  if (waitForATResponse(2000)) {
    netOptStatus.compressionEnabled = true;
  }
  
  // 5. Set Network Registration for Fast Handover
  sendATCommand("AT+CGREG=2"); // Enable network registration with location info
  waitForATResponse(1000);
  
  // 6. Optimize Power Management for Performance
  sendATCommand("AT+CPSMS=0"); // Disable PSM for always-on connectivity
  waitForATResponse(1000);
  
  // 7. Set Fast Dormancy Disabled for Always-Active Data
  sendATCommand("AT+CFGRI=0"); // Disable fast dormancy
  waitForATResponse(1000);
  
  optimizationsApplied = true;
  netOptStatus.lastApplied = millis(); // Update optimization timestamp
  LOG_INFO(MODULE_MODEM, "✅ Optimasi jaringan berhasil diterapkan");
  
  return true;
}

// Re-apply optimizations after reset or reconnection
void ModemManager::reapplyOptimizations() {
  optimizationsApplied = false;
  // Reset optimization status
  netOptStatus = {false, false, false, false, false, 0};
  applyNetworkOptimizations();
}

// Tunggu response AT command dengan timeout yang lebih aggressive
bool ModemManager::waitForATResponse(unsigned long timeout) {
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    if (serialAT.available()) {
      char c = serialAT.read();
      response += c;
      
      // Quick check for OK/ERROR without waiting for full response
      if (response.endsWith("OK\r\n") || response.endsWith("OK\n")) {
        return true;
      }
      if (response.indexOf("ERROR") >= 0) {
        LOG_WARN(MODULE_MODEM, "AT command error: %s", response.c_str());
        return false;
      }
    }
    delay(10); // Smaller delay for faster response
  }
  
  LOG_DEBUG(MODULE_MODEM, "AT response timeout. Response: %s", response.c_str());
  return false;
}

// Tunggu koneksi network dengan optimized polling
bool ModemManager::waitForNetwork(unsigned long timeout) {
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (modem.isNetworkConnected()) {
      currentStatus = MODEM_STATUS_NETWORK_CONNECTED;
      
      // Apply optimizations as soon as network is connected
      applyNetworkOptimizations();
      
      return true;
    }
    Utils::safeDelay(200); // Faster polling for quicker detection
    
    // Update signal quality selama menunggu
    int csq = modem.getSignalQuality();
    if (csq != 99) {
      LOG_DEBUG(MODULE_MODEM, "Signal quality: %d", csq);
    }
  }
  
  return false;
}

// Cek status SIM card dengan faster timeout
bool ModemManager::checkSimCard() {
  LOG_DEBUG(MODULE_MODEM, "Memeriksa status SIM card...");
  
  serialAT.println("AT+CPIN?");
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < 3000) { // Reduced timeout from 5000 to 3000
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
    Utils::safeDelay(50); // Faster polling
  }
  
  LOG_ERROR(MODULE_MODEM, "⏱️ Timeout menunggu status SIM");
  return false;
}

// Enhanced GPRS connection with optimization
bool ModemManager::connectGprs() {
  // Disconnect first if connected
  if (modem.isGprsConnected()) {
    modem.gprsDisconnect();
    Utils::safeDelay(500); // Reduced delay
  }
  
  // Apply optimizations before connecting
  if (!optimizationsApplied) {
    applyNetworkOptimizations();
  }
  
  // Connect with optimized APN
  bool connected = modem.gprsConnect(APN, "", "");
  
  if (connected) {
    // Verify connection with faster method
    sendATCommand("AT+CGACT?");
    Utils::safeDelay(200);
    
    String response = readATResponse(1000);
    if (response.indexOf("+CGACT: 1,1") >= 0) {
      LOG_DEBUG(MODULE_MODEM, "✅ PDP context aktif");
      return true;
    }
  }
  
  return connected;
}

// Setup lengkap modem dengan optimasi
bool ModemManager::setup() {
  LOG_INFO(MODULE_MODEM, "Memulai setup modem...");
  
  // Clear buffer serial AT
  Utils::clearSerialBuffer(serialAT);
  
  // Test AT command dengan reduced attempts untuk faster startup
  bool atResponse = false;
  for (int i = 0; i < 3; i++) { // Reduced from 5 to 3 attempts
    LOG_DEBUG(MODULE_MODEM, "AT command attempt %d/3", i+1);
    serialAT.println("AT");
    atResponse = waitForATResponse(2000); // Reduced timeout
    if (atResponse) break;
    Utils::safeDelay(500 * (i+1)); // Reduced delay
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
  Utils::safeDelay(1500); // Reduced delay
  Utils::clearSerialBuffer(serialAT);
  
  // Inisialisasi modem dengan TinyGSM
  if (!modem.init()) {
    LOG_ERROR(MODULE_MODEM, "❌ Gagal inisialisasi modem");
    currentStatus = MODEM_STATUS_ERROR;
    return false;
  }
  
  currentStatus = MODEM_STATUS_READY;
  
  // Get modem info (faster)
  String modemInfo = modem.getModemInfo();
  LOG_INFO(MODULE_MODEM, "Modem Info: %s", modemInfo.c_str());
  
  // Get IMEI
  String imei = modem.getIMEI();
  if (imei.length() > 0) {
    LOG_INFO(MODULE_MODEM, "IMEI: %s", imei.c_str());
  }
  
  // Update SIM info
  updateSimInfo();
  
  // Tunggu network dengan reduced timeout
  LOG_INFO(MODULE_MODEM, "Menunggu koneksi jaringan...");
  
  bool networkConnected = Utils::retryOperation(
    MODULE_MODEM, 
    "koneksi jaringan",
    [this]() { return waitForNetwork(10000); }, // Reduced from 15000 to 10000
    2, // Reduced from 3 to 2 retries
    2000 // Reduced delay
  );
  
  if (!networkConnected) {
    LOG_ERROR(MODULE_MODEM, "❌ Gagal terhubung ke jaringan");
    currentStatus = MODEM_STATUS_ERROR;
    return false;
  }
  
  LOG_INFO(MODULE_MODEM, "✅ Terhubung ke jaringan");
  
  // Update network info
  updateNetworkStatus();
  
  // Koneksi GPRS dengan optimasi
  LOG_INFO(MODULE_MODEM, "Menghubungkan ke GPRS (APN: %s)...", APN);
  
  bool gprsConnected = Utils::retryOperation(
    MODULE_MODEM,
    "koneksi GPRS",
    [this]() { return connectGprs(); },
    2, // Reduced retries
    1500 // Reduced delay
  );
  
  if (!gprsConnected) {
    LOG_ERROR(MODULE_MODEM, "❌ Gagal koneksi GPRS");
    currentStatus = MODEM_STATUS_NETWORK_CONNECTED;
    return false;
  }
  
  currentStatus = MODEM_STATUS_GPRS_CONNECTED;
  LOG_INFO(MODULE_MODEM, "✅ GPRS terhubung");
  
  // Final optimization check
  if (!optimizationsApplied) {
    applyNetworkOptimizations();
  }
  
  // Log final network info
  LOG_INFO(MODULE_MODEM, getNetworkInfo().c_str());
  
  return true;
}

// Pastikan koneksi GPRS aktif dengan faster recovery
bool ModemManager::ensureConnection() {
  if (!modem.isGprsConnected()) {
    LOG_WARN(MODULE_MODEM, "⚠️ GPRS terputus, mencoba reconnect...");
    
    // Fast reconnection
    bool reconnected = Utils::retryOperation(
      MODULE_MODEM,
      "GPRS reconnection",
      [this]() { return connectGprs(); },
      2, // Reduced retries for faster recovery
      1000 // Reduced delay
    );
    
    if (!reconnected) {
      LOG_ERROR(MODULE_MODEM, "❌ Gagal reconnect GPRS");
      return false;
    }
    
    // Re-apply optimizations after reconnection
    reapplyOptimizations();
    
    Utils::safeDelay(500); // Reduced stabilization time
  }
  
  // Quick verification without verbose logging
  return modem.isGprsConnected();
}

// Enhanced keep-alive for persistent high-performance connection
void ModemManager::maintainConnection() {
  static unsigned long lastMaintenance = 0;
  unsigned long currentTime = millis();
  
  // Maintain connection every 30 seconds
  if (currentTime - lastMaintenance > 30000) {
    if (currentStatus == MODEM_STATUS_GPRS_CONNECTED) {
      // Quick connectivity check
      if (!modem.isGprsConnected()) {
        LOG_WARN(MODULE_MODEM, "🔧 Maintenance: GPRS disconnected, reconnecting...");
        ensureConnection();
      }
      
      // Re-apply optimizations if needed
      if (!optimizationsApplied) {
        reapplyOptimizations();
      }
      
      // Update signal quality for monitoring
      updateNetworkStatus();
    }
    
    lastMaintenance = currentTime;
  }
}

// REMOVED: areOptimizationsApplied() - already defined inline in header

// Force re-optimization (useful after modem reset)
void ModemManager::forceOptimizationReapply() {
  optimizationsApplied = false;
  // Reset optimization status
  netOptStatus = {false, false, false, false, false, 0};
  applyNetworkOptimizations();
}

// Mulai reset modem (optimized for faster recovery)
bool ModemManager::startReset() {
  LOG_INFO(MODULE_MODEM, "🔄 Memulai proses reset modem...");
  
  resetStage = 0;
  resetStartTime = millis();
  resetInProgress = true;
  optimizationsApplied = false; // Reset optimization flag
  netOptStatus = {false, false, false, false, false, 0}; // Reset optimization status
  
  // Disconnect GPRS jika masih terhubung
  if (modem.isGprsConnected()) {
    modem.gprsDisconnect();
  }
  
  // Coba shutdown modem dengan clean
  Utils::clearSerialBuffer(serialAT);
  serialAT.println("AT+CFUN=0"); // Set minimum functionality
  Utils::safeDelay(500); // Reduced delay
  
  currentStatus = MODEM_STATUS_INITIALIZING;
  
  return true;
}

// Continue reset with faster timings
bool ModemManager::continueReset() {
  if (!resetInProgress) {
    return false;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - resetStartTime;
  
  Utils::feedWatchdog();
  
  switch (resetStage) {
    case 0: // Power cycle start
      if (elapsedTime < 1000) { // Reduced from 1500
        digitalWrite(POWER_PIN, HIGH);
      } else {
        digitalWrite(POWER_PIN, LOW);
        resetStage = 1;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 1: Power key HIGH");
      }
      break;
      
    case 1: // Wait 1 second
      if (elapsedTime >= 1000) { // Reduced from 1500
        digitalWrite(POWER_PIN, HIGH);
        resetStage = 2;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 2: Power key LOW");
      }
      break;
      
    case 2: // Wait 1 more second
      if (elapsedTime >= 1000) { // Reduced from 1500
        digitalWrite(POWER_PIN, LOW);
        resetStage = 3;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 3: Menunggu modem siap");
      }
      break;
      
    case 3: // Wait for modem ready (reduced time)
      if (elapsedTime >= 8000) { // Reduced from 12000
        LOG_INFO(MODULE_MODEM, "Modem reset selesai, mencoba reinisialisasi");
        resetStage = 4;
      }
      break;
      
    case 4: // Test AT command (faster)
      Utils::clearSerialBuffer(serialAT);
      
      static int atAttempts = 0;
      serialAT.println("AT");
      
      if (waitForATResponse(500)) { // Reduced timeout
        LOG_INFO(MODULE_MODEM, "✅ Modem merespon setelah reset");
        resetStage = 5;
        atAttempts = 0;
      } else {
        atAttempts++;
        if (atAttempts >= 3) { // Reduced attempts
          LOG_WARN(MODULE_MODEM, "⚠️ Modem belum respon setelah 3 percobaan");
          resetStage = 5;
          atAttempts = 0;
        } else {
          LOG_DEBUG(MODULE_MODEM, "Modem belum respon (attempt %d/3)...", atAttempts);
          Utils::safeDelay(500); // Reduced delay
        }
      }
      break;
      
    case 5: // Stabilization commands (faster)
      LOG_DEBUG(MODULE_MODEM, "Mengirim command stabilisasi");
      Utils::clearSerialBuffer(serialAT);
      
      serialAT.println("AT+CFUN=1"); // Full functionality
      Utils::safeDelay(1000); // Reduced delay
      serialAT.println("AT+CMEE=2"); // Verbose error
      Utils::safeDelay(200);
      serialAT.println("AT+CPIN?"); // Check SIM
      Utils::safeDelay(500);
      
      resetStage = 6;
      resetStartTime = currentTime;
      break;
      
    case 6: // Try full setup (faster)
      if (elapsedTime >= 2000) { // Reduced from 3000
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

// Update informasi SIM card (faster)
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
  
  // Skip phone number for faster setup (optional info)
  
  return simInfo.isReady;
}

// Update status network dengan faster polling
void ModemManager::updateNetworkStatus() {
  // Update signal quality
  lastSignalQuality = modem.getSignalQuality();
  
  // Update operator
  lastOperator = modem.getOperator();
  
  lastStatusCheck = millis();
}

// Get signal quality dengan faster caching
int ModemManager::getSignalQuality() {
  // Update jika data lama (> 15 detik instead of 30)
  if (millis() - lastStatusCheck > 15000) {
    updateNetworkStatus();
  }
  return lastSignalQuality;
}

// Get operator dengan faster caching
String ModemManager::getOperator() {
  // Update jika data lama (> 15 detik instead of 30)
  if (millis() - lastStatusCheck > 15000) {
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
  info += "Optimizations: " + String(optimizationsApplied ? "APPLIED" : "NOT APPLIED") + "\n";
  
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

// Read AT response untuk debugging dengan faster timeout
String ModemManager::readATResponse(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (serialAT.available()) {
      response += serialAT.readString();
    }
    delay(5); // Reduced delay for faster response
  }
  
  return response;
}

// Performance monitoring methods
void ModemManager::startLatencyMeasurement() {
  perfStats.lastTransmissionStart = millis();
  perfStats.measuring = true;
}

void ModemManager::endLatencyMeasurement() {
  if (perfStats.measuring) {
    unsigned long latency = millis() - perfStats.lastTransmissionStart;
    perfStats.totalTransmissions++;
    perfStats.totalLatency += latency;
    
    if (latency < perfStats.minLatency) {
      perfStats.minLatency = latency;
    }
    if (latency > perfStats.maxLatency) {
      perfStats.maxLatency = latency;
    }
    
    perfStats.measuring = false;
    
    LOG_TRACE(MODULE_MODEM, "Transmission latency: %lu ms", latency);
  }
}

void ModemManager::resetPerformanceStats() {
  perfStats = {0, 0, UINT32_MAX, 0, 0, false};
}

unsigned long ModemManager::getAverageLatency() const {
  if (perfStats.totalTransmissions == 0) return 0;
  return perfStats.totalLatency / perfStats.totalTransmissions;
}

String ModemManager::getPerformanceReport() const {
  String report = "=== MODEM PERFORMANCE ===\n";
  report += "Total Transmissions: " + String(perfStats.totalTransmissions) + "\n";
  
  if (perfStats.totalTransmissions > 0) {
    report += "Average Latency: " + String(getAverageLatency()) + " ms\n";
    report += "Min Latency: " + String(perfStats.minLatency) + " ms\n";
    report += "Max Latency: " + String(perfStats.maxLatency) + " ms\n";
  }
  
  return report;
}

// Network diagnostic
bool ModemManager::performNetworkDiagnostic() {
  LOG_INFO(MODULE_MODEM, "Performing network diagnostic...");
  
  // 1. Check registration
  sendATCommand("AT+CREG?");
  String response = readATResponse(1000);
  if (response.indexOf("+CREG: 0,1") < 0 && response.indexOf("+CREG: 0,5") < 0) {
    LOG_ERROR(MODULE_MODEM, "Network registration failed");
    return false;
  }
  
  // 2. Check signal quality
  int signal = getSignalQuality();
  if (signal < 5 || signal == 99) {
    LOG_ERROR(MODULE_MODEM, "Signal too weak: %d", signal);
    return false;
  }
  
  // 3. Check PDP context
  sendATCommand("AT+CGACT?");
  response = readATResponse(1000);
  if (response.indexOf("+CGACT: 1,1") < 0) {
    LOG_ERROR(MODULE_MODEM, "PDP context not active");
    return false;
  }
  
  LOG_INFO(MODULE_MODEM, "Network diagnostic passed");
  return true;
}

String ModemManager::getCurrentNetworkTechnology() {
  sendATCommand("AT+COPS?");
  String response = readATResponse(1000);
  
  // Parse technology from response
  if (response.indexOf(",7") > 0) return "LTE";
  if (response.indexOf(",2") > 0) return "UMTS/3G";
  if (response.indexOf(",0") > 0) return "GSM/2G";
  
  return "Unknown";
}

int ModemManager::getBandInfo() {
  sendATCommand("AT+CBAND?");
  String response = readATResponse(1000);
  
  // Parse band info
  int bandStart = response.indexOf("+CBAND:");
  if (bandStart > 0) {
    int bandEnd = response.indexOf("\r", bandStart);
    String bandStr = response.substring(bandStart + 7, bandEnd);
    return bandStr.toInt();
  }
  
  return -1;
}

bool ModemManager::testDataConnection() {
  // Simple ping test to Google DNS
  sendATCommand("AT+CIPPING=\"8.8.8.8\",1,32,100");
  String response = readATResponse(5000);
  
  return (response.indexOf("+CIPPING: 1") > 0);
}

void ModemManager::logOptimizationDetails() {
  LOG_INFO(MODULE_MODEM, "=== OPTIMIZATION DETAILS ===");
  LOG_INFO(MODULE_MODEM, "LTE-Only Mode: %s", netOptStatus.lteOnlyMode ? "YES" : "NO");
  LOG_INFO(MODULE_MODEM, "All Bands Enabled: %s", netOptStatus.allBandsEnabled ? "YES" : "NO");
  LOG_INFO(MODULE_MODEM, "TCP Optimized: %s", netOptStatus.tcpOptimized ? "YES" : "NO");
  LOG_INFO(MODULE_MODEM, "Compression Enabled: %s", netOptStatus.compressionEnabled ? "YES" : "NO");
  LOG_INFO(MODULE_MODEM, "Keep-Alive Configured: %s", netOptStatus.keepAliveConfigured ? "YES" : "NO");
  
  if (netOptStatus.lastApplied > 0) {
    LOG_INFO(MODULE_MODEM, "Last Applied: %lu seconds ago", 
             (millis() - netOptStatus.lastApplied) / 1000);
  }
}

bool ModemManager::requiresOptimization() const {
  // Check if optimization is needed based on conditions
  if (!optimizationsApplied) return true;
  if (lastSignalQuality < SIGNAL_WEAK_THRESHOLD) return true;
  if (getAverageLatency() > MAX_LATENCY_THRESHOLD) return true;
  
  return false;
}

void ModemManager::setOptimizationMode(bool enable) {
  if (enable && !optimizationsApplied) {
    applyNetworkOptimizations();
  }
}

void ModemManager::setPerformanceMonitoring(bool enable) {
  if (!enable) {
    resetPerformanceStats();
  }
}

// REMOVED: updateOptimizationStatus() - not declared in header