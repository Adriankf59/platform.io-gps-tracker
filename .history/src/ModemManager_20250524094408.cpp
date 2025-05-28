// ModemManager.cpp
#include "ModemManager.h"

ModemManager::ModemManager(TinyGsm& modemInstance, HardwareSerial& serial)
  : modem(modemInstance),
    serialAT(serial),
    resetStartTime(0),
    resetStage(0),
    resetInProgress(false),
    resetRetries(0) {
}

void ModemManager::begin() {
  LOG_INFO(MODULE_MODEM, "Initializing GSM A7670C module...");
  
  // Power sequence for A7670C module
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  delay(100);
  digitalWrite(POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(POWER_PIN, LOW);
  
  // Reset watchdog during initialization
  for (int i = 0; i < 5; i++) {
    Utils::safeDelay(1000);
  }
  
  // Initialize serial communication
  serialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(500);
  
  LOG_INFO(MODULE_MODEM, "Modem hardware initialized");
}

bool ModemManager::waitForATResponse(unsigned long timeout) {
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (serialAT.available()) {
      String response = serialAT.readString();
      if (response.indexOf("OK") >= 0) {
        return true;
      }
    }
    Utils::safeDelay(100);
  }
  
  return false;
}

bool ModemManager::waitForNetwork(unsigned long timeout) {
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (modem.isNetworkConnected()) {
      return true;
    }
    Utils::safeDelay(500);
  }
  
  return false;
}

bool ModemManager::setup() {
  LOG_INFO(MODULE_MODEM, "Setting up modem...");
  
  // Clear AT command serial buffer
  Utils::clearSerialBuffer(serialAT);
  
  // Send AT command to check if modem is responsive
  serialAT.println("AT");
  
  if (!waitForATResponse()) {
    LOG_ERROR(MODULE_MODEM, "Modem not responding to AT command");
    return false;
  }
  
  LOG_DEBUG(MODULE_MODEM, "Modem responding to AT commands");
  
  // Initialize modem with TinyGSM
  if (!modem.init()) {
    LOG_ERROR(MODULE_MODEM, "Failed to initialize modem");
    return false;
  }
  
  String modemInfo = modem.getModemInfo();
  LOG_INFO(MODULE_MODEM, "Modem Info: %s", modemInfo.c_str());
  
  // Wait for network
  LOG_INFO(MODULE_MODEM, "Waiting for network...");
  
  bool networkConnected = Utils::retryOperation(
    MODULE_MODEM, 
    "network connection",
    [this]() { return waitForNetwork(10000); },
    3, 
    2000
  );
  
  if (!networkConnected) {
    LOG_ERROR(MODULE_MODEM, "Failed to connect to network");
    return false;
  }
  
  LOG_INFO(MODULE_MODEM, "Network connected");
  
  // Connect to GPRS
  LOG_INFO(MODULE_MODEM, "Connecting to APN: %s", APN);
  
  if (!modem.gprsConnect(APN, "", "")) {
    LOG_ERROR(MODULE_MODEM, "Failed to connect to GPRS");
    return false;
  }
  
  LOG_INFO(MODULE_MODEM, "GPRS connected successfully");
  return true;
}

bool ModemManager::ensureConnection() {
  if (!modem.isGprsConnected()) {
    LOG_WARN(MODULE_MODEM, "GPRS disconnected, attempting to reconnect...");
    
    // Disconnect first to clear stale connections
    modem.gprsDisconnect();
    Utils::safeDelay(1000);
    
    // Try to reconnect
    bool reconnected = Utils::retryOperation(
      MODULE_MODEM,
      "GPRS reconnection",
      [this]() { return modem.gprsConnect(APN); },
      3,
      2000
    );
    
    if (!reconnected) {
      LOG_ERROR(MODULE_MODEM, "Failed to reconnect to GPRS");
      return false;
    }
    
    Utils::safeDelay(1000); // Stabilization time
  }
  
  // Verify connection with AT commands
  LOG_DEBUG(MODULE_MODEM, "Verifying GPRS connection status...");
  serialAT.println("AT+CGACT?");
  delay(500);
  
  if (serialAT.available()) {
    String response = serialAT.readString();
    LOG_TRACE(MODULE_MODEM, "CGACT Response: %s", response.c_str());
    
    if (response.indexOf("+CGACT: 1,1") < 0) {
      LOG_WARN(MODULE_MODEM, "PDP context not active, reactivating...");
      modem.gprsDisconnect();
      Utils::safeDelay(1000);
      return modem.gprsConnect(APN);
    }
  }
  
  return true;
}

bool ModemManager::startReset() {
  LOG_INFO(MODULE_MODEM, "Starting modem reset process...");
  
  resetStage = 0;
  resetStartTime = millis();
  resetInProgress = true;
  
  // Disconnect GPRS if connected
  if (modem.isGprsConnected()) {
    modem.gprsDisconnect();
  }
  
  return true;
}

bool ModemManager::continueReset() {
  if (!resetInProgress) {
    return false;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - resetStartTime;
  
  Utils::feedWatchdog();
  
  switch (resetStage) {
    case 0