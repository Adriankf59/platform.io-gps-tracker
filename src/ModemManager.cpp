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
  String response = "";
  
  while (millis() - start < timeout) {
    if (serialAT.available()) {
      response += serialAT.readString();
      if (response.indexOf("OK") >= 0) {
        return true;
      }
      // Check for error responses
      if (response.indexOf("ERROR") >= 0) {
        LOG_WARN(MODULE_MODEM, "AT command error: %s", response.c_str());
        return false;
      }
    }
    Utils::safeDelay(100);
  }
  
  LOG_DEBUG(MODULE_MODEM, "AT response timeout. Last response: %s", response.c_str());
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
  
  // Multiple AT command attempts with increasing delays
  bool atResponse = false;
  for (int i = 0; i < 5; i++) {
    LOG_DEBUG(MODULE_MODEM, "AT command attempt %d/5", i+1);
    serialAT.println("AT");
    atResponse = waitForATResponse(3000);
    if (atResponse) break;
    Utils::safeDelay(1000 * (i+1)); // Increasing delay between attempts
  }
  
  if (!atResponse) {
    LOG_ERROR(MODULE_MODEM, "Modem not responding to AT commands after multiple attempts");
    return false;
  }
  
  LOG_DEBUG(MODULE_MODEM, "Modem responding to AT commands");
  
  // Check SIM card status before proceeding
  serialAT.println("AT+CPIN?");
  String response = "";
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (serialAT.available()) {
      response += serialAT.readString();
      if (response.indexOf("+CPIN: READY") >= 0) {
        LOG_INFO(MODULE_MODEM, "SIM card ready");
        break;
      } else if (response.indexOf("CME ERROR: SIM failure") >= 0 ||
                response.indexOf("SIM failure") >= 0) {
        LOG_ERROR(MODULE_MODEM, "SIM card failure detected");
        LOG_INFO(MODULE_MODEM, "Try to: 1) Check if SIM is inserted properly 2) Clean SIM contacts 3) Try another SIM");
        return false;
      } else if (response.indexOf("CME ERROR: SIM PIN required") >= 0) {
        LOG_ERROR(MODULE_MODEM, "SIM PIN required, please remove PIN from SIM card");
        return false;
      } else if (response.indexOf("ERROR") >= 0) {
        LOG_ERROR(MODULE_MODEM, "Unknown SIM error: %s", response.c_str());
        return false;
      }
    }
    Utils::safeDelay(100);
  }
  
  // Check if we timed out waiting for SIM response
  if (response.indexOf("+CPIN: READY") < 0) {
    LOG_ERROR(MODULE_MODEM, "Timeout waiting for SIM card status");
    return false;
  }
  
  // Additional AT commands to stabilize the modem
  serialAT.println("AT+CFUN=1"); // Set full functionality
  Utils::safeDelay(2000);
  Utils::clearSerialBuffer(serialAT);
  
  // Initialize modem with TinyGSM
  if (!modem.init()) {
    LOG_ERROR(MODULE_MODEM, "Failed to initialize modem");
    return false;
  }
  
  String modemInfo = modem.getModemInfo();
  LOG_INFO(MODULE_MODEM, "Modem Info: %s", modemInfo.c_str());
  
  // Check IMEI number
  String imei = modem.getIMEI();
  if (imei.length() > 0) {
    LOG_INFO(MODULE_MODEM, "IMEI: %s", imei.c_str());
  } else {
    LOG_WARN(MODULE_MODEM, "Could not get IMEI");
  }
  
  // Wait for network with longer timeout
  LOG_INFO(MODULE_MODEM, "Waiting for network...");
  
  bool networkConnected = Utils::retryOperation(
    MODULE_MODEM, 
    "network connection",
    [this]() { return waitForNetwork(15000); }, // Increased timeout
    3, 
    3000  // Longer delay between retries
  );
  
  if (!networkConnected) {
    LOG_ERROR(MODULE_MODEM, "Failed to connect to network");
    return false;
  }
  
  LOG_INFO(MODULE_MODEM, "Network connected");
  
  // Get operator name
  String operatorName = modem.getOperator();
  LOG_INFO(MODULE_MODEM, "Connected to operator: %s", operatorName.c_str());
  
  // Get signal quality
  int csq = modem.getSignalQuality();
  LOG_INFO(MODULE_MODEM, "Signal quality: %d (%s)", csq, Utils::getSignalQualityString(csq));
  
  // Connect to GPRS with multiple attempts
  LOG_INFO(MODULE_MODEM, "Connecting to APN: %s", APN);
  
  bool gprsConnected = Utils::retryOperation(
    MODULE_MODEM,
    "GPRS connection",
    [this]() {
      // Disconnect first if needed
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
      [this]() { return modem.gprsConnect(APN, "", ""); },
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
      return modem.gprsConnect(APN, "", "");
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
  
  // Try to cleanly shutdown the modem first
  Utils::clearSerialBuffer(serialAT);
  serialAT.println("AT+CFUN=0"); // Set minimum functionality
  Utils::safeDelay(1000);
  
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
    case 0: // Power cycle start
      if (elapsedTime < 1500) { // Longer initial pulse
        digitalWrite(POWER_PIN, HIGH);
      } else {
        digitalWrite(POWER_PIN, LOW);
        resetStage = 1;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 1: Power key HIGH");
      }
      break;
      
    case 1: // Wait 1.5 seconds
      if (elapsedTime >= 1500) { // Longer wait
        digitalWrite(POWER_PIN, HIGH);
        resetStage = 2;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 2: Power key LOW");
      }
      break;
      
    case 2: // Wait 1.5 more seconds
      if (elapsedTime >= 1500) { // Longer wait
        digitalWrite(POWER_PIN, LOW);
        resetStage = 3;
        resetStartTime = currentTime;
        LOG_DEBUG(MODULE_MODEM, "Reset stage 3: Waiting for modem ready");
      }
      break;
      
    case 3: // Wait for modem ready
      if (elapsedTime >= 12000) { // Longer 12 seconds wait
        LOG_INFO(MODULE_MODEM, "Modem reset complete, trying to reinitialize");
        resetStage = 4;
      }
      break;
      
    case 4: // Initialize modem - multiple attempts
      Utils::clearSerialBuffer(serialAT);
      
      // Try AT command up to 3 times with increasing delays
      static int atAttempts = 0;
      serialAT.println("AT");
      
      if (waitForATResponse(1000)) {
        LOG_INFO(MODULE_MODEM, "Modem responding after reset");
        resetStage = 5;
        atAttempts = 0; // Reset counter
      } else {
        atAttempts++;
        if (atAttempts >= 5) {
          LOG_WARN(MODULE_MODEM, "Modem not responding after 5 AT attempts, continuing anyway");
          resetStage = 5;
          atAttempts = 0; // Reset counter
        } else {
          LOG_DEBUG(MODULE_MODEM, "Modem not responding yet (attempt %d/5)...", atAttempts);
          Utils::safeDelay(1000 * atAttempts); // Increasing delay
          // Stay in stage 4 to retry
        }
      }
      break;
      
    case 5: // Initial AT commands before full setup
      LOG_DEBUG(MODULE_MODEM, "Sending stabilization commands");
      Utils::clearSerialBuffer(serialAT);
      
      // Send stabilization commands
      serialAT.println("AT+CFUN=1"); // Full functionality
      Utils::safeDelay(2000);
      serialAT.println("AT+CMEE=2"); // Enable verbose error messages
      Utils::safeDelay(500);
      serialAT.println("AT+CPIN?"); // Check SIM status
      Utils::safeDelay(1000);
      
      resetStage = 6;
      resetStartTime = currentTime;
      break;
      
    case 6: // Full setup after stabilization
      if (elapsedTime >= 3000) { // Wait 3 seconds after stabilization commands
        LOG_INFO(MODULE_MODEM, "Attempting full modem setup after reset");
        
        if (setup()) {
          resetRetries++;
          LOG_INFO(MODULE_MODEM, "Modem reset successful (attempt #%d)", resetRetries);
          resetInProgress = false;
          return true;
        } else {
          LOG_ERROR(MODULE_MODEM, "Failed to setup modem after reset");
          // Check if we've tried too many times
          if (resetRetries >= MAX_RESET_RETRIES) {
            LOG_ERROR(MODULE_MODEM, "Exceeded maximum reset attempts (%d). Hardware may need inspection.", MAX_RESET_RETRIES);
            LOG_INFO(MODULE_MODEM, "Troubleshooting tips: 1) Check power supply 2) Check SIM card 3) Check antenna 4) Check wiring");
            resetInProgress = false;
            return false;
          }
          // Increment retry counter but continue with setup attempts
          resetRetries++;
          resetStage = 0; // Restart the reset process
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