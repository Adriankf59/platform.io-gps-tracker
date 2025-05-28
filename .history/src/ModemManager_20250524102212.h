// ModemManager.h
#ifndef MODEM_MANAGER_H
#define MODEM_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

class ModemManager {
private:
  TinyGsm& modem;
  HardwareSerial& serialAT;
  
  // Reset state machine
  unsigned long resetStartTime;
  int resetStage;
  bool resetInProgress;
  int resetRetries;
  
  // Helper methods
  bool waitForATResponse(unsigned long timeout = 3000);
  bool waitForNetwork(unsigned long timeout = 10000);
  
public:
  ModemManager(TinyGsm& modemInstance, HardwareSerial& serial);
  
  void begin();
  bool setup();
  bool ensureConnection();
  
  // Non-blocking reset
  bool startReset();
  bool continueReset();
  
  // Status getters
  bool isNetworkConnected() const { return modem.isNetworkConnected(); }
  bool isGprsConnected() const { return modem.isGprsConnected(); }
  int getSignalQuality() const { return modem.getSignalQuality(); }
  String getOperator() const { return modem.getOperator(); }
  int getResetRetries() const { return resetRetries; }
  
  // GPRS operations (public)
  void disconnectGprs() { modem.gprsDisconnect(); }
  bool connectGprs() { return modem.gprsConnect(APN); }
  
  // For unit testing
  friend class ModemManagerTest;
};

#endif // MODEM_MANAGER_H