// ModemManager.h - Manajer Modem GSM A7670C untuk ESP32 Tracker
#ifndef MODEM_MANAGER_H
#define MODEM_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// Status modem untuk monitoring
enum ModemStatus {
  MODEM_STATUS_OFF,
  MODEM_STATUS_INITIALIZING,
  MODEM_STATUS_READY,
  MODEM_STATUS_NETWORK_CONNECTED,
  MODEM_STATUS_GPRS_CONNECTED,
  MODEM_STATUS_ERROR
};

// Informasi SIM card
struct SimInfo {
  bool isReady;
  String imsi;
  String iccid;
  String phoneNumber;
};

class ModemManager {
private:
  TinyGsm& modem;
  HardwareSerial& serialAT;
  
  // State untuk reset non-blocking
  unsigned long resetStartTime;
  int resetStage;
  bool resetInProgress;
  int resetRetries;
  
  // Status tracking
  ModemStatus currentStatus;
  unsigned long lastStatusCheck;
  int lastSignalQuality;
  String lastOperator;
  
  // SIM card info
  SimInfo simInfo;
  
  // Helper methods
  bool waitForATResponse(unsigned long timeout = 3000);
  bool waitForNetwork(unsigned long timeout = 10000);
  bool checkSimCard();
  void updateStatus();
  
public:
  ModemManager(TinyGsm& modemInstance, HardwareSerial& serial);
  
  // ===== FUNGSI UTAMA =====
  void begin();              // Inisialisasi hardware modem
  bool setup();              // Setup lengkap (init + network + GPRS)
  bool ensureConnection();   // Pastikan koneksi GPRS aktif
  
  // ===== RESET NON-BLOCKING =====
  bool startReset();         // Mulai proses reset
  bool continueReset();      // Lanjutkan proses reset (return true jika masih proses)
  
  // ===== STATUS GETTERS =====
  bool isNetworkConnected() const { return modem.isNetworkConnected(); }
  bool isGprsConnected() const { return modem.isGprsConnected(); }
  int getSignalQuality();    // Update dan return signal quality
  String getOperator();      // Update dan return operator name
  int getResetRetries() const { return resetRetries; }
  ModemStatus getStatus() const { return currentStatus; }
  const char* getStatusString() const;
  
  // ===== SIM CARD INFO =====
  const SimInfo& getSimInfo() const { return simInfo; }
  bool updateSimInfo();      // Update informasi SIM card
  
  // ===== OPERASI GPRS =====
  void disconnectGprs() { 
    modem.gprsDisconnect(); 
    currentStatus = MODEM_STATUS_NETWORK_CONNECTED;
  }
  bool connectGprs() { 
    bool result = modem.gprsConnect(APN, "", "");
    if (result) currentStatus = MODEM_STATUS_GPRS_CONNECTED;
    return result;
  }
  
  // ===== MONITORING =====
  void updateNetworkStatus();  // Update semua status network
  String getNetworkInfo();     // Get formatted network info
  bool isSignalWeak() const { return lastSignalQuality < 10; }
  
  // ===== DEBUGGING =====
  void sendATCommand(const String& command);
  String readATResponse(unsigned long timeout = 1000);
  
  // Untuk unit testing
  friend class ModemManagerTest;
};

#endif // MODEM_MANAGER_H