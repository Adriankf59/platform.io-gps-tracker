// ModemManager.h - Manajer Modem GSM A7670C untuk ESP32 Tracker (Optimized for Low Latency)
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

// Network optimization status
struct NetworkOptimization {
  bool lteOnlyMode;
  bool allBandsEnabled;
  bool tcpOptimized;
  bool compressionEnabled;
  bool keepAliveConfigured;
  unsigned long lastApplied;
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
  
  // ===== OPTIMIZATION TRACKING =====
  bool optimizationsApplied;
  NetworkOptimization netOptStatus;
  unsigned long lastMaintenanceCheck;
  
  // ===== PERFORMANCE METRICS =====
  struct PerformanceStats {
    unsigned long totalTransmissions;
    unsigned long totalLatency;
    unsigned long minLatency;
    unsigned long maxLatency;
    unsigned long lastTransmissionStart;
    bool measuring;
  } perfStats;
  
  // ===== HELPER METHODS =====
  bool waitForATResponse(unsigned long timeout = 2000);  // Reduced default timeout
  bool waitForNetwork(unsigned long timeout = 8000);     // Reduced default timeout
  bool checkSimCard();
  void updateStatus();
  
  // ===== OPTIMIZATION METHODS =====
  bool applyLTEOnlyMode();
  bool enableAllLTEBands();
  bool optimizeTCPSettings();
  bool enableHeaderCompression();
  bool configureFastHandover();
  bool disablePowerSaving();
  void resetOptimizationStatus();
  
public:
  ModemManager(TinyGsm& modemInstance, HardwareSerial& serial);
  
  // ===== FUNGSI UTAMA =====
  void begin();              // Inisialisasi hardware modem
  bool setup();              // Setup lengkap (init + network + GPRS)
  bool ensureConnection();   // Pastikan koneksi GPRS aktif
  
  // ===== RESET NON-BLOCKING =====
  bool startReset();         // Mulai proses reset
  bool continueReset();      // Lanjutkan proses reset (return true jika masih proses)
  
  // ===== OPTIMIZATION FUNCTIONS =====
  bool applyNetworkOptimizations();     // Apply all network optimizations
  void reapplyOptimizations();          // Re-apply optimizations after reset
  void maintainConnection();            // Maintain optimized connection
  bool areOptimizationsApplied() const { return optimizationsApplied; }
  void forceOptimizationReapply();      // Force re-apply optimizations
  NetworkOptimization getOptimizationStatus() const { return netOptStatus; }
  
  // ===== PERFORMANCE MONITORING =====
  void startLatencyMeasurement();       // Start measuring transmission latency
  void endLatencyMeasurement();         // End measuring and record latency
  void resetPerformanceStats();         // Reset performance statistics
  unsigned long getAverageLatency() const;
  unsigned long getMinLatency() const { return perfStats.minLatency; }
  unsigned long getMaxLatency() const { return perfStats.maxLatency; }
  unsigned long getTotalTransmissions() const { return perfStats.totalTransmissions; }
  String getPerformanceReport() const;  // Get formatted performance report
  
  // ===== STATUS GETTERS =====
  bool isNetworkConnected() const { return modem.isNetworkConnected(); }
  bool isGprsConnected() const { return modem.isGprsConnected(); }
  int getSignalQuality();    // Update dan return signal quality (faster caching)
  String getOperator();      // Update dan return operator name (faster caching)
  int getResetRetries() const { return resetRetries; }
  ModemStatus getStatus() const { return currentStatus; }
  const char* getStatusString() const;
  
  // ===== SIM CARD INFO =====
  const SimInfo& getSimInfo() const { return simInfo; }
  bool updateSimInfo();      // Update informasi SIM card (optimized)
  
  // ===== OPERASI GPRS =====
  void disconnectGprs() { 
    modem.gprsDisconnect(); 
    currentStatus = MODEM_STATUS_NETWORK_CONNECTED;
  }
  bool connectGprs();        // Enhanced GPRS connection with optimization
  
  // ===== MONITORING =====
  void updateNetworkStatus();  // Update semua status network (faster)
  String getNetworkInfo();     // Get formatted network info (includes optimization status)
  bool isSignalWeak() const { return lastSignalQuality < 10; }
  bool isSignalStrong() const { return lastSignalQuality > 20; }
  bool requiresOptimization() const;  // Check if optimization is needed
  
  // ===== ADVANCED DIAGNOSTICS =====
  bool performNetworkDiagnostic();     // Comprehensive network test
  String getCurrentNetworkTechnology(); // Get current network tech (2G/3G/4G)
  int getBandInfo();                   // Get current LTE band
  bool testDataConnection();           // Quick data connection test
  void logOptimizationDetails();       // Log detailed optimization status
  
  // ===== DEBUGGING =====
  void sendATCommand(const String& command);
  String readATResponse(unsigned long timeout = 500);  // Reduced default timeout
  
  // ===== CONFIGURATION =====
  void setOptimizationMode(bool enable);  // Enable/disable auto-optimization
  void setPerformanceMonitoring(bool enable); // Enable/disable perf monitoring
  
  // ===== CONSTANTS FOR OPTIMIZATION =====
  static const unsigned long MAINTENANCE_INTERVAL = 30000;     // 30 seconds
  static const unsigned long OPTIMIZATION_RECHECK = 300000;    // 5 minutes
  static const unsigned long FAST_STATUS_UPDATE = 15000;       // 15 seconds
  static const int MIN_SIGNAL_FOR_OPTIMIZATION = 15;           // Minimum signal for optimization
  static const int MAX_LATENCY_THRESHOLD = 2000;               // 2 seconds max acceptable latency
  
  // Untuk unit testing
  friend class ModemManagerTest;
};

// ===== HELPER FUNCTIONS =====
namespace ModemOptimization {
  // Utility functions for optimization
  bool isLTEBandSupported(int band);
  String getOptimizationRecommendation(int signalQuality, const String& operator_name);
  unsigned long calculateOptimalTimeout(int signalQuality);
  bool shouldApplyAggression(int consecutiveFailures);
}

#endif // MODEM_MANAGER_H