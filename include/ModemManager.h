// ModemManager.h - Enhanced with A7670C CPSI Signal Monitoring
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

// Signal Quality Information (A7670C CPSI Support)
struct SignalInfo {
  int csq;                    // Classic signal quality (0-31, 99=unknown)
  float rsrq;                 // Reference Signal Received Quality (dB) - from CPSI
  float rsrp;                 // Reference Signal Received Power (dBm) - from CPSI
  unsigned long lastUpdate;   // Timestamp of last update
  bool rsrqValid;             // RSRQ value is valid (from CPSI)
  bool rsrpValid;             // RSRP value is valid (from CPSI)
  
  void reset() {
    csq = 99;
    rsrq = RSRQ_INVALID_VALUE;
    rsrp = RSRP_INVALID_VALUE;
    lastUpdate = 0;
    rsrqValid = false;
    rsrpValid = false;
  }
  
  bool isValid() const {
    return csq != 99 && csq > 0;
  }
  
  bool hasLteMetrics() const {
    return rsrqValid && rsrpValid;
  }
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
  String lastOperator;
  
  // Signal monitoring (A7670C CPSI)
  SignalInfo signalInfo;
  unsigned long lastSignalUpdate;
  
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
  bool waitForATResponse(unsigned long timeout = 2000);
  bool waitForNetwork(unsigned long timeout = 8000);
  bool checkSimCard();
  void updateStatus();
  
  // ===== A7670C CPSI SIGNAL MONITORING METHODS (NEW) =====
  bool updateSignalMetrics();                   // Update RSRQ/RSRP using CPSI
  bool parseCPSIResponse(const String& response); // Parse AT+CPSI response (A7670C)
  bool parseLTECPSI(const String& cpsiLine);    // Parse LTE specific CPSI data
  bool debugCPSIResponse();                     // Debug CPSI response for troubleshooting
  bool forceLTEMode();                          // Force A7670C to LTE-only mode
  void logSignalQuality();                      // Log signal metrics
  
  // ===== LEGACY CESQ METHODS (fallback for compatibility) =====
  bool parseSignalResponse(const String& response); // Parse AT+CESQ response (fallback)
  float parseRSRQ(int rawValue);                // Convert raw RSRQ to dB (CESQ)
  float parseRSRP(int rawValue);                // Convert raw RSRP to dBm (CESQ)
  
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
  
  // ===== A7670C SIGNAL MONITORING FUNCTIONS (CPSI-based) =====
  void updateSignalInfo();              // Update all signal information using CPSI
  int getSignalQuality();               // Get classic CSQ value (0-31)
  float getRSRQ();                      // Get RSRQ value in dB (from CPSI)
  float getRSRP();                      // Get RSRP value in dBm (from CPSI)
  const SignalInfo& getSignalInfo() const { return signalInfo; }
  String getSignalQualityReport() const; // Get formatted signal report
  bool isSignalWeak() const;            // Check if signal is weak
  bool isSignalStrong() const;          // Check if signal is strong
  bool hasValidLteMetrics() const { return signalInfo.hasLteMetrics(); }
  bool isInLTEMode() const;             // Check if modem is in LTE mode
  String getCurrentNetworkMode();       // Get current network mode (LTE/GSM/UMTS)
  String getLTEBandInfo();              // Get current LTE band information
  bool validateLTESignal() const;       // Validate if LTE signal metrics are reliable
  
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
  String getNetworkInfo();     // Get formatted network info (includes A7670C CPSI data)
  bool requiresOptimization() const;  // Check if optimization is needed
  
  // ===== ENHANCED DIAGNOSTICS FOR A7670C =====
  bool performNetworkDiagnostic();     // Comprehensive network test
  bool performLTEDiagnostic();          // A7670C LTE-specific diagnostic
  String getCurrentNetworkTechnology(); // Get current network tech (2G/3G/4G)
  String getCPSIInfo();                 // Get formatted CPSI information
  bool optimizeForLTE();                // Apply A7670C LTE optimizations
  int getBandInfo();                    // Get current LTE band
  bool testDataConnection();            // Quick data connection test
  void logOptimizationDetails();        // Log detailed optimization status
  
  // ===== DEBUGGING =====
  void sendATCommand(const String& command);
  String readATResponse(unsigned long timeout = 500);
  
  // ===== CONFIGURATION =====
  void setOptimizationMode(bool enable);  // Enable/disable auto-optimization
  void setPerformanceMonitoring(bool enable); // Enable/disable perf monitoring
  void setSignalMonitoring(bool enable);  // Enable/disable A7670C CPSI signal monitoring
  
  // ===== CONSTANTS FOR OPTIMIZATION =====
  static const unsigned long MAINTENANCE_INTERVAL = 30000;     // 30 seconds
  static const unsigned long OPTIMIZATION_RECHECK = 300000;    // 5 minutes
  static const unsigned long FAST_STATUS_UPDATE = 15000;       // 15 seconds
  static const int MIN_SIGNAL_FOR_OPTIMIZATION = 15;           // Minimum signal for optimization
  static const int MAX_LATENCY_THRESHOLD = 2000;               // 2 seconds max acceptable latency
  
  // ===== A7670C SPECIFIC CONSTANTS =====
  static const unsigned long A7670C_CPSI_INTERVAL = 5000;        // CPSI update interval
  static const unsigned long A7670C_LTE_SWITCH_TIMEOUT = 10000;  // LTE mode switch timeout
  static const int A7670C_MIN_LTE_RSRP = -120;                   // Minimum usable LTE RSRP
  static const int A7670C_MIN_LTE_RSRQ = -15;                    // Minimum usable LTE RSRQ
  
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

// ===== SIGNAL QUALITY HELPERS (A7670C CPSI) =====
namespace SignalAnalysis {
  // Signal quality analysis functions
  String getSignalQualityDescription(int csq);
  String getRSRQQualityDescription(float rsrq);
  String getRSRPQualityDescription(float rsrp);
  bool isSignalSuitableForOptimization(const SignalInfo& signal);
  float calculateSignalScore(const SignalInfo& signal); // 0-100 score
}

#endif // MODEM_MANAGER_H