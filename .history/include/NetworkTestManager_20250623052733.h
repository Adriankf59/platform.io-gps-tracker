// NetworkTestManager.h - Network Stability Testing Module
#ifndef NETWORK_TEST_MANAGER_H
#define NETWORK_TEST_MANAGER_H

#include <Arduino.h>
#include "Config.h"
#include "Logger.h"
#include "ModemManager.h"
#include "WebSocketManager.h"
#include "HttpClient.h"

// Test modes
enum TestMode {
  TEST_MODE_NONE = 0,
  TEST_MODE_CONTINUOUS_TRANSMISSION,
  TEST_MODE_CONNECTION_DISRUPTION,
  TEST_MODE_LOAD_TEST,
  TEST_MODE_SIGNAL_MONITORING
};

// Network metrics structure
struct NetworkMetrics {
  // Connection metrics
  unsigned long totalPacketsSent = 0;
  unsigned long successfulPackets = 0;
  unsigned long failedPackets = 0;
  float successRate = 0.0;
  
  // Timing metrics
  unsigned long totalResponseTime = 0;
  unsigned long avgResponseTime = 0;
  unsigned long minResponseTime = 999999;
  unsigned long maxResponseTime = 0;
  
  // Connection stability
  unsigned long connectionUptime = 0;
  unsigned long connectionDowntime = 0;
  float uptimePercentage = 0.0;
  unsigned long disconnectCount = 0;
  
  // Recovery metrics
  unsigned long totalRecoveryTime = 0;
  unsigned long avgRecoveryTime = 0;
  unsigned long lastDisconnectTime = 0;
  
  // Signal metrics
  int currentSignalStrength = 0;
  int avgSignalStrength = 0;
  int minSignalStrength = 99;
  int maxSignalStrength = 0;
  
  // Test timing
  unsigned long testStartTime = 0;
  unsigned long testDuration = 0;
};

// Signal condition thresholds
struct SignalCondition {
  const char* name;
  int minRSSI;
  int maxRSSI;
};

class NetworkTestManager {
private:
  ModemManager* modemManager;
  WebSocketManager* wsManager;
  HttpClientWrapper* httpClient;
  
  // Test state
  TestMode currentTestMode = TEST_MODE_NONE;
  NetworkMetrics metrics;
  bool testRunning = false;
  
  // Test parameters
  unsigned long testStartTime = 0;
  unsigned long testDuration = 1800000; // 30 minutes default
  unsigned long dataInterval = 10000; // 10 seconds default
  unsigned long lastDataSendTime = 0;
  
  // Connection monitoring
  bool wasConnected = false;
  unsigned long lastSignalCheckTime = 0;
  unsigned long signalCheckInterval = 5000; // Check every 5 seconds
  
  // Disruption test parameters
  bool disruptionScheduled = false;
  unsigned long disruptionStartTime = 0;
  unsigned long disruptionDuration = 0;
  int disruptionPhase = 0;
  
  // Load test parameters
  unsigned long loadTestPacketCount = 0;
  unsigned long loadTestStartTime = 0;
  
  // Signal conditions
  SignalCondition signalConditions[3] = {
    {"STRONG", 20, 31},   // CSQ > 20 (approx > -70 dBm)
    {"MEDIUM", 10, 19},   // CSQ 10-19 (approx -70 to -90 dBm)
    {"WEAK", 2, 9}        // CSQ 2-9 (approx -90 to -110 dBm)
  };
  
  // Helper methods
  void updateSignalMetrics();
  void updateConnectionMetrics();
  void calculateAverages();
  const char* getCurrentSignalCondition();
  void logMetrics();
  void saveMetricsToFile();
  
public:
  NetworkTestManager(ModemManager* modem, WebSocketManager* ws, HttpClientWrapper* http);
  
  // Test control
  bool startTest(TestMode mode, unsigned long duration = 1800000);
  void stopTest();
  void update();
  
  // Test configuration
  void setDataInterval(unsigned long interval) { dataInterval = interval; }
  void setTestDuration(unsigned long duration) { testDuration = duration; }
  
  // Disruption test
  void scheduleDisruption(unsigned long delay, unsigned long duration);
  
  // Metrics access
  NetworkMetrics getMetrics() const { return metrics; }
  void printMetrics();
  void resetMetrics();
  
  // Test status
  bool isTestRunning() const { return testRunning; }
  TestMode getCurrentTestMode() const { return currentTestMode; }
  const char* getTestModeName() const;
  unsigned long getTestElapsedTime() const;
  float getTestProgress() const;
  
  // Data transmission for testing
  bool sendTestData();
  
  // Signal monitoring
  void monitorSignalQuality();
  
  // File operations for test results
  void exportTestResults();
};

#endif // NETWORK_TEST_MANAGER_H