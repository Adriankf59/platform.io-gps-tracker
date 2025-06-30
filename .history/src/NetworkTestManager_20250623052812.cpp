// NetworkTestManager.cpp - Network Testing Implementation
#include "NetworkTestManager.h"
#include <ArduinoJson.h>

NetworkTestManager::NetworkTestManager(ModemManager* modem, WebSocketManager* ws, HttpClientWrapper* http)
  : modemManager(modem), wsManager(ws), httpClient(http) {
  resetMetrics();
}

bool NetworkTestManager::startTest(TestMode mode, unsigned long duration) {
  if (testRunning) {
    LOG_WARN(MODULE_SYS, "Test already running");
    return false;
  }
  
  currentTestMode = mode;
  testDuration = duration;
  testRunning = true;
  testStartTime = millis();
  
  resetMetrics();
  metrics.testStartTime = testStartTime;
  
  LOG_INFO(MODULE_SYS, "=== NETWORK TEST STARTED ===");
  LOG_INFO(MODULE_SYS, "Mode: %s", getTestModeName());
  LOG_INFO(MODULE_SYS, "Duration: %lu minutes", duration / 60000);
  LOG_INFO(MODULE_SYS, "Data Interval: %lu seconds", dataInterval / 1000);
  LOG_INFO(MODULE_SYS, "Initial Signal: CSQ %d (%s)", 
           modemManager->getSignalQuality(), getCurrentSignalCondition());
  
  // Initialize connection state
  wasConnected = wsManager->isReady();
  
  return true;
}

void NetworkTestManager::stopTest() {
  if (!testRunning) return;
  
  testRunning = false;
  metrics.testDuration = millis() - testStartTime;
  
  calculateAverages();
  
  LOG_INFO(MODULE_SYS, "=== NETWORK TEST COMPLETED ===");
  printMetrics();
  saveMetricsToFile();
  
  currentTestMode = TEST_MODE_NONE;
}

void NetworkTestManager::update() {
  if (!testRunning) return;
  
  unsigned long currentTime = millis();
  
  // Check if test duration exceeded
  if (currentTime - testStartTime >= testDuration) {
    stopTest();
    return;
  }
  
  // Update signal metrics
  if (currentTime - lastSignalCheckTime >= signalCheckInterval) {
    updateSignalMetrics();
    lastSignalCheckTime = currentTime;
  }
  
  // Update connection metrics
  updateConnectionMetrics();
  
  // Execute test mode specific actions
  switch (currentTestMode) {
    case TEST_MODE_CONTINUOUS_TRANSMISSION:
      if (currentTime - lastDataSendTime >= dataInterval) {
        sendTestData();
        lastDataSendTime = currentTime;
      }
      break;
      
    case TEST_MODE_CONNECTION_DISRUPTION:
      // Handle scheduled disruptions
      if (disruptionScheduled && currentTime >= disruptionStartTime) {
        if (disruptionPhase == 0) {
          // Start disruption
          LOG_INFO(MODULE_SYS, "🔴 Starting connection disruption for %lu seconds", 
                   disruptionDuration / 1000);
          wsManager->disconnect();
          modemManager->disconnectGprs();
          disruptionPhase = 1;
        } else if (currentTime - disruptionStartTime >= disruptionDuration) {
          // End disruption
          LOG_INFO(MODULE_SYS, "🟢 Ending disruption, attempting reconnection...");
          disruptionScheduled = false;
          disruptionPhase = 0;
          // Connection will be restored by main loop
        }
      }
      
      // Send data when not disrupted
      if (!disruptionScheduled && currentTime - lastDataSendTime >= dataInterval) {
        sendTestData();
        lastDataSendTime = currentTime;
      }
      break;
      
    case TEST_MODE_LOAD_TEST:
      // Send data as fast as possible
      if (wsManager->isReady()) {
        sendTestData();
        loadTestPacketCount++;
        
        // Log progress every 100 packets
        if (loadTestPacketCount % 100 == 0) {
          LOG_INFO(MODULE_SYS, "Load test: %lu packets sent, success rate: %.1f%%", 
                   loadTestPacketCount, metrics.successRate);
        }
      }
      break;
      
    case TEST_MODE_SIGNAL_MONITORING:
      // Just monitor signal, don't send data
      monitorSignalQuality();
      break;
  }
  
  // Log periodic status
  if ((currentTime - testStartTime) % 60000 < 100) { // Every minute
    float progress = getTestProgress();
    LOG_INFO(MODULE_SYS, "Test progress: %.1f%%, Success rate: %.1f%%, Uptime: %.1f%%", 
             progress, metrics.successRate, metrics.uptimePercentage);
  }
}

void NetworkTestManager::updateSignalMetrics() {
  int csq = modemManager->getSignalQuality();
  if (csq >= 0 && csq <= 31) {
    metrics.currentSignalStrength = csq;
    
    // Update min/max
    if (csq < metrics.minSignalStrength) metrics.minSignalStrength = csq;
    if (csq > metrics.maxSignalStrength) metrics.maxSignalStrength = csq;
    
    // Calculate running average
    static int signalSamples = 0;
    signalSamples++;
    metrics.avgSignalStrength = ((metrics.avgSignalStrength * (signalSamples - 1)) + csq) / signalSamples;
  }
}

void NetworkTestManager::updateConnectionMetrics() {
  bool currentlyConnected = wsManager->isReady();
  unsigned long currentTime = millis();
  
  // Detect connection state changes
  if (currentlyConnected && !wasConnected) {
    // Connection restored
    if (metrics.lastDisconnectTime > 0) {
      unsigned long recoveryTime = currentTime - metrics.lastDisconnectTime;
      metrics.totalRecoveryTime += recoveryTime;
      LOG_INFO(MODULE_SYS, "✅ Connection restored after %lu ms", recoveryTime);
    }
  } else if (!currentlyConnected && wasConnected) {
    // Connection lost
    metrics.disconnectCount++;
    metrics.lastDisconnectTime = currentTime;
    LOG_WARN(MODULE_SYS, "❌ Connection lost (count: %lu)", metrics.disconnectCount);
  }
  
  // Update uptime/downtime
  if (currentlyConnected) {
    metrics.connectionUptime += (currentTime - testStartTime) - 
                                (metrics.connectionUptime + metrics.connectionDowntime);
  } else {
    metrics.connectionDowntime += (currentTime - testStartTime) - 
                                 (metrics.connectionUptime + metrics.connectionDowntime);
  }
  
  // Calculate uptime percentage
  unsigned long totalTime = currentTime - testStartTime;
  if (totalTime > 0) {
    metrics.uptimePercentage = (float)metrics.connectionUptime / totalTime * 100.0;
  }
  
  wasConnected = currentlyConnected;
}

bool NetworkTestManager::sendTestData() {
  unsigned long sendStartTime = millis();
  
  metrics.totalPacketsSent++;
  
  // Prepare test data with timestamp
  char timestamp[30];
  snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
           2025, 1, 15, 12, 0, 0, (int)(millis() % 1000));
  
  // Send via WebSocket
  bool success = false;
  if (wsManager->isReady()) {
    // For testing, send with test metadata
    success = wsManager->sendVehicleData(
      -6.175110,  // Test latitude
      106.865039, // Test longitude
      0.0,        // Speed
      10,         // Satellites
      timestamp
    );
  }
  
  unsigned long responseTime = millis() - sendStartTime;
  
  if (success) {
    metrics.successfulPackets++;
    metrics.totalResponseTime += responseTime;
    
    // Update min/max response time
    if (responseTime < metrics.minResponseTime) metrics.minResponseTime = responseTime;
    if (responseTime > metrics.maxResponseTime) metrics.maxResponseTime = responseTime;
    
    LOG_DEBUG(MODULE_SYS, "Test packet sent successfully (RTT: %lu ms)", responseTime);
  } else {
    metrics.failedPackets++;
    LOG_WARN(MODULE_SYS, "Test packet failed");
  }
  
  // Calculate success rate
  if (metrics.totalPacketsSent > 0) {
    metrics.successRate = (float)metrics.successfulPackets / metrics.totalPacketsSent * 100.0;
  }
  
  return success;
}

void NetworkTestManager::scheduleDisruption(unsigned long delay, unsigned long duration) {
  if (currentTestMode != TEST_MODE_CONNECTION_DISRUPTION) {
    LOG_WARN(MODULE_SYS, "Disruption can only be scheduled in CONNECTION_DISRUPTION mode");
    return;
  }
  
  disruptionScheduled = true;
  disruptionStartTime = millis() + delay;
  disruptionDuration = duration;
  disruptionPhase = 0;
  
  LOG_INFO(MODULE_SYS, "Disruption scheduled in %lu seconds for %lu seconds", 
           delay / 1000, duration / 1000);
}

void NetworkTestManager::calculateAverages() {
  // Average response time
  if (metrics.successfulPackets > 0) {
    metrics.avgResponseTime = metrics.totalResponseTime / metrics.successfulPackets;
  }
  
  // Average recovery time
  if (metrics.disconnectCount > 0) {
    metrics.avgRecoveryTime = metrics.totalRecoveryTime / metrics.disconnectCount;
  }
}

const char* NetworkTestManager::getCurrentSignalCondition() {
  int csq = modemManager->getSignalQuality();
  
  for (int i = 0; i < 3; i++) {
    if (csq >= signalConditions[i].minRSSI && csq <= signalConditions[i].maxRSSI) {
      return signalConditions[i].name;
    }
  }
  
  return "UNKNOWN";
}

const char* NetworkTestManager::getTestModeName() const {
  switch (currentTestMode) {
    case TEST_MODE_CONTINUOUS_TRANSMISSION: return "CONTINUOUS_TRANSMISSION";
    case TEST_MODE_CONNECTION_DISRUPTION: return "CONNECTION_DISRUPTION";
    case TEST_MODE_LOAD_TEST: return "LOAD_TEST";
    case TEST_MODE_SIGNAL_MONITORING: return "SIGNAL_MONITORING";
    default: return "NONE";
  }
}

unsigned long NetworkTestManager::getTestElapsedTime() const {
  if (!testRunning) return 0;
  return millis() - testStartTime;
}

float NetworkTestManager::getTestProgress() const {
  if (!testRunning) return 0.0;
  return (float)getTestElapsedTime() / testDuration * 100.0;
}

void NetworkTestManager::printMetrics() {
  LOG_INFO(MODULE_SYS, "=== TEST METRICS ===");
  LOG_INFO(MODULE_SYS, "Test Duration: %s", Utils::formatUptime(metrics.testDuration).c_str());
  LOG_INFO(MODULE_SYS, "Signal Condition: %s (avg CSQ: %d)", 
           getCurrentSignalCondition(), metrics.avgSignalStrength);
  
  LOG_INFO(MODULE_SYS, "--- Transmission Metrics ---");
  LOG_INFO(MODULE_SYS, "Total Packets: %lu", metrics.totalPacketsSent);
  LOG_INFO(MODULE_SYS, "Successful: %lu (%.1f%%)", 
           metrics.successfulPackets, metrics.successRate);
  LOG_INFO(MODULE_SYS, "Failed: %lu", metrics.failedPackets);
  
  LOG_INFO(MODULE_SYS, "--- Timing Metrics ---");
  LOG_INFO(MODULE_SYS, "Avg Response Time: %lu ms", metrics.avgResponseTime);
  LOG_INFO(MODULE_SYS, "Min Response Time: %lu ms", metrics.minResponseTime);
  LOG_INFO(MODULE_SYS, "Max Response Time: %lu ms", metrics.maxResponseTime);
  
  LOG_INFO(MODULE_SYS, "--- Connection Stability ---");
  LOG_INFO(MODULE_SYS, "Uptime: %.1f%% (%s)", 
           metrics.uptimePercentage, 
           Utils::formatUptime(metrics.connectionUptime).c_str());
  LOG_INFO(MODULE_SYS, "Disconnections: %lu", metrics.disconnectCount);
  LOG_INFO(MODULE_SYS, "Avg Recovery Time: %lu ms", metrics.avgRecoveryTime);
  
  LOG_INFO(MODULE_SYS, "--- Signal Quality ---");
  LOG_INFO(MODULE_SYS, "Current CSQ: %d", metrics.currentSignalStrength);
  LOG_INFO(MODULE_SYS, "Min CSQ: %d", metrics.minSignalStrength);
  LOG_INFO(MODULE_SYS, "Max CSQ: %d", metrics.maxSignalStrength);
}

void NetworkTestManager::resetMetrics() {
  metrics = NetworkMetrics(); // Reset to default values
}

void NetworkTestManager::monitorSignalQuality() {
  static unsigned long lastLogTime = 0;
  unsigned long currentTime = millis();
  
  // Log signal quality every 10 seconds
  if (currentTime - lastLogTime >= 10000) {
    int csq = modemManager->getSignalQuality();
    LOG_INFO(MODULE_SYS, "Signal Monitor - CSQ: %d (%s), RSSI: ~-%d dBm", 
             csq, getCurrentSignalCondition(), 113 - (csq * 2));
    lastLogTime = currentTime;
  }
}

void NetworkTestManager::saveMetricsToFile() {
  // Create JSON document with test results
  StaticJsonDocument<1024> doc;
  
  doc["test_mode"] = getTestModeName();
  doc["duration_ms"] = metrics.testDuration;
  doc["signal_condition"] = getCurrentSignalCondition();
  
  JsonObject transmission = doc.createNestedObject("transmission");
  transmission["total_packets"] = metrics.totalPacketsSent;
  transmission["successful"] = metrics.successfulPackets;
  transmission["failed"] = metrics.failedPackets;
  transmission["success_rate"] = metrics.successRate;
  
  JsonObject timing = doc.createNestedObject("timing");
  timing["avg_response_ms"] = metrics.avgResponseTime;
  timing["min_response_ms"] = metrics.minResponseTime;
  timing["max_response_ms"] = metrics.maxResponseTime;
  
  JsonObject stability = doc.createNestedObject("stability");
  stability["uptime_percentage"] = metrics.uptimePercentage;
  stability["disconnections"] = metrics.disconnectCount;
  stability["avg_recovery_ms"] = metrics.avgRecoveryTime;
  
  JsonObject signal = doc.createNestedObject("signal");
  signal["avg_csq"] = metrics.avgSignalStrength;
  signal["min_csq"] = metrics.minSignalStrength;
  signal["max_csq"] = metrics.maxSignalStrength;
  
  // Log the JSON result
  String output;
  serializeJsonPretty(doc, output);
  LOG_INFO(MODULE_SYS, "Test Results JSON:\n%s", output.c_str());
}

void NetworkTestManager::exportTestResults() {
  saveMetricsToFile();
}