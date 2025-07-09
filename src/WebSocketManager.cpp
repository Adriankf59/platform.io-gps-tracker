// WebSocketManager.cpp - WebSocket Implementation with Utils dependency
#include "WebSocketManager.h"
#include "Utils.h"

// Implementation of methods that require Utils

void WebSocketManager::disconnect() {
  if (state != WS_DISCONNECTED) {
    LOG_INFO(MODULE_WS, "🔌 Memutuskan WebSocket...");
    wsClient->disconnect();
    state = WS_DISCONNECTED;
    vehicleSubscribed = false;
    
    // Log statistik koneksi
    if (stats.connectionTime > 0) {
      unsigned long duration = millis() - stats.connectionTime;
      LOG_INFO(MODULE_WS, "📊 Durasi koneksi: %s", Utils::formatUptime(duration).c_str());
      LOG_INFO(MODULE_WS, "📊 Messages: %lu, Sent: %lu KB, Received: %lu KB",
               stats.totalMessages,
               stats.totalBytesSent / 1024,
               stats.totalBytesReceived / 1024);
      
      if (ENABLE_LATENCY_MONITORING && stats.latencySamples > 0) {
        LOG_INFO(MODULE_WS, "📊 Avg Latency: %lu ms, Min: %lu ms, Max: %lu ms",
                 latencyTracker.getAverage(),
                 latencyTracker.getMin(),
                 latencyTracker.getMax());
      }
      
      if (ENABLE_LATENCY_CALCULATION && stats.endToEndSamples > 0) {
        LOG_INFO(MODULE_WS, "📊 End-to-End Avg: %lu ms, Min: %lu ms, Max: %lu ms",
                 latencyTracker.getEndToEndAverage(),
                 latencyTracker.getEndToEndMin(),
                 latencyTracker.getEndToEndMax());
      }
    }
  }
}