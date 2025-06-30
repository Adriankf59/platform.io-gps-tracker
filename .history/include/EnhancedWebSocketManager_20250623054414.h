// EnhancedWebSocketManager.h - WebSocket Manager for High-Frequency Updates
#ifndef ENHANCED_WEBSOCKET_MANAGER_H
#define ENHANCED_WEBSOCKET_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "Logger.h"
#include "SimpleWebSocketClient.h"

// Update modes
enum UpdateMode {
  UPDATE_MODE_NORMAL,      // Standard updates
  UPDATE_MODE_REALTIME,    // High-frequency updates
  UPDATE_MODE_BATCH        // Batch multiple updates
};

// Position update structure
struct PositionUpdate {
  double latitude;
  double longitude;
  float speed;
  float course;
  float accuracy;
  String timestamp;
  unsigned long localTime;
};

class EnhancedWebSocketManager {
private:
  SimpleWebSocketClient* wsClient;
  TinyGsmClient* gsmClient;
  
  // Connection state
  WSState state = WS_DISCONNECTED;
  unsigned long lastPingTime = 0;
  unsigned long lastReconnectAttempt = 0;
  int reconnectAttempts = 0;
  
  // Update management
  UpdateMode currentMode = UPDATE_MODE_NORMAL;
  unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 1000; // Default 1 second
  
  // Batch management
  static const int MAX_BATCH_SIZE = 10;
  PositionUpdate batchBuffer[MAX_BATCH_SIZE];
  int batchCount = 0;
  unsigned long batchStartTime = 0;
  static const unsigned long BATCH_TIMEOUT = 500; // Send batch after 500ms
  
  // Throttling
  unsigned long minUpdateInterval = 200; // Minimum 200ms between updates
  bool throttlingEnabled = true;
  
  // Delta compression
  PositionUpdate lastSentPosition;
  bool lastPositionValid = false;
  float minDistanceThreshold = 1.0; // meters
  float minSpeedChangeThreshold = 0.5; // km/h
  
  // Statistics
  unsigned long totalUpdatesSent = 0;
  unsigned long totalBatchesSent = 0;
  unsigned long totalBytesOptimized = 0;
  
  // Callbacks
  void (*onRelayUpdateCallback)(bool newState) = nullptr;
  
  // Private methods
  bool shouldSendUpdate(const PositionUpdate& current);
  void sendBatch();
  void sendSingleUpdate(const PositionUpdate& update);
  void processMessage(const String& message);
  void subscribeToVehicle();
  String createCompactUpdate(const PositionUpdate& update, bool useDelta);
  
public:
  EnhancedWebSocketManager(TinyGsmClient* client);
  ~EnhancedWebSocketManager();
  
  void begin();
  void setOnRelayUpdate(void (*callback)(bool));
  
  // Connection management
  bool connect();
  void disconnect();
  void update();
  
  // Update modes
  void setUpdateMode(UpdateMode mode);
  void setUpdateInterval(unsigned long interval);
  void setMinUpdateInterval(unsigned long interval) { minUpdateInterval = interval; }
  void enableThrottling(bool enable) { throttlingEnabled = enable; }
  
  // Position updates
  bool sendPositionUpdate(double lat, double lon, float speed, float course, 
                         float accuracy, const String& timestamp);
  
  // Batch updates
  void addToBatch(double lat, double lon, float speed, float course, 
                  float accuracy, const String& timestamp);
  void flushBatch();
  
  // Delta compression settings
  void setMinDistanceThreshold(float meters) { minDistanceThreshold = meters; }
  void setMinSpeedChangeThreshold(float kmh) { minSpeedChangeThreshold = kmh; }
  
  // Status
  bool isReady() { return state == WS_SUBSCRIBED && wsClient->isConnected(); }
  WSState getState() { return state; }
  const char* getStateString();
  
  // Statistics
  unsigned long getTotalUpdatesSent() { return totalUpdatesSent; }
  unsigned long getTotalBatchesSent() { return totalBatchesSent; }
  unsigned long getBytesOptimized() { return totalBytesOptimized; }
  float getAverageUpdateRate();
};

// Simplified WebSocket client (reuse from original or create new)
class SimpleWebSocketClient {
  // ... (same as before or simplified version)
};

#endif // ENHANCED_WEBSOCKET_MANAGER_H