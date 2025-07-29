// WebSocketManager.h - Enhanced WebSocket Manager (DECLARATION ONLY)
#ifndef WEBSOCKET_MANAGER_H
#define WEBSOCKET_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// WebSocket buffer size constant (optimized)
#define WS_MAX_MESSAGE_SIZE 2048

// State WebSocket
enum WSState {
  WS_DISCONNECTED,    // Tidak terhubung
  WS_CONNECTING,      // Sedang menghubungkan
  WS_CONNECTED,       // Terhubung tapi belum subscribe
  WS_SUBSCRIBED      // Terhubung dan sudah subscribe
};

// Statistik koneksi dengan performance metrics
struct WSStats {
  unsigned long totalMessages;
  unsigned long totalBytesSent;
  unsigned long totalBytesReceived;
  unsigned long connectionTime;
  unsigned long lastMessageTime;
  int reconnectCount;
  
  // Performance metrics
  unsigned long totalLatency;
  unsigned long minLatency;
  unsigned long maxLatency;
  unsigned long latencySamples;
  unsigned long lastTransmissionStart;
  bool measuringLatency;
  
  // Health metrics
  unsigned long lastSuccessfulTransmission;
  unsigned long consecutiveFailures;
  unsigned long totalConnectionTime;
  unsigned long longestConnection;
};

// Performance tracking
struct LatencyTracker {
  unsigned long samples[LATENCY_SAMPLE_SIZE];
  int currentIndex;
  int sampleCount;
  unsigned long totalLatency;
  bool initialized;
  
  void addSample(unsigned long latency);
  unsigned long getAverage() const;
  unsigned long getMin() const;
  unsigned long getMax() const;
  void reset();
};

// Forward declaration
class SimpleWebSocketClient;

// WebSocket Manager Class (DECLARATION ONLY)
class WebSocketManager {
private:
  SimpleWebSocketClient* wsClient;
  TinyGsmClient* gsmClient;
  
  WSState state;
  unsigned long lastPingTime;
  unsigned long lastReconnectAttempt;
  int reconnectAttempts;
  
  // Statistics
  WSStats stats;
  LatencyTracker latencyTracker;
  
  // Health monitoring
  unsigned long connectionStartTime;
  unsigned long lastHealthCheck;
  int consecutiveHealthFailures;
  bool forceRestart;
  unsigned long restartDelayTimer;
  
  // Callbacks
  void (*onRelayUpdateCallback)(bool newState);
  
  // Configuration
  const char* wsUrl;
  const unsigned long PING_INTERVAL;
  const unsigned long RECONNECT_BASE_DELAY;
  const int MAX_RECONNECT_ATTEMPTS;
  
  // Subscription management
  bool vehicleSubscribed;
  unsigned long lastSubscribeAttempt;
  
  // Payload buffer for reuse
  char payloadBuffer[MAX_PAYLOAD_SIZE];
  
  // Private methods (DECLARATIONS ONLY)
  void subscribeToVehicle();
  void processMessage(const String& message);
  void handleSubscriptionMessage(JsonDocument& doc);
  void handleAuthMessage(JsonDocument& doc);
  void handleErrorMessage(JsonDocument& doc);
  void handleLargeMessage(const String& message);
  void tryExtractVehicleData(const String& message);
  void processInitialData(JsonVariant data);
  void processVehicleUpdate(JsonVariant data);
  void processVehicleItem(const JsonObject& item);
  String formatTimestamp(unsigned long unixTime);
  void performHealthCheck();
  
public:
  // Constructor & Destructor
  WebSocketManager(TinyGsmClient* client);
  ~WebSocketManager();
  
  // Core methods
  void begin();
  bool connect();
  void disconnect();
  void update();
  void maintainConnection();
  
  // Data transmission
  bool sendVehicleData(float lat, float lon, float speed, int satellites, 
                       const String& timestamp, float battery = 12.5);
  
  // Performance monitoring
  void startLatencyMeasurement();
  void endLatencyMeasurement();
  unsigned long getAverageLatency();
  unsigned long getMinLatency();
  unsigned long getMaxLatency();
  String getPerformanceReport();
  void resetPerformanceStats();
  
  // Status methods
  bool isReady();
  WSState getState();
  const char* getStateString();
  const WSStats& getStats();
  bool isPerformanceGood();
  bool isHealthy();
  String getHealthStatus();
  
  // Utility methods
  void setOnRelayUpdate(void (*callback)(bool));
  void resetReconnectAttempts();
  void forcePing();
  unsigned long getConnectionUptime();
  bool shouldForceRestart();
  int getPayloadMode();
  size_t getLastPayloadSize();
};

// SimpleWebSocketClient class (DECLARATION ONLY)
class SimpleWebSocketClient {
private:
  TinyGsmClient* client;
  String host;
  String path;
  int port;
  bool connected;
  String wsKey;
  
  // WebSocket frame opcodes
  static const uint8_t WS_OPCODE_TEXT = 0x1;
  static const uint8_t WS_OPCODE_CLOSE = 0x8;
  static const uint8_t WS_OPCODE_PING = 0x9;
  static const uint8_t WS_OPCODE_PONG = 0xA;
  
  String generateWebSocketKey();
  bool performHandshake();
  void sendFrame(uint8_t opcode, const String& payload);
  
public:
  SimpleWebSocketClient(TinyGsmClient* gsmClient);
  
  bool connect(const String& url);
  void disconnect();
  bool isConnected();
  void sendText(const String& text);
  void sendPing();
  bool readMessage(String& message, unsigned long& bytesReceived);
};

#endif // WEBSOCKET_MANAGER_H