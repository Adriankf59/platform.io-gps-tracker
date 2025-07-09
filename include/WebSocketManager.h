// WebSocketManager.h - Updated for Database Compliance and Latency Calculation
#ifndef WEBSOCKET_MANAGER_H
#define WEBSOCKET_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "Logger.h"

// Forward declaration
class Utils;

// WebSocket buffer size constant (optimized)
#define WS_MAX_MESSAGE_SIZE 2048

// State WebSocket
enum WSState {
  WS_DISCONNECTED,    // Tidak terhubung
  WS_CONNECTING,      // Sedang menghubungkan
  WS_CONNECTED,       // Terhubung tapi belum subscribe
  WS_SUBSCRIBED      // Terhubung dan sudah subscribe
};

// Latency Tracking Structure (ENHANCED)
struct LatencyMeasurement {
  unsigned long transmissionStart;    // When transmission started
  unsigned long databaseReceived;     // When database confirmation received
  unsigned long endToEndLatency;     // Total latency (transmission to DB storage)
  bool measuring;                     // Currently measuring
  bool completed;                     // Measurement completed
  String transmissionId;              // Unique ID for tracking
  
  void reset() {
    transmissionStart = 0;
    databaseReceived = 0;
    endToEndLatency = 0;
    measuring = false;
    completed = false;
    transmissionId = "";
  }
  
  void start(const String& id) {
    transmissionStart = millis();
    transmissionId = id;
    measuring = true;
    completed = false;
    databaseReceived = 0;
    endToEndLatency = 0;
  }
  
  bool finish() {
    if (measuring && transmissionStart > 0) {
      databaseReceived = millis();
      endToEndLatency = databaseReceived - transmissionStart;
      measuring = false;
      completed = true;
      return true;
    }
    return false;
  }
  
  bool isValid() const {
    return completed && endToEndLatency > 0 && endToEndLatency < LATENCY_TIMEOUT;
  }
};

// Statistik koneksi dengan enhanced latency tracking
struct WSStats {
  unsigned long totalMessages;
  unsigned long totalBytesSent;
  unsigned long totalBytesReceived;
  unsigned long connectionTime;
  unsigned long lastMessageTime;
  int reconnectCount;
  
  // Enhanced latency metrics
  unsigned long totalLatency;
  unsigned long minLatency;
  unsigned long maxLatency;
  unsigned long latencySamples;
  unsigned long lastTransmissionStart;     // Added back for compatibility
  unsigned long endToEndLatency;           // NEW: Database storage latency
  unsigned long totalEndToEndLatency;      // NEW: Total end-to-end latency
  unsigned long minEndToEndLatency;        // NEW: Min database latency
  unsigned long maxEndToEndLatency;        // NEW: Max database latency
  unsigned long endToEndSamples;           // NEW: Number of end-to-end samples
  bool measuringLatency;
};

// Performance tracking
struct LatencyTracker {
  unsigned long samples[LATENCY_SAMPLE_SIZE];
  unsigned long endToEndSamples[LATENCY_SAMPLE_SIZE];  // NEW: End-to-end latency samples
  int currentIndex;
  int endToEndIndex;  // NEW: Index for end-to-end samples
  int sampleCount;
  int endToEndCount;  // NEW: Count of end-to-end samples
  unsigned long totalLatency;
  unsigned long totalEndToEndLatency;  // NEW: Total end-to-end latency
  bool initialized;
  
  void addSample(unsigned long latency) {
    if (!initialized) {
      memset(samples, 0, sizeof(samples));
      memset(endToEndSamples, 0, sizeof(endToEndSamples));
      currentIndex = 0;
      endToEndIndex = 0;
      sampleCount = 0;
      endToEndCount = 0;
      totalLatency = 0;
      totalEndToEndLatency = 0;
      initialized = true;
    }
    
    // Remove old sample if buffer is full
    if (sampleCount == LATENCY_SAMPLE_SIZE) {
      totalLatency -= samples[currentIndex];
    } else {
      sampleCount++;
    }
    
    // Add new sample
    samples[currentIndex] = latency;
    totalLatency += latency;
    currentIndex = (currentIndex + 1) % LATENCY_SAMPLE_SIZE;
  }
  
  void addEndToEndSample(unsigned long latency) {  // NEW: Add end-to-end sample
    if (!initialized) {
      memset(samples, 0, sizeof(samples));
      memset(endToEndSamples, 0, sizeof(endToEndSamples));
      currentIndex = 0;
      endToEndIndex = 0;
      sampleCount = 0;
      endToEndCount = 0;
      totalLatency = 0;
      totalEndToEndLatency = 0;
      initialized = true;
    }
    
    // Remove old sample if buffer is full
    if (endToEndCount == LATENCY_SAMPLE_SIZE) {
      totalEndToEndLatency -= endToEndSamples[endToEndIndex];
    } else {
      endToEndCount++;
    }
    
    // Add new sample
    endToEndSamples[endToEndIndex] = latency;
    totalEndToEndLatency += latency;
    endToEndIndex = (endToEndIndex + 1) % LATENCY_SAMPLE_SIZE;
  }
  
  unsigned long getAverage() const {
    return sampleCount > 0 ? totalLatency / sampleCount : 0;
  }
  
  unsigned long getEndToEndAverage() const {  // NEW: Get average end-to-end latency
    return endToEndCount > 0 ? totalEndToEndLatency / endToEndCount : 0;
  }
  
  unsigned long getMin() const {
    if (sampleCount == 0) return 0;
    unsigned long min_val = samples[0];
    for (int i = 1; i < sampleCount; i++) {
      if (samples[i] < min_val) min_val = samples[i];
    }
    return min_val;
  }
  
  unsigned long getMax() const {
    if (sampleCount == 0) return 0;
    unsigned long max_val = samples[0];
    for (int i = 1; i < sampleCount; i++) {
      if (samples[i] > max_val) max_val = samples[i];
    }
    return max_val;
  }
  
  unsigned long getEndToEndMin() const {  // NEW: Get min end-to-end latency
    if (endToEndCount == 0) return 0;
    unsigned long min_val = endToEndSamples[0];
    for (int i = 1; i < endToEndCount; i++) {
      if (endToEndSamples[i] < min_val) min_val = endToEndSamples[i];
    }
    return min_val;
  }
  
  unsigned long getEndToEndMax() const {  // NEW: Get max end-to-end latency
    if (endToEndCount == 0) return 0;
    unsigned long max_val = endToEndSamples[0];
    for (int i = 1; i < endToEndCount; i++) {
      if (endToEndSamples[i] > max_val) max_val = endToEndSamples[i];
    }
    return max_val;
  }
};

// Implementasi WebSocket Client Sederhana (Same as before)
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
  
  String generateWebSocketKey() {
    // Generate random 16-byte key dan encode ke base64
    String key = "";
    for(int i = 0; i < 22; i++) {
      key += char(random(65, 90)); // Simple random key
    }
    return key + "==";
  }
  
  bool performHandshake() {
    wsKey = generateWebSocketKey();
    
    LOG_DEBUG(MODULE_WS, "Memulai WebSocket handshake...");
    
    // Kirim WebSocket upgrade request (optimized headers)
    client->print("GET ");
    client->print(path);
    client->println(" HTTP/1.1");
    client->print("Host: ");
    client->println(host);
    client->println("Upgrade: websocket");
    client->println("Connection: Upgrade");
    client->print("Sec-WebSocket-Key: ");
    client->println(wsKey);
    client->println("Sec-WebSocket-Version: 13");
    client->println("Origin: http://esp32-tracker");
    client->println("User-Agent: ESP32-GPS-Tracker/2.0");
    
    // Optimize for low latency
    if (TCP_NODELAY) {
      client->println("Connection: keep-alive");
    }
    
    client->println();
    
    // Tunggu response dengan reduced timeout
    unsigned long timeout = millis() + WS_CONNECT_TIMEOUT;
    while (client->connected() && !client->available()) {
      if (millis() > timeout) {
        LOG_ERROR(MODULE_WS, "Handshake timeout");
        return false;
      }
      delay(5); // Reduced delay for faster response
    }
    
    // Baca response headers dengan faster timeout
    String response = "";
    response.reserve(512); // Pre-allocate for performance
    bool headerComplete = false;
    timeout = millis() + WS_RESPONSE_TIMEOUT;
    
    while (client->available() && !headerComplete && millis() < timeout) {
      char c = client->read();
      response += c;
      if (response.endsWith("\r\n\r\n")) {
        headerComplete = true;
      }
    }
    
    // Cek upgrade berhasil
    if (response.indexOf("101 Switching Protocols") != -1) {
      LOG_INFO(MODULE_WS, "✅ WebSocket handshake berhasil");
      connected = true;
      return true;
    }
    
    LOG_ERROR(MODULE_WS, "❌ Handshake gagal. Response: %.200s", response.c_str());
    return false;
  }
  
  void sendFrame(uint8_t opcode, const String& payload) {
    if (!connected) {
      LOG_WARN(MODULE_WS, "Tidak dapat mengirim frame - tidak terhubung");
      return;
    }
    
    size_t len = payload.length();
    
    // Validasi ukuran payload (optimized limit)
    if (len > MAX_PAYLOAD_SIZE) {
      LOG_ERROR(MODULE_WS, "Payload terlalu besar: %d bytes (max: %d)", len, MAX_PAYLOAD_SIZE);
      return;
    }
    
    // Kirim frame header (optimized for small payloads)
    client->write(0x80 | opcode); // FIN = 1
    
    // Payload length dengan masking bit (optimized for typical GPS payload size)
    if (len < 126) {
      client->write(0x80 | len); // Mask = 1
    } else {
      client->write(0x80 | 126);
      client->write((len >> 8) & 0xFF);
      client->write(len & 0xFF);
    }
    
    // Masking key (wajib untuk client) - optimized generation
    uint8_t mask[4];
    for (int i = 0; i < 4; i++) {
      mask[i] = random(0, 256);
      client->write(mask[i]);
    }
    
    // Kirim masked payload (optimized loop)
    const char* payloadData = payload.c_str();
    for (size_t i = 0; i < len; i++) {
      client->write(payloadData[i] ^ mask[i % 4]);
    }
    
    LOG_TRACE(MODULE_WS, "Frame dikirim: opcode=0x%X, len=%d", opcode, len);
  }
  
public:
  SimpleWebSocketClient(TinyGsmClient* gsmClient) : client(gsmClient), connected(false) {}
  
  bool connect(const String& url) {
    // Parse WebSocket URL
    if (url.startsWith("ws://")) {
      port = 80;
      host = url.substring(5);
    } else if (url.startsWith("wss://")) {
      port = 443;
      host = url.substring(6);
      LOG_ERROR(MODULE_WS, "WSS (SSL) belum didukung");
      return false;
    } else {
      LOG_ERROR(MODULE_WS, "URL WebSocket tidak valid: %s", url.c_str());
      return false;
    }
    
    // Extract path
    int pathIndex = host.indexOf('/');
    if (pathIndex > 0) {
      path = host.substring(pathIndex);
      host = host.substring(0, pathIndex);
    } else {
      path = "/";
    }
    
    LOG_INFO(MODULE_WS, "Menghubungkan ke ws://%s:%d%s", host.c_str(), port, path.c_str());
    
    // Connect TCP dengan optimized timeout
    if (!client->connect(host.c_str(), port)) {
      LOG_ERROR(MODULE_WS, "❌ Koneksi TCP gagal");
      return false;
    }
    
    LOG_DEBUG(MODULE_WS, "✅ TCP terhubung, melakukan handshake...");
    
    // Perform WebSocket handshake
    return performHandshake();
  }
  
  void disconnect() {
    if (connected) {
      LOG_INFO(MODULE_WS, "Memutuskan koneksi WebSocket...");
      // Kirim close frame
      sendFrame(WS_OPCODE_CLOSE, "");
      delay(50); // Reduced delay
      connected = false;
    }
    client->stop();
  }
  
  bool isConnected() {
    return connected && client->connected();
  }
  
  void sendText(const String& text) {
    sendFrame(WS_OPCODE_TEXT, text);
  }
  
  void sendPing() {
    sendFrame(WS_OPCODE_PING, "");
    LOG_TRACE(MODULE_WS, "Ping dikirim");
  }
  
  bool readMessage(String& message, unsigned long& bytesReceived) {
    if (!client->available()) return false;
    
    // Baca frame header
    uint8_t header = client->read();
    bool fin = (header & 0x80) != 0;
    uint8_t opcode = header & 0x0F;
    
    // Baca payload length
    uint8_t len1 = client->read();
    bool masked = (len1 & 0x80) != 0;
    size_t len = len1 & 0x7F;
    
    if (len == 126) {
      len = (client->read() << 8) | client->read();
    } else if (len == 127) {
      // Skip 8 bytes untuk 64-bit length (tidak support message besar)
      for (int i = 0; i < 8; i++) client->read();
      LOG_ERROR(MODULE_WS, "Message terlalu besar (64-bit length)");
      return false;
    }
    
    // Skip mask jika ada (server->client seharusnya tidak masked)
    if (masked) {
      for (int i = 0; i < 4; i++) client->read();
    }
    
    // Baca payload dengan size limit (optimized)
    message = "";
    size_t actualLen = min(len, (size_t)WS_MAX_MESSAGE_SIZE);
    message.reserve(actualLen + 1); // Pre-allocate for performance
    bytesReceived = actualLen;
    
    for (size_t i = 0; i < actualLen; i++) {
      if (client->available()) {
        message += (char)client->read();
      } else {
        // Tunggu data dengan reduced delay
        delay(5);
        if (client->available()) {
          message += (char)client->read();
        } else {
          LOG_WARN(MODULE_WS, "Message tidak lengkap (received %d/%d)", i, actualLen);
          break;
        }
      }
    }
    
    // Jika message lebih besar dari limit, buang sisanya
    if (len > WS_MAX_MESSAGE_SIZE) {
      LOG_WARN(MODULE_WS, "Message terpotong dari %d ke %d bytes", len, WS_MAX_MESSAGE_SIZE);
      size_t remaining = len - WS_MAX_MESSAGE_SIZE;
      while (remaining-- > 0 && client->available()) {
        client->read();
      }
    }
    
    // Handle opcode berbeda
    switch (opcode) {
      case WS_OPCODE_TEXT:
        LOG_TRACE(MODULE_WS, "Text message diterima: %d bytes", actualLen);
        return true;
        
      case WS_OPCODE_PING:
        LOG_DEBUG(MODULE_WS, "Ping diterima, mengirim pong");
        sendFrame(WS_OPCODE_PONG, message);
        return false;
        
      case WS_OPCODE_CLOSE:
        LOG_INFO(MODULE_WS, "Server mengirim close frame");
        connected = false;
        return false;
        
      default:
        LOG_WARN(MODULE_WS, "Opcode tidak dikenal: 0x%X", opcode);
        return false;
    }
  }
};

// WebSocket Manager Class (Enhanced for Database Compliance)
class WebSocketManager {
private:
  SimpleWebSocketClient* wsClient;
  TinyGsmClient* gsmClient;
  
  WSState state = WS_DISCONNECTED;
  unsigned long lastPingTime = 0;
  unsigned long lastReconnectAttempt = 0;
  int reconnectAttempts = 0;
  
  // Statistik dengan enhanced latency tracking
  WSStats stats = {0, 0, 0, 0, 0, 0, 0, UINT32_MAX, 0, 0, 0, 0, UINT32_MAX, 0, 0, false};
  LatencyTracker latencyTracker = {{0}, {0}, 0, 0, 0, 0, 0, 0, false};
  LatencyMeasurement currentMeasurement;  // NEW: Current latency measurement
  
  // Callbacks
  void (*onRelayUpdateCallback)(bool newState) = nullptr;
  void (*onDataConfirmationCallback)(const String& id, bool success, unsigned long latency) = nullptr;  // NEW
  
  // Konfigurasi (optimized)
  const char* wsUrl = WS_URL;
  const unsigned long PING_INTERVAL = WS_PING_INTERVAL;
  const unsigned long RECONNECT_BASE_DELAY = WS_RECONNECT_DELAY;
  const int MAX_RECONNECT_ATTEMPTS = 8; // Reduced from 10
  
  // Subscription management
  bool vehicleSubscribed = false;
  unsigned long lastSubscribeAttempt = 0;
  
  // Optimized payload buffer for reuse
  char payloadBuffer[MAX_PAYLOAD_SIZE];
  char websocketBuffer[MAX_PAYLOAD_SIZE + 100];  // Extra space for WebSocket wrapper
  
  void subscribeToVehicle() {
    if (millis() - lastSubscribeAttempt < 3000) { // Reduced from 5000
      LOG_DEBUG(MODULE_WS, "Subscribe attempt dibatasi (cooldown 3s)");
      return;
    }
    
    LOG_INFO(MODULE_WS, "Mengirim subscription request untuk vehicle...");
    
    // Gunakan JSON document kecil (optimized)
    StaticJsonDocument<200> doc; // Reduced from 256
    doc["type"] = "subscribe";
    doc["collection"] = "vehicle";
    
    JsonObject query = doc.createNestedObject("query");
    JsonArray fields = query.createNestedArray("fields");
    fields.add("*");
    
    String message;
    message.reserve(200); // Pre-allocate
    serializeJson(doc, message);
    
    wsClient->sendText(message);
    stats.totalBytesSent += message.length();
    stats.totalMessages++;
    lastSubscribeAttempt = millis();
    
    LOG_DEBUG(MODULE_WS, "Subscription request dikirim: %s", message.c_str());
  }
  
  void processMessage(const String& message) {
    // Log bagian awal message untuk debugging
    LOG_DEBUG(MODULE_WS, "Memproses message (len=%d): %.100s...", 
              message.length(), message.c_str());
    
    // Check for database confirmation first (NEW)
    if (message.indexOf("\"collection\":\"vehicle_datas\"") > 0 && 
        message.indexOf("\"action\":\"create\"") > 0) {
      processDatabaseConfirmation(message);
      return;
    }
    
    // Gunakan smaller JSON document untuk better performance
    DynamicJsonDocument doc(2048); // Reduced from 4096
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      LOG_ERROR(MODULE_WS, "JSON parse error: %s (msg len: %d)", 
                error.c_str(), message.length());
      
      // Jika NoMemory, coba handle basic message types
      if (error == DeserializationError::NoMemory) {
        handleLargeMessage(message);
      }
      return;
    }
    
    // Process normal JSON
    const char* type = doc["type"];
    
    if (strcmp(type, "subscription") == 0) {
      handleSubscriptionMessage(doc);
    } else if (strcmp(type, "auth") == 0) {
      handleAuthMessage(doc);
    } else if (strcmp(type, "error") == 0) {
      handleErrorMessage(doc);
    } else if (strcmp(type, "ping") == 0) {
      LOG_DEBUG(MODULE_WS, "Ping dari server diterima");
    } else {
      LOG_DEBUG(MODULE_WS, "Message type tidak dikenal: %s", type);
    }
  }
  
  // NEW: Process database confirmation for latency calculation
  void processDatabaseConfirmation(const String& message) {
    LOG_DEBUG(MODULE_WS, "📨 Database confirmation received");
    
    // Extract timestamp or ID from confirmation message
    // This is a simplified approach - in practice you'd parse the JSON properly
    if (currentMeasurement.measuring) {
      if (currentMeasurement.finish()) {
        unsigned long endToEndLatency = currentMeasurement.endToEndLatency;
        
        // Update statistics
        stats.endToEndSamples++;
        stats.totalEndToEndLatency += endToEndLatency;
        
        if (endToEndLatency < stats.minEndToEndLatency) {
          stats.minEndToEndLatency = endToEndLatency;
        }
        if (endToEndLatency > stats.maxEndToEndLatency) {
          stats.maxEndToEndLatency = endToEndLatency;
        }
        
        // Add to tracker
        latencyTracker.addEndToEndSample(endToEndLatency);
        
        LOG_INFO(MODULE_WS, "📊 End-to-end latency: %lu ms", endToEndLatency);
        
        // Call callback if set
        if (onDataConfirmationCallback) {
          onDataConfirmationCallback(currentMeasurement.transmissionId, true, endToEndLatency);
        }
        
        // Log if latency is high
        if (endToEndLatency > LATENCY_WARNING_THRESHOLD) {
          LOG_WARN(MODULE_PERF, "⚠️ High end-to-end latency: %lu ms", endToEndLatency);
        }
      }
    }
  }
  
  void handleSubscriptionMessage(JsonDocument& doc) {
    const char* event = doc["event"];
    
    if (strcmp(event, "init") == 0) {
      LOG_INFO(MODULE_WS, "✅ Subscription diinisialisasi");
      state = WS_SUBSCRIBED;
      vehicleSubscribed = true;
      processInitialData(doc["data"]);
    } else if (strcmp(event, "create") == 0 || strcmp(event, "update") == 0) {
      LOG_INFO(MODULE_WS, "📨 Vehicle update event: %s", event);
      processVehicleUpdate(doc["data"]);
    } else if (strcmp(event, "delete") == 0) {
      LOG_WARN(MODULE_WS, "⚠️ Vehicle delete event");
    }
  }
  
  void handleAuthMessage(JsonDocument& doc) {
    const char* status = doc["status"];
    if (strcmp(status, "ok") == 0) {
      LOG_INFO(MODULE_WS, "✅ Autentikasi berhasil");
      subscribeToVehicle();
    } else {
      LOG_ERROR(MODULE_WS, "❌ Autentikasi gagal: %s", status);
    }
  }
  
  void handleErrorMessage(JsonDocument& doc) {
    const char* errorMsg = doc["error"]["message"] | "Unknown error";
    const char* errorCode = doc["error"]["code"] | "UNKNOWN";
    LOG_ERROR(MODULE_WS, "❌ Server error [%s]: %s", errorCode, errorMsg);
    
    // Handle specific errors
    if (strcmp(errorCode, "SUBSCRIPTION_FAILED") == 0) {
      vehicleSubscribed = false;
      // Retry subscription after reduced delay
      lastSubscribeAttempt = millis() - 2000; // Allow retry in 1 second
    }
  }
  
  void handleLargeMessage(const String& message) {
    // Handle message yang terlalu besar untuk di-parse lengkap
    LOG_INFO(MODULE_WS, "Handling large message dengan parsing manual");
    
    // Extract message type
    int typeStart = message.indexOf("\"type\":\"");
    if (typeStart > 0) {
      typeStart += 8;
      int typeEnd = message.indexOf("\"", typeStart);
      if (typeEnd > typeStart) {
        String msgType = message.substring(typeStart, typeEnd);
        LOG_INFO(MODULE_WS, "Message type: %s (terlalu besar untuk parse penuh)", msgType.c_str());
        
        if (msgType == "subscription") {
          // Extract event type
          int eventStart = message.indexOf("\"event\":\"");
          if (eventStart > 0) {
            eventStart += 9;
            int eventEnd = message.indexOf("\"", eventStart);
            if (eventEnd > eventStart) {
              String eventType = message.substring(eventStart, eventEnd);
              if (eventType == "init") {
                LOG_INFO(MODULE_WS, "✅ Subscription confirmed (data terlalu besar)");
                state = WS_SUBSCRIBED;
                vehicleSubscribed = true;
                
                // Coba cari vehicle kita
                tryExtractVehicleData(message);
              }
            }
          }
        }
      }
    }
  }
  
  void tryExtractVehicleData(const String& message) {
    // Cari GPS ID kita dalam response
    int gpsIdPos = message.indexOf(GPS_ID);
    if (gpsIdPos > 0) {
      LOG_DEBUG(MODULE_WS, "Vehicle kita ditemukan dalam response");
      
      // Cari relay_status di dekat GPS ID
      int searchStart = max(0, gpsIdPos - 300);
      int searchEnd = min((int)message.length(), gpsIdPos + 100);
      
      int relayPos = message.indexOf("\"relay_status\"", searchStart);
      if (relayPos > 0 && relayPos < searchEnd) {
        relayPos = message.indexOf(":", relayPos) + 1;
        int relayEnd = message.indexOf(",", relayPos);
        if (relayEnd < 0) relayEnd = message.indexOf("}", relayPos);
        
        if (relayEnd > relayPos) {
          String relayValue = message.substring(relayPos, relayEnd);
          relayValue.trim();
          relayValue.replace("\"", "");
          
          bool newState = (relayValue == "ON");
          LOG_INFO(MODULE_WS, "Relay status ditemukan: %s", relayValue.c_str());
          
          if (onRelayUpdateCallback) {
            onRelayUpdateCallback(newState);
          }
        }
      }
    }
  }
  
  void processInitialData(JsonVariant data) {
    if (!data.is<JsonArray>()) {
      LOG_WARN(MODULE_WS, "Initial data bukan array");
      return;
    }
    
    JsonArray vehicles = data.as<JsonArray>();
    LOG_INFO(MODULE_WS, "Memproses %d vehicles", vehicles.size());
    
    for (JsonObject vehicle : vehicles) {
      String gpsId = vehicle["gps_id"] | "";
      if (gpsId == GPS_ID) {
        String relayStatus = vehicle["relay_status"] | "OFF";
        bool newState = (relayStatus == "ON");
        
        LOG_INFO(MODULE_WS, "✅ Vehicle kita ditemukan - Relay: %s", relayStatus.c_str());
        
        if (onRelayUpdateCallback) {
          onRelayUpdateCallback(newState);
        }
        break;
      }
    }
  }
  
  void processVehicleUpdate(JsonVariant data) {
    if (data.is<JsonArray>()) {
      JsonArray items = data.as<JsonArray>();
      
      for (JsonObject item : items) {
        processVehicleItem(item);
      }
    } else if (data.is<JsonObject>()) {
      // FIX: Buat variabel lokal untuk menghindari binding error
      JsonObject obj = data.as<JsonObject>();
      processVehicleItem(obj);
    }
  }
  
  // FIX: Ubah parameter menjadi const reference
  void processVehicleItem(const JsonObject& item) {
    String gpsId = item["gps_id"] | "";
    if (gpsId == GPS_ID) {
      String relayStatus = item["relay_status"] | "OFF";
      bool newState = (relayStatus == "ON");
      
      LOG_INFO(MODULE_WS, "📨 Relay status update: %s", relayStatus.c_str());
      
      if (onRelayUpdateCallback) {
        onRelayUpdateCallback(newState);
      }
      
      // Log info tambahan jika ada
      if (item.containsKey("updated_at")) {
        const char* updatedAt = item["updated_at"];
        LOG_DEBUG(MODULE_WS, "Update timestamp: %s", updatedAt);
      }
    }
  }
  
  // Format timestamp untuk server (ISO8601) - FIXED
  String formatTimestamp(unsigned long unixTime) {
    time_t rawTime = unixTime;
    struct tm *timeInfo = gmtime(&rawTime);
    
    char timestamp[32];
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            timeInfo->tm_year + 1900, 
            timeInfo->tm_mon + 1, 
            timeInfo->tm_mday,
            timeInfo->tm_hour, 
            timeInfo->tm_min, 
            timeInfo->tm_sec);
    
    return String(timestamp);
  }
  
  // Generate unique transmission ID (NEW)
  String generateTransmissionId() {
    return String(millis()) + "_" + String(random(1000, 9999));
  }
  
public:
  WebSocketManager(TinyGsmClient* client) : gsmClient(client) {
    wsClient = new SimpleWebSocketClient(client);
  }
  
  ~WebSocketManager() {
    delete wsClient;
  }
  
  void begin() {
    LOG_INFO(MODULE_WS, "WebSocket Manager diinisialisasi");
    LOG_INFO(MODULE_WS, "Target: %s", wsUrl);
    LOG_INFO(MODULE_WS, "Ping Interval: %lu ms", PING_INTERVAL);
    LOG_INFO(MODULE_WS, "Max Payload Size: %d bytes", MAX_PAYLOAD_SIZE);
    
    // Initialize performance tracking
    if (ENABLE_LATENCY_MONITORING) {
      LOG_INFO(MODULE_WS, "Performance monitoring enabled");
      latencyTracker.initialized = false;
    }
    
    // Initialize latency calculation
    if (ENABLE_LATENCY_CALCULATION) {
      LOG_INFO(MODULE_WS, "End-to-end latency calculation enabled");
      currentMeasurement.reset();
    }
  }
  
  void setOnRelayUpdate(void (*callback)(bool)) {
    onRelayUpdateCallback = callback;
    LOG_DEBUG(MODULE_WS, "Relay update callback diset");
  }
  
  // NEW: Set callback for data confirmation
  void setOnDataConfirmation(void (*callback)(const String&, bool, unsigned long)) {
    onDataConfirmationCallback = callback;
    LOG_DEBUG(MODULE_WS, "Data confirmation callback diset");
  }
  
  bool connect() {
    if (state != WS_DISCONNECTED) {
      LOG_WARN(MODULE_WS, "Sudah terhubung atau sedang menghubungkan");
      return false;
    }
    
    LOG_INFO(MODULE_WS, "🔌 Mencoba koneksi WebSocket...");
    state = WS_CONNECTING;
    
    if (wsClient->connect(wsUrl)) {
      LOG_INFO(MODULE_WS, "✅ WebSocket terhubung");
      state = WS_CONNECTED;
      stats.connectionTime = millis();
      lastPingTime = millis();
      reconnectAttempts = 0;
      vehicleSubscribed = false;
      
      // Subscribe segera
      subscribeToVehicle();
      
      return true;
    }
    
    LOG_ERROR(MODULE_WS, "❌ Koneksi WebSocket gagal");
    state = WS_DISCONNECTED;
    stats.reconnectCount++;
    return false;
  }
  
  void disconnect();  // Implemented in cpp file
  
  void update() {
    if (state == WS_DISCONNECTED) {
      // Handle reconnection dengan reduced exponential backoff
      unsigned long now = millis();
      unsigned long delay = RECONNECT_BASE_DELAY * (1 << min(reconnectAttempts, 4)); // Reduced from 5
      delay = min(delay, 30000UL); // Reduced max from 60s to 30s
      
      if (now - lastReconnectAttempt >= delay) {
        LOG_INFO(MODULE_WS, "🔄 Percobaan reconnect #%d (delay: %lu ms)", 
                 reconnectAttempts + 1, delay);
        lastReconnectAttempt = now;
        reconnectAttempts++;
        
        if (reconnectAttempts > MAX_RECONNECT_ATTEMPTS) {
          LOG_ERROR(MODULE_WS, "❌ Max reconnect attempts reached, perlu reset manual");
          reconnectAttempts = MAX_RECONNECT_ATTEMPTS; // Cap it
        } else {
          connect();
        }
      }
      return;
    }
    
    if (!wsClient->isConnected()) {
      LOG_WARN(MODULE_WS, "⚠️ Koneksi WebSocket terputus");
      state = WS_DISCONNECTED;
      vehicleSubscribed = false;
      return;
    }
    
    // Kirim ping untuk keep-alive dengan optimized interval
    if (millis() - lastPingTime >= PING_INTERVAL) {
      wsClient->sendPing();
      lastPingTime = millis();
    }
    
    // Re-subscribe jika perlu dengan reduced timeout
    if (state == WS_CONNECTED && !vehicleSubscribed && 
        millis() - lastSubscribeAttempt > 7000) { // Reduced from 10000
      LOG_WARN(MODULE_WS, "⚠️ Belum subscribe, mencoba lagi...");
      subscribeToVehicle();
    }
    
    // Baca incoming messages
    String message;
    unsigned long bytesReceived = 0;
    if (wsClient->readMessage(message, bytesReceived)) {
      stats.totalBytesReceived += bytesReceived;
      stats.lastMessageTime = millis();
      processMessage(message);
    }
    
    // Check for latency measurement timeout (NEW)
    if (ENABLE_LATENCY_CALCULATION && currentMeasurement.measuring) {
      if (millis() - currentMeasurement.transmissionStart > LATENCY_TIMEOUT) {
        LOG_WARN(MODULE_WS, "⚠️ Latency measurement timeout for ID: %s", 
                 currentMeasurement.transmissionId.c_str());
        
        if (onDataConfirmationCallback) {
          onDataConfirmationCallback(currentMeasurement.transmissionId, false, (unsigned long)LATENCY_INVALID_VALUE);
        }
        
        currentMeasurement.reset();
      }
    }
  }
  
  // ENHANCED: Send vehicle data with database-compliant format and signal monitoring
  bool sendVehicleData(float lat, float lon, float speed, int satellites, 
                       const String& timestamp, float rsrq = RSRQ_INVALID_VALUE, 
                       float rsrp = RSRP_INVALID_VALUE) {
    if (!isReady()) {
      LOG_WARN(MODULE_WS, "⚠️ Tidak siap mengirim data (state: %s)", getStateString());
      return false;
    }
    
    // Generate transmission ID for latency tracking
    String transmissionId = generateTransmissionId();
    
    // Start latency measurement
    if (ENABLE_LATENCY_MONITORING) {
      stats.lastTransmissionStart = millis();
      stats.measuringLatency = true;
    }
    
    // Start end-to-end latency measurement
    if (ENABLE_LATENCY_CALCULATION) {
      currentMeasurement.start(transmissionId);
    }
    
    // Create database-compliant payload
    char latStr[16], lngStr[16];
    snprintf(latStr, sizeof(latStr), "%.5f", lat);
    snprintf(lngStr, sizeof(lngStr), "%.5f", lon);
    
    // Calculate preliminary latency (transmission preparation time)
    float preliminaryLatency = LATENCY_INVALID_VALUE;
    if (ENABLE_LATENCY_CALCULATION && currentMeasurement.measuring) {
      preliminaryLatency = millis() - currentMeasurement.transmissionStart;
    }
    
    String dataPayload;
    dataPayload.reserve(MAX_PAYLOAD_SIZE);
    
    // Choose payload format based on available data
    if (rsrq != RSRQ_INVALID_VALUE && rsrp != RSRP_INVALID_VALUE) {
      // Full payload with signal monitoring
      snprintf(payloadBuffer, sizeof(payloadBuffer), 
        DB_FULL_PAYLOAD_TEMPLATE,
        latStr, lngStr, (int)round(speed), satellites, timestamp.c_str(),
        rsrq, rsrp, preliminaryLatency
      );
    } else {
      // Essential payload without signal data
      snprintf(payloadBuffer, sizeof(payloadBuffer),
        DB_ESSENTIAL_PAYLOAD_TEMPLATE,
        latStr, lngStr, (int)round(speed), satellites, timestamp.c_str()
      );
    }
    
    // Wrap for WebSocket transmission
    snprintf(websocketBuffer, sizeof(websocketBuffer),
      WS_DB_PAYLOAD_WRAPPER, payloadBuffer);
    
    String message = String(websocketBuffer);
    
    // Log payload info
    if (DEBUG_PAYLOAD_SIZE) {
      LOG_DEBUG(MODULE_WS, "📤 Database payload size: %d bytes", message.length());
      if (rsrq != RSRQ_INVALID_VALUE && rsrp != RSRP_INVALID_VALUE) {
        LOG_DEBUG(MODULE_WS, "📶 Signal metrics: RSRQ=%.2f dB, RSRP=%.2f dBm", rsrq, rsrp);
      }
    }
    
    LOG_DEBUG(MODULE_WS, "📤 Mengirim: %s", message.c_str());
    wsClient->sendText(message);
    
    stats.totalBytesSent += message.length();
    stats.totalMessages++;
    
    return true;
  }
  
  // Alternative compact send method (UPDATED for database compliance)
  bool sendVehicleDataCompact(float lat, float lon, float speed, int satellites, 
                             unsigned long unixTimestamp, float rsrq = RSRQ_INVALID_VALUE, 
                             float rsrp = RSRP_INVALID_VALUE) {
    if (!isReady()) {
      LOG_WARN(MODULE_WS, "⚠️ Tidak siap mengirim data (state: %s)", getStateString());
      return false;
    }
    
    // Format timestamp in ISO8601 format
    String timestamp = formatTimestamp(unixTimestamp);
    
    // Call main send function
    return sendVehicleData(lat, lon, speed, satellites, timestamp, rsrq, rsrp);
  }
  
  // Performance monitoring methods
  void endLatencyMeasurement() {
    if (ENABLE_LATENCY_MONITORING && stats.measuringLatency) {
      unsigned long latency = millis() - stats.lastTransmissionStart;
      
      // Update statistics
      stats.totalLatency += latency;
      stats.latencySamples++;
      
      if (latency < stats.minLatency) stats.minLatency = latency;
      if (latency > stats.maxLatency) stats.maxLatency = latency;
      
      // Add to latency tracker
      latencyTracker.addSample(latency);
      
      stats.measuringLatency = false;
      
      // Log performance metrics
      if (DEBUG_LATENCY_TRACKING) {
        LOG_DEBUG(MODULE_PERF, "📊 Transmission latency: %lu ms (avg: %lu ms)", 
                  latency, latencyTracker.getAverage());
      }
      
      // Warn if latency is high
      if (latency > LATENCY_WARNING_THRESHOLD) {
        LOG_WARN(MODULE_PERF, "⚠️ High latency detected: %lu ms (threshold: %d ms)", 
                 latency, LATENCY_WARNING_THRESHOLD);
      }
    }
  }
  
  void startLatencyMeasurement() {
    if (ENABLE_LATENCY_MONITORING) {
      stats.lastTransmissionStart = millis();
      stats.measuringLatency = true;
    }
  }
  
  unsigned long getAverageLatency() {
    return latencyTracker.getAverage();
  }
  
  unsigned long getMinLatency() {
    return latencyTracker.getMin();
  }
  
  unsigned long getMaxLatency() {
    return latencyTracker.getMax();
  }
  
  // NEW: End-to-end latency methods
  unsigned long getEndToEndAverageLatency() {
    return latencyTracker.getEndToEndAverage();
  }
  
  unsigned long getEndToEndMinLatency() {
    return latencyTracker.getEndToEndMin();
  }
  
  unsigned long getEndToEndMaxLatency() {
    return latencyTracker.getEndToEndMax();
  }
  
  String getPerformanceReport() {
    String report = "=== WEBSOCKET PERFORMANCE ===\n";
    report += "State: " + String(getStateString()) + "\n";
    report += "Total Messages: " + String(stats.totalMessages) + "\n";
    report += "Bytes Sent: " + String(stats.totalBytesSent) + " (" + String(stats.totalBytesSent/1024) + " KB)\n";
    report += "Bytes Received: " + String(stats.totalBytesReceived) + " (" + String(stats.totalBytesReceived/1024) + " KB)\n";
    report += "Reconnect Count: " + String(stats.reconnectCount) + "\n";
    
    if (ENABLE_LATENCY_MONITORING && stats.latencySamples > 0) {
      report += "=== TRANSMISSION LATENCY ===\n";
      report += "Samples: " + String(stats.latencySamples) + "\n";
      report += "Average: " + String(getAverageLatency()) + " ms\n";
      report += "Min: " + String(getMinLatency()) + " ms\n";
      report += "Max: " + String(getMaxLatency()) + " ms\n";
    }
    
    if (ENABLE_LATENCY_CALCULATION && stats.endToEndSamples > 0) {
      report += "=== END-TO-END LATENCY ===\n";
      report += "Samples: " + String(stats.endToEndSamples) + "\n";
      report += "Average: " + String(getEndToEndAverageLatency()) + " ms\n";
      report += "Min: " + String(getEndToEndMinLatency()) + " ms\n";
      report += "Max: " + String(getEndToEndMaxLatency()) + " ms\n";
      
      if (getEndToEndAverageLatency() <= MAX_ACCEPTABLE_LATENCY) {
        report += "Performance: ✅ GOOD\n";
      } else {
        report += "Performance: ⚠️ NEEDS OPTIMIZATION\n";
      }
    }
    
    return report;
  }
  
  void resetPerformanceStats() {
    memset(&stats, 0, sizeof(stats));
    stats.minLatency = UINT32_MAX;
    stats.minEndToEndLatency = UINT32_MAX;
    latencyTracker.initialized = false;
    currentMeasurement.reset();
    LOG_INFO(MODULE_PERF, "Performance statistics reset");
  }
  
  // Connection health monitoring
  void maintainConnection() {
    static unsigned long lastHealthCheck = 0;
    unsigned long now = millis();
    
    if (now - lastHealthCheck < CONNECTION_HEALTH_CHECK_INTERVAL) {
      return;
    }
    
    lastHealthCheck = now;
    
    // Check connection health
    if (state == WS_SUBSCRIBED && wsClient->isConnected()) {
      // Connection is healthy
      if (DEBUG_NETWORK_QUALITY) {
        LOG_DEBUG(MODULE_WS, "📡 Connection health: GOOD");
      }
    } else if (state != WS_DISCONNECTED) {
      // Connection issues detected
      LOG_WARN(MODULE_WS, "🔧 Connection health issue detected, attempting recovery");
      
      if (!wsClient->isConnected()) {
        // TCP connection lost
        state = WS_DISCONNECTED;
        vehicleSubscribed = false;
      } else if (state == WS_CONNECTED && !vehicleSubscribed) {
        // WebSocket connected but not subscribed
        subscribeToVehicle();
      }
    }
    
    // Performance-based optimization triggers
    if (ENABLE_LATENCY_BASED_OPTIMIZATION && stats.latencySamples >= 3) {
      unsigned long avgLatency = getAverageLatency();
      if (avgLatency > MAX_ACCEPTABLE_LATENCY) {
        LOG_WARN(MODULE_PERF, "🚀 High latency detected (%lu ms), triggering optimization", avgLatency);
        // Trigger optimization in ModemManager if available
        // This could be a callback to ModemManager::reapplyOptimizations()
      }
    }
  }
  
  bool isReady() {
    return state == WS_SUBSCRIBED && wsClient->isConnected();
  }
  
  WSState getState() {
    return state;
  }
  
  const char* getStateString() {
    switch (state) {
      case WS_DISCONNECTED: return "DISCONNECTED";
      case WS_CONNECTING: return "CONNECTING";
      case WS_CONNECTED: return "CONNECTED";
      case WS_SUBSCRIBED: return "SUBSCRIBED";
      default: return "UNKNOWN";
    }
  }
  
  const WSStats& getStats() {
    return stats;
  }
  
  void resetReconnectAttempts() {
    reconnectAttempts = 0;
    LOG_INFO(MODULE_WS, "Reconnect attempts direset");
  }
  
  // Diagnostic methods
  bool isPerformanceGood() {
    if (!ENABLE_LATENCY_MONITORING || stats.latencySamples < 3) {
      return true; // Not enough data
    }
    return getAverageLatency() <= MAX_ACCEPTABLE_LATENCY;
  }
  
  bool isEndToEndPerformanceGood() {  // NEW
    if (!ENABLE_LATENCY_CALCULATION || stats.endToEndSamples < 3) {
      return true; // Not enough data
    }
    return getEndToEndAverageLatency() <= MAX_ACCEPTABLE_LATENCY;
  }
  
  size_t getLastPayloadSize() {
    return strlen(payloadBuffer);
  }
  
  // Force immediate ping for testing
  void forcePing() {
    if (wsClient && wsClient->isConnected()) {
      wsClient->sendPing();
      LOG_INFO(MODULE_WS, "🏓 Force ping sent");
    }
  }
  
  // NEW: Get current measurement info
  const LatencyMeasurement& getCurrentMeasurement() const {
    return currentMeasurement;
  }
  
  // NEW: Check if latency measurement is active
  bool isLatencyMeasuring() const {
    return currentMeasurement.measuring;
  }
};

#endif // WEBSOCKET_MANAGER_H