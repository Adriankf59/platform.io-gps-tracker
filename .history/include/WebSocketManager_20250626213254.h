// WebSocketManager.h - Manajer WebSocket untuk Real-time Communication
#ifndef WEBSOCKET_MANAGER_H
#define WEBSOCKET_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "Logger.h"

// State WebSocket
enum WSState {
  WS_DISCONNECTED,    // Tidak terhubung
  WS_CONNECTING,      // Sedang menghubungkan
  WS_CONNECTED,       // Terhubung tapi belum subscribe
  WS_SUBSCRIBED      // Terhubung dan sudah subscribe
};

// Statistik koneksi
struct WSStats {
  unsigned long totalMessages;
  unsigned long totalBytesSent;
  unsigned long totalBytesReceived;
  unsigned long connectionTime;
  unsigned long lastMessageTime;
  int reconnectCount;
};

// Implementasi WebSocket Client Sederhana
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
  
  // Buffer settings
  static constexpr size_t MAX_MESSAGE_SIZE = 4096;  // Increased from 2048
  
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
    
    // Kirim WebSocket upgrade request
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
    client->println("User-Agent: ESP32-GPS-Tracker/1.0");
    client->println();
    
    // Tunggu response
    unsigned long timeout = millis() + 5000;
    while (client->connected() && !client->available()) {
      if (millis() > timeout) {
        LOG_ERROR(MODULE_WS, "Handshake timeout");
        return false;
      }
      delay(10);
    }
    
    // Baca response headers
    String response = "";
    bool headerComplete = false;
    timeout = millis() + 3000;
    
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
    
    // Validasi ukuran payload
    if (len > 65535) {
      LOG_ERROR(MODULE_WS, "Payload terlalu besar: %d bytes", len);
      return;
    }
    
    // Kirim frame header
    client->write(0x80 | opcode); // FIN = 1
    
    // Payload length dengan masking bit
    if (len < 126) {
      client->write(0x80 | len); // Mask = 1
    } else if (len < 65516) {
      client->write(0x80 | 126);
      client->write((len >> 8) & 0xFF);
      client->write(len & 0xFF);
    }
    
    // Masking key (wajib untuk client)
    uint8_t mask[4];
    for (int i = 0; i < 4; i++) {
      mask[i] = random(0, 256);
      client->write(mask[i]);
    }
    
    // Kirim masked payload
    for (size_t i = 0; i < len; i++) {
      client->write(payload[i] ^ mask[i % 4]);
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
    
    // Connect TCP
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
      delay(100);
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
    
    // Baca payload dengan size limit
    message = "";
    message.reserve(min(len, MAX_MESSAGE_SIZE) + 1);
    
    size_t actualLen = min(len, MAX_MESSAGE_SIZE);
    bytesReceived = actualLen;
    
    for (size_t i = 0; i < actualLen; i++) {
      if (client->available()) {
        message += (char)client->read();
      } else {
        // Tunggu data
        delay(10);
        if (client->available()) {
          message += (char)client->read();
        } else {
          LOG_WARN(MODULE_WS, "Message tidak lengkap (received %d/%d)", i, actualLen);
          break;
        }
      }
    }
    
    // Jika message lebih besar dari limit, buang sisanya
    if (len > MAX_MESSAGE_SIZE) {
      LOG_WARN(MODULE_WS, "Message terpotong dari %d ke %d bytes", len, MAX_MESSAGE_SIZE);
      size_t remaining = len - MAX_MESSAGE_SIZE;
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

// WebSocket Manager Class
class WebSocketManager {
private:
  SimpleWebSocketClient* wsClient;
  TinyGsmClient* gsmClient;
  
  WSState state = WS_DISCONNECTED;
  unsigned long lastPingTime = 0;
  unsigned long lastReconnectAttempt = 0;
  int reconnectAttempts = 0;
  
  // Statistik
  WSStats stats = {0, 0, 0, 0, 0, 0};
  
  // Callbacks
  void (*onRelayUpdateCallback)(bool newState) = nullptr;
  
  // Konfigurasi
  const char* wsUrl = WS_URL;
  const unsigned long PING_INTERVAL = WS_PING_INTERVAL;
  const unsigned long RECONNECT_BASE_DELAY = WS_RECONNECT_DELAY;
  const int MAX_RECONNECT_ATTEMPTS = 10;
  
  // Subscription management
  bool vehicleSubscribed = false;
  unsigned long lastSubscribeAttempt = 0;
  
  void subscribeToVehicle() {
    if (millis() - lastSubscribeAttempt < 5000) {
      LOG_DEBUG(MODULE_WS, "Subscribe attempt dibatasi (cooldown 5s)");
      return;
    }
    
    LOG_INFO(MODULE_WS, "Mengirim subscription request untuk vehicle...");
    
    // Gunakan JSON document kecil
    StaticJsonDocument<256> doc;
    doc["type"] = "subscribe";
    doc["collection"] = "vehicle";
    
    JsonObject query = doc.createNestedObject("query");
    JsonArray fields = query.createNestedArray("fields");
    fields.add("*");
    
    String message;
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
    
    // Gunakan DynamicJsonDocument untuk response besar
    DynamicJsonDocument doc(4096);
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
      // Retry subscription after delay
      lastSubscribeAttempt = millis() - 4000; // Allow retry in 1 second
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
  }
  
  void setOnRelayUpdate(void (*callback)(bool)) {
    onRelayUpdateCallback = callback;
    LOG_DEBUG(MODULE_WS, "Relay update callback diset");
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
  
  void disconnect() {
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
      }
    }
  }
  
  void update() {
    if (state == WS_DISCONNECTED) {
      // Handle reconnection dengan exponential backoff
      unsigned long now = millis();
      unsigned long delay = RECONNECT_BASE_DELAY * (1 << min(reconnectAttempts, 5));
      delay = min(delay, 60000UL); // Max 1 menit
      
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
    
    // Kirim ping untuk keep-alive
    if (millis() - lastPingTime >= PING_INTERVAL) {
      wsClient->sendPing();
      lastPingTime = millis();
    }
    
    // Re-subscribe jika perlu
    if (state == WS_CONNECTED && !vehicleSubscribed && 
        millis() - lastSubscribeAttempt > 10000) {
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
  }
  
  bool sendVehicleData(float lat, float lon, float speed, int satellites, 
                       const String& timestamp) {
    if (!isReady()) {
      LOG_WARN(MODULE_WS, "⚠️ Tidak siap mengirim data (state: %s)", getStateString());
      return false;
    }
    
    // Buat JSON compact
    StaticJsonDocument<512> doc;
    
    doc["type"] = "items";
    doc["collection"] = "vehicle_datas";
    doc["action"] = "create";
    
    JsonObject data = doc.createNestedObject("data");
    data["gps_id"] = GPS_ID;
    data["latitude"] = serialized(String(lat, 6));
    data["longitude"] = serialized(String(lon, 6));
    data["speed"] = (int)speed;
    data["satellites_used"] = satellites;
    data["timestamp"] = timestamp;
    data["battery_level"] = 12.5; // TODO: Get dari battery monitor
    
    // Optional fields
    if (speed > 0) {
      data["is_moving"] = true;
    }
    
    String message;
    serializeJson(doc, message);
    
    LOG_DEBUG(MODULE_WS, "📤 Mengirim: %s", message.c_str());
    wsClient->sendText(message);
    
    stats.totalBytesSent += message.length();
    stats.totalMessages++;
    
    return true;
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
};

#endif // WEBSOCKET_MANAGER_H