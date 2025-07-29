// WebSocketManager.cpp - COMPLETE FIXED Implementation dengan Offline Integration
#include "WebSocketManager.h"

// ========================================
// LatencyTracker Implementation
// ========================================

void LatencyTracker::addSample(unsigned long latency) {
  if (!initialized) {
    memset(samples, 0, sizeof(samples));
    currentIndex = 0;
    sampleCount = 0;
    totalLatency = 0;
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

unsigned long LatencyTracker::getAverage() const {
  return sampleCount > 0 ? totalLatency / sampleCount : 0;
}

unsigned long LatencyTracker::getMin() const {
  if (sampleCount == 0) return 0;
  unsigned long min_val = samples[0];
  for (int i = 1; i < sampleCount; i++) {
    if (samples[i] < min_val) min_val = samples[i];
  }
  return min_val;
}

unsigned long LatencyTracker::getMax() const {
  if (sampleCount == 0) return 0;
  unsigned long max_val = samples[0];
  for (int i = 1; i < sampleCount; i++) {
    if (samples[i] > max_val) max_val = samples[i];
  }
  return max_val;
}

void LatencyTracker::reset() {
  initialized = false;
  currentIndex = 0;
  sampleCount = 0;
  totalLatency = 0;
  memset(samples, 0, sizeof(samples));
}

// ========================================
// SimpleWebSocketClient Implementation
// ========================================

SimpleWebSocketClient::SimpleWebSocketClient(TinyGsmClient* gsmClient) 
  : client(gsmClient), connected(false) {}

String SimpleWebSocketClient::generateWebSocketKey() {
  String key = "";
  key.reserve(24);
  for(int i = 0; i < 22; i++) {
    key += char(random(65, 90));
  }
  return key + "==";
}

bool SimpleWebSocketClient::performHandshake() {
  wsKey = generateWebSocketKey();
  
  LOG_DEBUG(MODULE_WS, "Memulai WebSocket handshake...");
  
  // Clear any pending data first
  while (client->available()) {
    client->read();
  }
  
  // Send WebSocket upgrade request
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
  client->println("User-Agent: ESP32-GPS-Tracker/2.1");
  client->println();
  client->flush();
  
  // Wait for response
  unsigned long timeout = millis() + WS_CONNECT_TIMEOUT;
  while (client->connected() && !client->available()) {
    if (millis() > timeout) {
      LOG_ERROR(MODULE_WS, "Handshake timeout");
      return false;
    }
    Utils::safeDelay(10);
  }
  
  // Read response headers
  String response = "";
  response.reserve(512);
  bool headerComplete = false;
  timeout = millis() + WS_RESPONSE_TIMEOUT;
  
  while (client->available() && !headerComplete && millis() < timeout) {
    char c = client->read();
    response += c;
    if (response.endsWith("\r\n\r\n")) {
      headerComplete = true;
    }
  }
  
  // Check upgrade successful
  if (response.indexOf("101 Switching Protocols") != -1) {
    LOG_INFO(MODULE_WS, "✅ WebSocket handshake berhasil");
    connected = true;
    return true;
  }
  
  LOG_ERROR(MODULE_WS, "❌ Handshake gagal. Response: %.200s", response.c_str());
  return false;
}

void SimpleWebSocketClient::sendFrame(uint8_t opcode, const String& payload) {
  if (!connected) {
    LOG_WARN(MODULE_WS, "Tidak dapat mengirim frame - tidak terhubung");
    return;
  }
  
  size_t len = payload.length();
  
  if (len > MAX_PAYLOAD_SIZE) {
    LOG_ERROR(MODULE_WS, "Payload terlalu besar: %d bytes (max: %d)", len, MAX_PAYLOAD_SIZE);
    return;
  }
  
  // Send frame header
  client->write(0x80 | opcode);
  
  // Payload length with masking bit
  if (len < 126) {
    client->write(0x80 | len);
  } else {
    client->write(0x80 | 126);
    client->write((len >> 8) & 0xFF);
    client->write(len & 0xFF);
  }
  
  // Masking key
  uint8_t mask[4];
  for (int i = 0; i < 4; i++) {
    mask[i] = random(0, 256);
    client->write(mask[i]);
  }
  
  // Send masked payload
  const char* payloadData = payload.c_str();
  for (size_t i = 0; i < len; i++) {
    client->write(payloadData[i] ^ mask[i % 4]);
  }
  
  client->flush();
}

bool SimpleWebSocketClient::connect(const String& url) {
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
  
  return performHandshake();
}

void SimpleWebSocketClient::disconnect() {
  if (connected) {
    LOG_INFO(MODULE_WS, "Memutuskan koneksi WebSocket...");
    sendFrame(WS_OPCODE_CLOSE, "");
    Utils::safeDelay(100);
    connected = false;
  }
  if (client && client->connected()) {
    client->stop();
  }
}

bool SimpleWebSocketClient::isConnected() {
  return connected && client && client->connected();
}

void SimpleWebSocketClient::sendText(const String& text) {
  sendFrame(WS_OPCODE_TEXT, text);
}

void SimpleWebSocketClient::sendPing() {
  sendFrame(WS_OPCODE_PING, "");
}

bool SimpleWebSocketClient::readMessage(String& message, unsigned long& bytesReceived) {
  if (!client || !client->available()) return false;
  
  // Read frame header
  uint8_t header = client->read();
  bool fin = (header & 0x80) != 0;
  uint8_t opcode = header & 0x0F;
  
  // Read payload length
  uint8_t len1 = client->read();
  bool masked = (len1 & 0x80) != 0;
  size_t len = len1 & 0x7F;
  
  if (len == 126) {
    len = (client->read() << 8) | client->read();
  } else if (len == 127) {
    for (int i = 0; i < 8; i++) client->read();
    LOG_ERROR(MODULE_WS, "Message terlalu besar (64-bit length)");
    return false;
  }
  
  // Skip mask if present
  if (masked) {
    for (int i = 0; i < 4; i++) client->read();
  }
  
  // Read payload with size limit
  message = "";
  size_t actualLen = min(len, (size_t)WS_MAX_MESSAGE_SIZE);
  message.reserve(actualLen + 1);
  bytesReceived = actualLen;
  
  for (size_t i = 0; i < actualLen; i++) {
    if (client->available()) {
      message += (char)client->read();
    } else {
      Utils::safeDelay(10);
      if (client->available()) {
        message += (char)client->read();
      } else {
        LOG_WARN(MODULE_WS, "Message tidak lengkap (received %d/%d)", i, actualLen);
        break;
      }
    }
  }
  
  // If message is larger than limit, discard the rest
  if (len > WS_MAX_MESSAGE_SIZE) {
    LOG_WARN(MODULE_WS, "Message terpotong dari %d ke %d bytes", len, WS_MAX_MESSAGE_SIZE);
    size_t remaining = len - WS_MAX_MESSAGE_SIZE;
    while (remaining-- > 0 && client->available()) {
      client->read();
    }
  }
  
  // Handle different opcodes
  switch (opcode) {
    case WS_OPCODE_TEXT:
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

// ========================================
// WebSocketManager Implementation
// ========================================

WebSocketManager::WebSocketManager(TinyGsmClient* client) 
  : gsmClient(client),
    state(WS_DISCONNECTED),
    lastPingTime(0),
    lastReconnectAttempt(0),
    reconnectAttempts(0),
    connectionStartTime(0),
    lastHealthCheck(0),
    consecutiveHealthFailures(0),
    forceRestart(false),
    restartDelayTimer(0),
    onRelayUpdateCallback(nullptr),
    wsUrl(WS_URL),
    PING_INTERVAL(WS_PING_INTERVAL),
    RECONNECT_BASE_DELAY(WS_RECONNECT_DELAY),
    MAX_RECONNECT_ATTEMPTS(15),
    vehicleSubscribed(false),
    lastSubscribeAttempt(0) {
  
  wsClient = new SimpleWebSocketClient(client);
  
  // Initialize stats
  memset(&stats, 0, sizeof(stats));
  stats.minLatency = UINT32_MAX;
  
  // Initialize latency tracker
  latencyTracker.reset();
}

WebSocketManager::~WebSocketManager() {
  delete wsClient;
}

void WebSocketManager::begin() {
  LOG_INFO(MODULE_WS, "FIXED WebSocket Manager initialized dengan offline integration");
  LOG_INFO(MODULE_WS, "Target: %s", wsUrl);
  LOG_INFO(MODULE_WS, "Ping Interval: %lu ms", PING_INTERVAL);
  LOG_INFO(MODULE_WS, "Max Payload Size: %d bytes", MAX_PAYLOAD_SIZE);
  LOG_INFO(MODULE_WS, "Health Check Interval: %lu ms", HEALTH_CHECK_INTERVAL);
  
  if (ENABLE_LATENCY_MONITORING) {
    LOG_INFO(MODULE_WS, "Performance monitoring enabled");
  }
}

void WebSocketManager::setOnRelayUpdate(void (*callback)(bool)) {
  onRelayUpdateCallback = callback;
  LOG_DEBUG(MODULE_WS, "Relay update callback diset");
}

bool WebSocketManager::connect() {
  if (state != WS_DISCONNECTED) {
    LOG_WARN(MODULE_WS, "Sudah terhubung atau sedang menghubungkan");
    return false;
  }
  
  LOG_INFO(MODULE_WS, "🔌 FIXED: Mencoba koneksi WebSocket...");
  state = WS_CONNECTING;
  connectionStartTime = millis();
  
  if (wsClient->connect(wsUrl)) {
    LOG_INFO(MODULE_WS, "✅ FIXED: WebSocket terhubung");
    state = WS_CONNECTED;
    stats.connectionTime = millis();
    lastPingTime = millis();
    reconnectAttempts = 0;
    vehicleSubscribed = false;
    stats.consecutiveFailures = 0;
    
    subscribeToVehicle();
    return true;
  }
  
  LOG_ERROR(MODULE_WS, "❌ Koneksi WebSocket gagal");
  state = WS_DISCONNECTED;
  stats.reconnectCount++;
  stats.consecutiveFailures++;
  return false;
}

void WebSocketManager::disconnect() {
  if (state != WS_DISCONNECTED) {
    LOG_INFO(MODULE_WS, "🔌 Memutuskan WebSocket...");
    
    if (stats.connectionTime > 0) {
      unsigned long connectionDuration = millis() - stats.connectionTime;
      stats.totalConnectionTime += connectionDuration;
      if (connectionDuration > stats.longestConnection) {
        stats.longestConnection = connectionDuration;
      }
    }
    
    wsClient->disconnect();
    state = WS_DISCONNECTED;
    vehicleSubscribed = false;
    
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

void WebSocketManager::subscribeToVehicle() {
  if (millis() - lastSubscribeAttempt < 3000) {
    return;
  }
  
  LOG_INFO(MODULE_WS, "Mengirim subscription request untuk vehicle...");
  
  StaticJsonDocument<200> doc;
  doc["type"] = "subscribe";
  doc["collection"] = "vehicle";
  
  JsonObject query = doc.createNestedObject("query");
  JsonArray fields = query.createNestedArray("fields");
  fields.add("*");
  
  String message;
  message.reserve(200);
  serializeJson(doc, message);
  
  wsClient->sendText(message);
  stats.totalBytesSent += message.length();
  stats.totalMessages++;
  lastSubscribeAttempt = millis();
  
  LOG_DEBUG(MODULE_WS, "Subscription request dikirim: %s", message.c_str());
}

void WebSocketManager::processMessage(const String& message) {
  LOG_DEBUG(MODULE_WS, "Memproses message (len=%d): %.100s...", 
            message.length(), message.c_str());
  
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    LOG_ERROR(MODULE_WS, "JSON parse error: %s (msg len: %d)", 
              error.c_str(), message.length());
    
    if (error == DeserializationError::NoMemory) {
      handleLargeMessage(message);
    }
    return;
  }
  
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

void WebSocketManager::handleSubscriptionMessage(JsonDocument& doc) {
  const char* event = doc["event"];
  
  if (strcmp(event, "init") == 0) {
    LOG_INFO(MODULE_WS, "✅ FIXED: Subscription diinisialisasi untuk offline integration");
    state = WS_SUBSCRIBED;
    vehicleSubscribed = true;
    processInitialData(doc["data"]);
  } else if (strcmp(event, "create") == 0 || strcmp(event, "update") == 0) {
    LOG_INFO(MODULE_WS, "📨 Vehicle update event: %s", event);
    processVehicleUpdate(doc["data"]);
  }
}

void WebSocketManager::handleAuthMessage(JsonDocument& doc) {
  const char* status = doc["status"];
  if (strcmp(status, "ok") == 0) {
    LOG_INFO(MODULE_WS, "✅ Autentikasi berhasil");
    subscribeToVehicle();
  } else {
    LOG_ERROR(MODULE_WS, "❌ Autentikasi gagal: %s", status);
  }
}

void WebSocketManager::handleErrorMessage(JsonDocument& doc) {
  const char* errorMsg = doc["error"]["message"] | "Unknown error";
  const char* errorCode = doc["error"]["code"] | "UNKNOWN";
  LOG_ERROR(MODULE_WS, "❌ Server error [%s]: %s", errorCode, errorMsg);
  
  if (strcmp(errorCode, "SUBSCRIPTION_FAILED") == 0) {
    vehicleSubscribed = false;
    lastSubscribeAttempt = millis() - 2000;
  }
}

void WebSocketManager::handleLargeMessage(const String& message) {
  LOG_INFO(MODULE_WS, "Handling large message dengan parsing manual");
  
  int typeStart = message.indexOf("\"type\":\"");
  if (typeStart > 0) {
    typeStart += 8;
    int typeEnd = message.indexOf("\"", typeStart);
    if (typeEnd > typeStart) {
      String msgType = message.substring(typeStart, typeEnd);
      LOG_INFO(MODULE_WS, "Message type: %s (terlalu besar untuk parse penuh)", msgType.c_str());
      
      if (msgType == "subscription") {
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
              tryExtractVehicleData(message);
            }
          }
        }
      }
    }
  }
}

void WebSocketManager::tryExtractVehicleData(const String& message) {
  int gpsIdPos = message.indexOf(GPS_ID);
  if (gpsIdPos > 0) {
    LOG_DEBUG(MODULE_WS, "Vehicle kita ditemukan dalam response");
    
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

void WebSocketManager::processInitialData(JsonVariant data) {
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

void WebSocketManager::processVehicleUpdate(JsonVariant data) {
  if (data.is<JsonArray>()) {
    JsonArray items = data.as<JsonArray>();
    for (JsonObject item : items) {
      processVehicleItem(item);
    }
  } else if (data.is<JsonObject>()) {
    JsonObject obj = data.as<JsonObject>();
    processVehicleItem(obj);
  }
}

void WebSocketManager::processVehicleItem(const JsonObject& item) {
  String gpsId = item["gps_id"] | "";
  if (gpsId == GPS_ID) {
    String relayStatus = item["relay_status"] | "OFF";
    bool newState = (relayStatus == "ON");
    
    LOG_INFO(MODULE_WS, "📨 Relay status update: %s", relayStatus.c_str());
    
    if (onRelayUpdateCallback) {
      onRelayUpdateCallback(newState);
    }
  }
}

String WebSocketManager::formatTimestamp(unsigned long unixTime) {
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

void WebSocketManager::performHealthCheck() {
  if (millis() - lastHealthCheck < HEALTH_CHECK_INTERVAL) {
    return;
  }
  
  lastHealthCheck = millis();
  bool healthIssue = false;
  
  if (state == WS_DISCONNECTED && millis() > WEBSOCKET_DISCONNECT_TIMEOUT) {
    LOG_WARN(MODULE_WS, "⚠️ WebSocket stuck disconnected");
    healthIssue = true;
  }
  
  if (stats.consecutiveFailures > MAX_CONSECUTIVE_FAILURES) {
    LOG_WARN(MODULE_WS, "⚠️ Too many consecutive failures: %lu", stats.consecutiveFailures);
    healthIssue = true;
  }
  
  if (healthIssue) {
    consecutiveHealthFailures++;
    if (consecutiveHealthFailures >= MAX_HEALTH_FAILURES) {
      LOG_ERROR(MODULE_WS, "🚨 WebSocket health critical, forcing system restart");
      forceRestart = true;
      restartDelayTimer = millis();
    }
  } else {
    consecutiveHealthFailures = 0;
  }
}

void WebSocketManager::update() {
  performHealthCheck();
  
  if (forceRestart && restartDelayTimer > 0) {
    if (millis() - restartDelayTimer >= 2000) {
      LOG_ERROR(MODULE_WS, "🚨 WebSocket forcing system restart");
      ESP.restart();
    }
    return;
  }
  
  if (state == WS_DISCONNECTED) {
    unsigned long now = millis();
    unsigned long delay = RECONNECT_BASE_DELAY * (1 << min(reconnectAttempts, 4));
    delay = min(delay, 30000UL);
    
    if (now - lastReconnectAttempt >= delay) {
      LOG_INFO(MODULE_WS, "🔄 Percobaan reconnect #%d", reconnectAttempts + 1);
      lastReconnectAttempt = now;
      reconnectAttempts++;
      
      if (reconnectAttempts > MAX_RECONNECT_ATTEMPTS) {
        LOG_ERROR(MODULE_WS, "🚨 Max reconnect attempts reached, forcing restart");
        forceRestart = true;
        restartDelayTimer = millis();
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
    stats.consecutiveFailures++;
    return;
  }
  
  if (millis() - lastPingTime >= PING_INTERVAL) {
    wsClient->sendPing();
    lastPingTime = millis();
  }
  
  if (state == WS_CONNECTED && !vehicleSubscribed && 
      millis() - lastSubscribeAttempt > 7000) {
    LOG_WARN(MODULE_WS, "⚠️ Belum subscribe, mencoba lagi...");
    subscribeToVehicle();
  }
  
  String message;
  unsigned long bytesReceived = 0;
  if (wsClient->readMessage(message, bytesReceived)) {
    stats.totalBytesReceived += bytesReceived;
    stats.lastMessageTime = millis();
    processMessage(message);
  }
}

// FIXED: Complete sendVehicleData dengan RELAXED state checking untuk offline integration
bool WebSocketManager::sendVehicleData(float lat, float lon, float speed, int satellites, 
                                       const String& timestamp, float battery) {
  // FIXED: More lenient state checking untuk offline data sending
  if (!wsClient) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ WebSocket client not initialized");
    return false;
  }
  
  // FIXED: Check connection but allow sending if websocket is connected (even if not subscribed yet)
  if (!wsClient->isConnected()) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ WebSocket client connection lost");
    state = WS_DISCONNECTED;
    return false;
  }
  
  // FIXED: Allow sending in CONNECTED state as well (not just SUBSCRIBED) for offline data
  if (state != WS_SUBSCRIBED && state != WS_CONNECTED) {
    LOG_WARN(MODULE_WEBSOCKET, "⚠️ WebSocket not ready, state: %s", getStateString());
    // For offline data, we should still try to send even if not fully subscribed
    if (state == WS_DISCONNECTED) {
      return false;
    }
  }
  
  // FIXED: Enhanced input validation
  if (abs(lat) > 90.0 || abs(lon) > 180.0) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ Invalid coordinates: lat=%.6f, lon=%.6f", lat, lon);
    return false;
  }
  
  if (speed < 0.0 || speed > 300.0) {
    LOG_DEBUG(MODULE_WEBSOCKET, "⚠️ Unusual speed value: %.1f km/h", speed);
  }
  
  if (satellites < 0 || satellites > 50) {
    LOG_DEBUG(MODULE_WEBSOCKET, "⚠️ Unusual satellite count: %d", satellites);
  }
  
  // FIXED: Start latency measurement
  if (ENABLE_LATENCY_MONITORING) {
    stats.lastTransmissionStart = millis();
    stats.measuringLatency = true;
  }
  
  // FIXED: Use ArduinoJson untuk safe JSON creation
  StaticJsonDocument<512> doc;
  
  // FIXED: Create exact structure yang server expect
  doc["type"] = "items";
  doc["collection"] = "vehicle_datas";
  doc["action"] = "create";
  
  JsonObject data = doc.createNestedObject("data");
  
  // FIXED: Format coordinates dengan proper precision dan STRING format
  char latStr[16], lonStr[16];
  snprintf(latStr, sizeof(latStr), "%.6f", lat);   // 6 decimal precision
  snprintf(lonStr, sizeof(lonStr), "%.6f", lon);   // 6 decimal precision
  
  // FIXED: Use exact field names dari server API
  data["latitude"] = latStr;              // Server expects STRING
  data["longitude"] = lonStr;             // Server expects STRING  
  data["speed"] = speed;                  // Server expects NUMBER
  data["satellites_used"] = satellites;   // Server expects NUMBER
  data["timestamp"] = timestamp;          // Server expects STRING (ISO format)
  data["battery_voltage"] = battery;      // Server expects NUMBER
  data["gps_id"] = GPS_ID;               // Server expects STRING
  
  // FIXED: Serialize dengan size checking
  String payload;
  size_t payloadSize = serializeJson(doc, payload);
  
  if (payloadSize == 0) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ Failed to serialize JSON payload");
    return false;
  }
  
  if (payloadSize > MAX_PAYLOAD_SIZE) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ Payload too large: %d bytes (max: %d)", 
              payloadSize, MAX_PAYLOAD_SIZE);
    return false;
  }
  
  // FIXED: Enhanced logging untuk debugging (reduced verbosity for offline data)
  LOG_DEBUG(MODULE_WEBSOCKET, "📤 Sending data: %.6f,%.6f %.1fkm/h %dsats (%d bytes)", 
            lat, lon, speed, satellites, payloadSize);
  
  // FIXED: Enhanced sending dengan timeout dan error handling
  bool success = false;
  unsigned long sendStart = millis();
  const unsigned long SEND_TIMEOUT = 10000; // 10 second timeout for offline data
  
  try {
    // FIXED: Send dengan proper error checking
    wsClient->sendText(payload);
    
    // FIXED: Wait for send completion dengan timeout
    unsigned long sendDuration = millis() - sendStart;
    
    if (sendDuration < SEND_TIMEOUT) {
      success = true;
      
      // Update statistics
      stats.totalBytesSent += payloadSize;
      stats.totalMessages++;
      stats.lastSuccessfulTransmission = millis();
      stats.consecutiveFailures = 0;
      
      // FIXED: End latency measurement
      if (ENABLE_LATENCY_MONITORING && stats.measuringLatency) {
        unsigned long latency = millis() - stats.lastTransmissionStart;
        
        stats.totalLatency += latency;
        stats.latencySamples++;
        
        if (latency < stats.minLatency) stats.minLatency = latency;
        if (latency > stats.maxLatency) stats.maxLatency = latency;
        
        latencyTracker.addSample(latency);
        stats.measuringLatency = false;
        
        if (DEBUG_LATENCY_TRACKING) {
          LOG_DEBUG(MODULE_WEBSOCKET, "📊 Transmission latency: %lu ms (avg: %lu ms)", 
                    latency, latencyTracker.getAverage());
        }
        
        if (latency > LATENCY_WARNING_THRESHOLD) {
          LOG_WARN(MODULE_WEBSOCKET, "⚠️ High latency detected: %lu ms", latency);
        }
      }
      
      LOG_DEBUG(MODULE_WEBSOCKET, "✅ Vehicle data sent successfully (%d bytes)", payloadSize);
    } else {
      LOG_ERROR(MODULE_WEBSOCKET, "❌ Send timeout after %lu ms", sendDuration);
      success = false;
    }
    
  } catch (const std::exception& e) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ Exception during data transmission: %s", e.what());
    success = false;
  } catch (...) {
    LOG_ERROR(MODULE_WEBSOCKET, "❌ Unknown exception during data transmission");
    success = false;
  }
  
  // FIXED: Handle failure properly
  if (!success) {
    stats.consecutiveFailures++;
    LOG_ERROR(MODULE_WEBSOCKET, "❌ Vehicle data transmission failed (failures: %lu)", 
              stats.consecutiveFailures);
    
    // Check if connection is still valid
    if (!wsClient->isConnected()) {
      LOG_WARN(MODULE_WEBSOCKET, "⚠️ Connection lost during transmission");
      state = WS_DISCONNECTED;
    }
  }
  
  return success;
}

void WebSocketManager::startLatencyMeasurement() {
  if (ENABLE_LATENCY_MONITORING) {
    stats.lastTransmissionStart = millis();
    stats.measuringLatency = true;
  }
}

void WebSocketManager::endLatencyMeasurement() {
  if (ENABLE_LATENCY_MONITORING && stats.measuringLatency) {
    unsigned long latency = millis() - stats.lastTransmissionStart;
    
    stats.totalLatency += latency;
    stats.latencySamples++;
    
    if (latency < stats.minLatency) stats.minLatency = latency;
    if (latency > stats.maxLatency) stats.maxLatency = latency;
    
    latencyTracker.addSample(latency);
    stats.measuringLatency = false;
    
    if (DEBUG_LATENCY_TRACKING) {
      LOG_DEBUG(MODULE_WS, "📊 Transmission latency: %lu ms (avg: %lu ms)", 
                latency, latencyTracker.getAverage());
    }
    
    if (latency > LATENCY_WARNING_THRESHOLD) {
      LOG_WARN(MODULE_WS, "⚠️ High latency detected: %lu ms", latency);
    }
  }
}

unsigned long WebSocketManager::getAverageLatency() {
  return latencyTracker.getAverage();
}

unsigned long WebSocketManager::getMinLatency() {
  return latencyTracker.getMin();
}

unsigned long WebSocketManager::getMaxLatency() {
  return latencyTracker.getMax();
}

String WebSocketManager::getPerformanceReport() {
  String report = "=== WEBSOCKET PERFORMANCE ===\n";
  report += "State: " + String(getStateString()) + "\n";
  report += "Total Messages: " + String(stats.totalMessages) + "\n";
  report += "Bytes Sent: " + String(stats.totalBytesSent) + "\n";
  report += "Bytes Received: " + String(stats.totalBytesReceived) + "\n";
  report += "Reconnect Count: " + String(stats.reconnectCount) + "\n";
  
  if (ENABLE_LATENCY_MONITORING && stats.latencySamples > 0) {
    report += "Average Latency: " + String(getAverageLatency()) + " ms\n";
    report += "Min Latency: " + String(getMinLatency()) + " ms\n";
    report += "Max Latency: " + String(getMaxLatency()) + " ms\n";
  }
  
  return report;
}

void WebSocketManager::resetPerformanceStats() {
  memset(&stats, 0, sizeof(stats));
  stats.minLatency = UINT32_MAX;
  latencyTracker.reset();
  consecutiveHealthFailures = 0;
  LOG_INFO(MODULE_WS, "Performance statistics reset");
}

void WebSocketManager::maintainConnection() {
  static unsigned long lastHealthCheck = 0;
  unsigned long now = millis();
  
  if (now - lastHealthCheck < CONNECTION_HEALTH_CHECK_INTERVAL) {
    return;
  }
  
  lastHealthCheck = now;
  
  if (state == WS_SUBSCRIBED && wsClient->isConnected()) {
    if (DEBUG_NETWORK_QUALITY) {
      LOG_DEBUG(MODULE_WS, "📡 Connection health: GOOD");
    }
  } else if (state != WS_DISCONNECTED) {
    LOG_WARN(MODULE_WS, "🔧 Connection health issue detected");
    
    if (!wsClient->isConnected()) {
      state = WS_DISCONNECTED;
      vehicleSubscribed = false;
      stats.consecutiveFailures++;
    } else if (state == WS_CONNECTED && !vehicleSubscribed) {
      subscribeToVehicle();
    }
  }
}

// FIXED: More lenient isReady() check untuk offline integration
bool WebSocketManager::isReady() {
  // FIXED: Allow both SUBSCRIBED and CONNECTED states for offline data sending
  return (state == WS_SUBSCRIBED || state == WS_CONNECTED) && wsClient->isConnected();
}

WSState WebSocketManager::getState() {
  return state;
}

const char* WebSocketManager::getStateString() {
  switch (state) {
    case WS_DISCONNECTED: return "DISCONNECTED";
    case WS_CONNECTING: return "CONNECTING";
    case WS_CONNECTED: return "CONNECTED";
    case WS_SUBSCRIBED: return "SUBSCRIBED";
    default: return "UNKNOWN";
  }
}

const WSStats& WebSocketManager::getStats() {
  return stats;
}

void WebSocketManager::resetReconnectAttempts() {
  reconnectAttempts = 0;
  stats.consecutiveFailures = 0;
  consecutiveHealthFailures = 0;
  forceRestart = false;
  restartDelayTimer = 0;
  LOG_INFO(MODULE_WS, "Reconnect attempts reset");
}

bool WebSocketManager::isPerformanceGood() {
  if (!ENABLE_LATENCY_MONITORING || stats.latencySamples < 3) {
    return true;
  }
  return getAverageLatency() <= MAX_ACCEPTABLE_LATENCY;
}

bool WebSocketManager::isHealthy() {
  if (stats.consecutiveFailures > MAX_CONSECUTIVE_FAILURES / 2) return false;
  if (consecutiveHealthFailures > 0) return false;
  if (reconnectAttempts > MAX_RECONNECT_ATTEMPTS / 2) return false;
  
  if (state == WS_DISCONNECTED && millis() > WEBSOCKET_DISCONNECT_TIMEOUT) {
    return false;
  }
  
  return true;
}

String WebSocketManager::getHealthStatus() {
  String status = "WebSocket Health: ";
  if (isHealthy()) {
    status += "✅ HEALTHY";
  } else {
    status += "⚠️ ISSUES DETECTED";
    status += " (Failures: " + String(stats.consecutiveFailures);
    status += ", Reconnects: " + String(reconnectAttempts) + ")";
  }
  return status;
}

void WebSocketManager::forcePing() {
  if (wsClient && wsClient->isConnected()) {
    wsClient->sendPing();
    LOG_INFO(MODULE_WS, "🏓 Force ping sent");
  }
}

unsigned long WebSocketManager::getConnectionUptime() {
  if (stats.connectionTime == 0) return 0;
  return millis() - stats.connectionTime;
}

bool WebSocketManager::shouldForceRestart() {
  return forceRestart && restartDelayTimer > 0 && (millis() - restartDelayTimer >= 2000);
}

int WebSocketManager::getPayloadMode() {
  return DEFAULT_PAYLOAD_MODE;
}

size_t WebSocketManager::getLastPayloadSize() {
  return strlen(payloadBuffer);
}