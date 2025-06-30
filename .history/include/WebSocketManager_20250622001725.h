// WebSocketManager.h - Enhanced with Relay Debugging
#ifndef WEBSOCKET_MANAGER_H
#define WEBSOCKET_MANAGER_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "Logger.h"

// WebSocket states
enum WSState {
  WS_DISCONNECTED,
  WS_CONNECTING,
  WS_CONNECTED,
  WS_SUBSCRIBED
};

// Simple WebSocket client implementation
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
    // Generate random 16-byte key and encode to base64
    String key = "";
    for(int i = 0; i < 22; i++) {
      key += char(random(65, 90)); // Simple random key
    }
    return key + "==";
  }
  
  bool performHandshake() {
    wsKey = generateWebSocketKey();
    
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
    client->println("Sec-WebSocket-Protocol: graphql-ws");
    client->println();
    
    // Wait for response
    unsigned long timeout = millis() + 5000;
    while (client->connected() && !client->available()) {
      if (millis() > timeout) {
        LOG_ERROR(MODULE_WS, "Handshake timeout");
        return false;
      }
      delay(10);
    }
    
    // Read response headers only
    String response = "";
    while (client->available()) {
      char c = client->read();
      response += c;
      if (response.endsWith("\r\n\r\n")) break;
    }
    
    // Check for successful upgrade
    if (response.indexOf("101 Switching Protocols") != -1) {
      LOG_INFO(MODULE_WS, "WebSocket handshake successful");
      connected = true;
      return true;
    }
    
    LOG_ERROR(MODULE_WS, "Handshake failed: %s", response.c_str());
    return false;
  }
  
  void sendFrame(uint8_t opcode, const String& payload) {
    if (!connected) return;
    
    size_t len = payload.length();
    
    // Send frame header
    client->write(0x80 | opcode); // FIN = 1
    
    // Payload length with masking bit
    if (len < 126) {
      client->write(0x80 | len); // Mask = 1
    } else if (len < 65536) {
      client->write(0x80 | 126);
      client->write((len >> 8) & 0xFF);
      client->write(len & 0xFF);
    } else {
      // Payload too large for WebSocket
      LOG_ERROR(MODULE_WS, "Payload too large: %d bytes", len);
      return;
    }
    
    // Masking key (required for client)
    uint8_t mask[4];
    for (int i = 0; i < 4; i++) {
      mask[i] = random(0, 256);
      client->write(mask[i]);
    }
    
    // Send masked payload
    for (size_t i = 0; i < len; i++) {
      client->write(payload[i] ^ mask[i % 4]);
    }
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
      LOG_ERROR(MODULE_WS, "WSS not supported yet");
      return false;
    } else {
      LOG_ERROR(MODULE_WS, "Invalid WebSocket URL");
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
    
    LOG_INFO(MODULE_WS, "Connecting to %s:%d%s", host.c_str(), port, path.c_str());
    
    // Connect TCP
    if (!client->connect(host.c_str(), port)) {
      LOG_ERROR(MODULE_WS, "TCP connection failed");
      return false;
    }
    
    // Perform WebSocket handshake
    return performHandshake();
  }
  
  void disconnect() {
    if (connected) {
      // Send close frame
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
  }
  
  bool readMessage(String& message) {
    if (!client->available()) return false;
    
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
      // Skip 8 bytes for 64-bit length (we don't support such large messages)
      for (int i = 0; i < 8; i++) client->read();
      LOG_ERROR(MODULE_WS, "Message too large");
      return false;
    }
    
    // Skip mask if present (server->client shouldn't be masked)
    if (masked) {
      for (int i = 0; i < 4; i++) client->read();
    }
    
    // Read payload with size limit
    message = "";
    message.reserve(len + 1);
    
    for (size_t i = 0; i < len && i < 2048; i++) { // Limit to 2KB
      if (client->available()) {
        message += (char)client->read();
      } else {
        // Wait a bit for more data
        delay(10);
        if (client->available()) {
          message += (char)client->read();
        } else {
          LOG_WARN(MODULE_WS, "Incomplete message received");
          break;
        }
      }
    }
    
    // If message was larger than our limit, drain the rest
    if (len > 2048) {
      LOG_WARN(MODULE_WS, "Message truncated from %d to 2048 bytes", len);
      while (len-- > 2048 && client->available()) {
        client->read();
      }
    }
    
    // Handle different opcodes
    switch (opcode) {
      case WS_OPCODE_TEXT:
        return true;
      case WS_OPCODE_PING:
        sendFrame(WS_OPCODE_PONG, message);
        return false;
      case WS_OPCODE_CLOSE:
        LOG_INFO(MODULE_WS, "Server sent close frame");
        connected = false;
        return false;
      default:
        LOG_WARN(MODULE_WS, "Unknown opcode: 0x%X", opcode);
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
  
  // Callbacks
  void (*onRelayUpdateCallback)(bool newState) = nullptr;
  
  // Configuration
  const char* wsUrl = WS_URL;
  const unsigned long PING_INTERVAL = 30000;      // 30 seconds
  const unsigned long RECONNECT_BASE_DELAY = 5000;
  
  // Track last relay state
  bool lastKnownRelayState = true;
  
  void subscribeToVehicle() {
    LOG_INFO(MODULE_WS, "📡 Subscribing to vehicle collection...");
    
    // Build the full filter with our GPS ID
    String filter = "{\"gps_id\":{\"_eq\":\"" + String(GPS_ID) + "\"}}";
    
    // Use smaller JSON document with complete subscription
    StaticJsonDocument<384> doc;
    doc["type"] = "subscribe";
    doc["collection"] = "vehicle";
    
    JsonObject query = doc.createNestedObject("query");
    query["filter"] = serialized(filter);
    
    JsonArray fields = query.createNestedArray("fields");
    fields.add("id");
    fields.add("gps_id");
    fields.add("relay_status");
    fields.add("date_updated");
    
    String message;
    serializeJson(doc, message);
    
    LOG_DEBUG(MODULE_WS, "Subscription message: %s", message.c_str());
    wsClient->sendText(message);
    
    LOG_INFO(MODULE_WS, "✅ Subscription request sent for GPS_ID: %s", GPS_ID);
  }
  
  void processMessage(const String& message) {
    // Log message for debugging relay issues
    LOG_DEBUG(MODULE_WS, "📨 Received message (len=%d): %.200s...", 
              message.length(), message.c_str());
    
    // Try to parse JSON
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      // If parse error, try to extract relay status manually
      if (error == DeserializationError::NoMemory) {
        LOG_WARN(MODULE_WS, "JSON too large, trying manual extraction...");
        
        // Look for our GPS ID first
        int gpsIdPos = message.indexOf(GPS_ID);
        if (gpsIdPos > 0) {
          LOG_DEBUG(MODULE_WS, "Found our GPS_ID at position %d", gpsIdPos);
          
          // Look for relay_status near our GPS ID
          int searchStart = max(0, gpsIdPos - 200);
          int searchEnd = min((int)message.length(), gpsIdPos + 200);
          String searchArea = message.substring(searchStart, searchEnd);
          
          int relayPos = searchArea.indexOf("\"relay_status\":");
          if (relayPos >= 0) {
            relayPos += 15; // Skip "relay_status":
            int relayEnd = searchArea.indexOf(",", relayPos);
            if (relayEnd < 0) relayEnd = searchArea.indexOf("}", relayPos);
            
            String relayValue = searchArea.substring(relayPos, relayEnd);
            relayValue.trim();
            relayValue.replace("\"", "");
            relayValue.replace(" ", "");
            
            LOG_INFO(MODULE_WS, "🔍 Extracted relay status: '%s'", relayValue.c_str());
            
            bool newState = (relayValue == "ON" || relayValue == "on" || relayValue == "true" || relayValue == "1");
            
            if (newState != lastKnownRelayState) {
              LOG_INFO(MODULE_WS, "🔄 Relay status changed: %s → %s", 
                       lastKnownRelayState ? "ON" : "OFF", 
                       newState ? "ON" : "OFF");
              lastKnownRelayState = newState;
              
              if (onRelayUpdateCallback) {
                onRelayUpdateCallback(newState);
              }
            }
          } else {
            LOG_DEBUG(MODULE_WS, "relay_status not found near GPS_ID");
          }
        } else {
          LOG_DEBUG(MODULE_WS, "Our GPS_ID not found in message");
        }
        return;
      }
      
      LOG_ERROR(MODULE_WS, "JSON parse error: %s", error.c_str());
      return;
    }
    
    // Normal JSON processing
    const char* type = doc["type"];
    
    if (strcmp(type, "subscription") == 0) {
      const char* event = doc["event"];
      
      LOG_INFO(MODULE_WS, "📡 Subscription event: %s", event);
      
      if (strcmp(event, "init") == 0) {
        LOG_INFO(MODULE_WS, "✅ Subscription initialized");
        state = WS_SUBSCRIBED;
        processInitialData(doc["data"]);
      } else if (strcmp(event, "create") == 0 || 
                 strcmp(event, "update") == 0) {
        LOG_INFO(MODULE_WS, "🔄 Vehicle %s event received", event);
        processVehicleUpdate(doc["data"]);
      }
    } else if (strcmp(type, "auth") == 0) {
      const char* status = doc["status"];
      if (strcmp(status, "ok") == 0) {
        LOG_INFO(MODULE_WS, "🔐 Authentication successful");
        subscribeToVehicle();
      } else {
        LOG_ERROR(MODULE_WS, "Authentication failed: %s", status);
      }
    } else if (strcmp(type, "error") == 0) {
      const char* errorMsg = doc["error"]["message"] | "Unknown error";
      LOG_ERROR(MODULE_WS, "❌ Server error: %s", errorMsg);
    } else if (strcmp(type, "ping") == 0) {
      LOG_DEBUG(MODULE_WS, "🏓 Received ping from server");
      // Send pong
      StaticJsonDocument<64> pongDoc;
      pongDoc["type"] = "pong";
      String pongMsg;
      serializeJson(pongDoc, pongMsg);
      wsClient->sendText(pongMsg);
    }
  }
  
  void processInitialData(JsonVariant data) {
    LOG_INFO(MODULE_WS, "🔍 Processing initial data...");
    
    if (!data.is<JsonArray>()) {
      // Could be a single object
      if (data.is<JsonObject>()) {
        JsonObject vehicle = data.as<JsonObject>();
        processVehicleObject(vehicle);
      } else {
        LOG_WARN(MODULE_WS, "Initial data is not an array or object");
      }
      return;
    }
    
    JsonArray vehicles = data.as<JsonArray>();
    LOG_INFO(MODULE_WS, "📋 Processing %d vehicles", vehicles.size());
    
    bool foundOurVehicle = false;
    for (JsonObject vehicle : vehicles) {
      String gpsId = vehicle["gps_id"] | "";
      LOG_DEBUG(MODULE_WS, "Checking vehicle with GPS_ID: %s", gpsId.c_str());
      
      if (gpsId == GPS_ID) {
        foundOurVehicle = true;
        processVehicleObject(vehicle);
        break;
      }
    }
    
    if (!foundOurVehicle) {
      LOG_WARN(MODULE_WS, "⚠️ Our vehicle (GPS_ID: %s) not found in initial data", GPS_ID);
    }
  }
  
  void processVehicleUpdate(JsonVariant data) {
    LOG_INFO(MODULE_WS, "🔄 Processing vehicle update...");
    
    if (data.is<JsonArray>()) {
      JsonArray items = data.as<JsonArray>();
      LOG_DEBUG(MODULE_WS, "Update contains %d items", items.size());
      
      for (JsonObject item : items) {
        String gpsId = item["gps_id"] | "";
        if (gpsId == GPS_ID) {
          processVehicleObject(item);
          break;
        }
      }
    } else if (data.is<JsonObject>()) {
      // Single item update
      JsonObject item = data.as<JsonObject>();
      String gpsId = item["gps_id"] | "";
      
      LOG_DEBUG(MODULE_WS, "Single item update for GPS_ID: %s", gpsId.c_str());
      
      if (gpsId == GPS_ID) {
        processVehicleObject(item);
      }
    }
  }
  
  void processVehicleObject(JsonObject vehicle) {
    String gpsId = vehicle["gps_id"] | "";
    String relayStatus = vehicle["relay_status"] | "OFF";
    String dateUpdated = vehicle["date_updated"] | "unknown";
    
    LOG_INFO(MODULE_WS, "📍 Found our vehicle:");
    LOG_INFO(MODULE_WS, "   GPS_ID: %s", gpsId.c_str());
    LOG_INFO(MODULE_WS, "   Relay Status: %s", relayStatus.c_str());
    LOG_INFO(MODULE_WS, "   Last Updated: %s", dateUpdated.c_str());
    
    // Convert relay status to boolean
    bool newState = (relayStatus == "ON" || relayStatus == "on" || 
                     relayStatus == "true" || relayStatus == "1");
    
    // Check if relay state changed
    if (newState != lastKnownRelayState) {
      LOG_INFO(MODULE_WS, "🎯 RELAY STATE CHANGED: %s → %s", 
               lastKnownRelayState ? "ON" : "OFF", 
               newState ? "ON" : "OFF");
      
      lastKnownRelayState = newState;
      
      if (onRelayUpdateCallback) {
        LOG_INFO(MODULE_WS, "🔔 Calling relay update callback...");
        onRelayUpdateCallback(newState);
      } else {
        LOG_ERROR(MODULE_WS, "❌ No relay update callback set!");
      }
    } else {
      LOG_DEBUG(MODULE_WS, "Relay state unchanged: %s", newState ? "ON" : "OFF");
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
    LOG_INFO(MODULE_WS, "WebSocket Manager initialized");
    LOG_INFO(MODULE_WS, "GPS_ID: %s", GPS_ID);
  }
  
  void setOnRelayUpdate(void (*callback)(bool)) {
    onRelayUpdateCallback = callback;
    LOG_INFO(MODULE_WS, "✅ Relay update callback registered");
  }
  
  bool connect() {
    if (state != WS_DISCONNECTED) {
      LOG_WARN(MODULE_WS, "Already connected or connecting");
      return false;
    }
    
    LOG_INFO(MODULE_WS, "🔌 Attempting WebSocket connection...");
    state = WS_CONNECTING;
    
    if (wsClient->connect(wsUrl)) {
      LOG_INFO(MODULE_WS, "✅ WebSocket connected successfully");
      state = WS_CONNECTED;
      lastPingTime = millis();
      reconnectAttempts = 0;
      
      // Send connection init
      StaticJsonDocument<64> initDoc;
      initDoc["type"] = "connection_init";
      String initMsg;
      serializeJson(initDoc, initMsg);
      wsClient->sendText(initMsg);
      
      LOG_INFO(MODULE_WS, "📤 Sent connection_init");
      
      // Subscribe immediately after a short delay
      delay(500);
      subscribeToVehicle();
      
      return true;
    }
    
    LOG_ERROR(MODULE_WS, "❌ WebSocket connection failed");
    state = WS_DISCONNECTED;
    return false;
  }
  
  void disconnect() {
    if (state != WS_DISCONNECTED) {
      wsClient->disconnect();
      state = WS_DISCONNECTED;
      LOG_INFO(MODULE_WS, "🔌 WebSocket disconnected");
    }
  }
  
  void update() {
    if (state == WS_DISCONNECTED) {
      // Handle reconnection with exponential backoff
      unsigned long now = millis();
      unsigned long delay = RECONNECT_BASE_DELAY * (1 << min(reconnectAttempts, 5));
      
      if (now - lastReconnectAttempt >= delay) {
        LOG_INFO(MODULE_WS, "🔄 Reconnection attempt %d (delay: %lu ms)", 
                 reconnectAttempts + 1, delay);
        lastReconnectAttempt = now;
        reconnectAttempts++;
        connect();
      }
      return;
    }
    
    if (!wsClient->isConnected()) {
      LOG_WARN(MODULE_WS, "⚠️ WebSocket connection lost");
      state = WS_DISCONNECTED;
      return;
    }
    
    // Send ping to keep connection alive
    if (millis() - lastPingTime >= PING_INTERVAL) {
      StaticJsonDocument<64> pingDoc;
      pingDoc["type"] = "ping";
      String pingMsg;
      serializeJson(pingDoc, pingMsg);
      wsClient->sendText(pingMsg);
      lastPingTime = millis();
      LOG_DEBUG(MODULE_WS, "🏓 Ping sent");
    }
    
    // Read incoming messages
    String message;
    if (wsClient->readMessage(message)) {
      processMessage(message);
    }
  }
  
  bool sendVehicleData(float lat, float lon, float speed, int satellites, 
                       const String& timestamp) {
    if (state != WS_SUBSCRIBED) {
      LOG_WARN(MODULE_WS, "Not ready to send (state: %s)", getStateString());
      return false;
    }
    
    // Create compact JSON
    StaticJsonDocument<384> doc;
    
    doc["type"] = "items";
    doc["collection"] = "vehicle_datas";
    doc["action"] = "create";
    
    JsonObject data = doc.createNestedObject("data");
    data["gps_id"] = GPS_ID;
    data["latitude"] = String(lat, 6);  // 6 decimal places
    data["longitude"] = String(lon, 6);
    data["speed"] = (int)speed;
    data["satellites_used"] = satellites;
    data["timestamp"] = timestamp;
    data["battery_level"] = 12.5;
    
    String message;
    serializeJson(doc, message);
    
    LOG_DEBUG(MODULE_WS, "📤 Sending GPS data: %s", message.c_str());
    wsClient->sendText(message);
    
    return true;
  }
  
  // Test function to manually update relay
  void testRelayUpdate(bool newState) {
    LOG_INFO(MODULE_WS, "🧪 TEST: Manually triggering relay update to %s", 
             newState ? "ON" : "OFF");
    
    if (onRelayUpdateCallback) {
      onRelayUpdateCallback(newState);
    } else {
      LOG_ERROR(MODULE_WS, "❌ No relay callback registered!");
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
  
  bool getLastKnownRelayState() {
    return lastKnownRelayState;
  }
};

#endif // WEBSOCKET_MANAGER_H