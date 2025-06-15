// WebSocketManager.h - Fixed version with proper memory allocation
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
    
    LOG_ERROR(MODULE_WS, "Handshake failed");
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
  const unsigned long PING_INTERVAL = 45000;      // 45 seconds
  const unsigned long RECONNECT_BASE_DELAY = 5000;
  
  void subscribeToVehicle() {
    // Use smaller JSON document
    StaticJsonDocument<200> doc;
    doc["type"] = "subscribe";
    doc["collection"] = "vehicle";
    JsonObject query = doc.createNestedObject("query");
    JsonArray fields = query.createNestedArray("fields");
    fields.add("*");
    
    String message;
    serializeJson(doc, message);
    wsClient->sendText(message);
    
    LOG_INFO(MODULE_WS, "Subscription request sent");
  }
  
  void processMessage(const String& message) {
    // Log first part of message for debugging
    LOG_DEBUG(MODULE_WS, "Processing message (len=%d): %.100s...", 
              message.length(), message.c_str());
    
    // Use DynamicJsonDocument for large responses
    DynamicJsonDocument doc(4096); // Increased size
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      LOG_ERROR(MODULE_WS, "JSON parse error: %s (msg len: %d)", 
                error.c_str(), message.length());
      
      // If NoMemory error, try to handle basic message types
      if (error == DeserializationError::NoMemory) {
        // Try to extract just the type
        int typeStart = message.indexOf("\"type\":\"");
        if (typeStart > 0) {
          typeStart += 8;
          int typeEnd = message.indexOf("\"", typeStart);
          if (typeEnd > typeStart) {
            String msgType = message.substring(typeStart, typeEnd);
            LOG_INFO(MODULE_WS, "Message type: %s (too large to parse fully)", msgType.c_str());
            
            if (msgType == "subscription") {
              // Extract event type
              int eventStart = message.indexOf("\"event\":\"");
              if (eventStart > 0) {
                eventStart += 9;
                int eventEnd = message.indexOf("\"", eventStart);
                if (eventEnd > eventStart) {
                  String eventType = message.substring(eventStart, eventEnd);
                  if (eventType == "init") {
                    LOG_INFO(MODULE_WS, "Subscription confirmed (data too large to parse)");
                    state = WS_SUBSCRIBED;
                    
                    // Try to find our vehicle in the response
                    int gpsIdPos = message.indexOf(GPS_ID);
                    if (gpsIdPos > 0) {
                      // Look for relay_status near our GPS ID
                      int relayPos = message.indexOf("\"relay_status\"", gpsIdPos - 200);
                      if (relayPos > 0 && relayPos < gpsIdPos) {
                        relayPos = message.indexOf(":", relayPos) + 1;
                        int relayEnd = message.indexOf(",", relayPos);
                        if (relayEnd < 0) relayEnd = message.indexOf("}", relayPos);
                        
                        String relayValue = message.substring(relayPos, relayEnd);
                        relayValue.trim();
                        relayValue.replace("\"", "");
                        
                        bool newState = (relayValue == "ON");
                        LOG_INFO(MODULE_WS, "Found relay status: %s", relayValue.c_str());
                        
                        if (onRelayUpdateCallback) {
                          onRelayUpdateCallback(newState);
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
        return;
      }
      
      return;
    }
    
    // Normal JSON processing for smaller messages
    const char* type = doc["type"];
    
    if (strcmp(type, "subscription") == 0) {
      const char* event = doc["event"];
      
      if (strcmp(event, "init") == 0) {
        LOG_INFO(MODULE_WS, "Subscription initialized");
        state = WS_SUBSCRIBED;
        processInitialData(doc["data"]);
      } else if (strcmp(event, "create") == 0 || 
                 strcmp(event, "update") == 0) {
        LOG_INFO(MODULE_WS, "Vehicle update event: %s", event);
        processVehicleUpdate(doc["data"]);
      }
    } else if (strcmp(type, "auth") == 0) {
      const char* status = doc["status"];
      if (strcmp(status, "ok") == 0) {
        LOG_INFO(MODULE_WS, "Authentication successful");
        subscribeToVehicle();
      }
    } else if (strcmp(type, "error") == 0) {
      const char* errorMsg = doc["error"]["message"] | "Unknown error";
      LOG_ERROR(MODULE_WS, "Server error: %s", errorMsg);
    } else if (strcmp(type, "ping") == 0) {
      LOG_DEBUG(MODULE_WS, "Received ping from server");
    }
  }
  
  void processInitialData(JsonVariant data) {
    if (!data.is<JsonArray>()) {
      LOG_WARN(MODULE_WS, "Initial data is not an array");
      return;
    }
    
    JsonArray vehicles = data.as<JsonArray>();
    LOG_INFO(MODULE_WS, "Processing %d vehicles", vehicles.size());
    
    for (JsonObject vehicle : vehicles) {
      String gpsId = vehicle["gps_id"] | "";
      if (gpsId == GPS_ID) {
        String relayStatus = vehicle["relay_status"] | "OFF";
        bool newState = (relayStatus == "ON");
        
        LOG_INFO(MODULE_WS, "Found our vehicle - Relay: %s", relayStatus.c_str());
        
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
        String gpsId = item["gps_id"] | "";
        if (gpsId == GPS_ID) {
          String relayStatus = item["relay_status"] | "OFF";
          bool newState = (relayStatus == "ON");
          
          LOG_INFO(MODULE_WS, "Relay status update: %s", relayStatus.c_str());
          
          if (onRelayUpdateCallback) {
            onRelayUpdateCallback(newState);
          }
          break;
        }
      }
    } else if (data.is<JsonObject>()) {
      // Single item update
      JsonObject item = data.as<JsonObject>();
      String gpsId = item["gps_id"] | "";
      if (gpsId == GPS_ID) {
        String relayStatus = item["relay_status"] | "OFF";
        bool newState = (relayStatus == "ON");
        
        LOG_INFO(MODULE_WS, "Relay status update: %s", relayStatus.c_str());
        
        if (onRelayUpdateCallback) {
          onRelayUpdateCallback(newState);
        }
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
    LOG_INFO(MODULE_WS, "WebSocket Manager initialized");
  }
  
  void setOnRelayUpdate(void (*callback)(bool)) {
    onRelayUpdateCallback = callback;
  }
  
  bool connect() {
    if (state != WS_DISCONNECTED) {
      LOG_WARN(MODULE_WS, "Already connected or connecting");
      return false;
    }
    
    LOG_INFO(MODULE_WS, "Attempting WebSocket connection...");
    state = WS_CONNECTING;
    
    if (wsClient->connect(wsUrl)) {
      LOG_INFO(MODULE_WS, "WebSocket connected successfully");
      state = WS_CONNECTED;
      lastPingTime = millis();
      reconnectAttempts = 0;
      
      // Subscribe immediately
      subscribeToVehicle();
      
      return true;
    }
    
    LOG_ERROR(MODULE_WS, "WebSocket connection failed");
    state = WS_DISCONNECTED;
    return false;
  }
  
  void disconnect() {
    if (state != WS_DISCONNECTED) {
      wsClient->disconnect();
      state = WS_DISCONNECTED;
      LOG_INFO(MODULE_WS, "WebSocket disconnected");
    }
  }
  
  void update() {
    if (state == WS_DISCONNECTED) {
      // Handle reconnection with exponential backoff
      unsigned long now = millis();
      unsigned long delay = RECONNECT_BASE_DELAY * (1 << min(reconnectAttempts, 5));
      
      if (now - lastReconnectAttempt >= delay) {
        LOG_INFO(MODULE_WS, "Reconnection attempt %d (delay: %lu ms)", 
                 reconnectAttempts + 1, delay);
        lastReconnectAttempt = now;
        reconnectAttempts++;
        connect();
      }
      return;
    }
    
    if (!wsClient->isConnected()) {
      LOG_WARN(MODULE_WS, "WebSocket connection lost");
      state = WS_DISCONNECTED;
      return;
    }
    
    // Send ping to keep connection alive
    if (millis() - lastPingTime >= PING_INTERVAL) {
      wsClient->sendPing();
      lastPingTime = millis();
      LOG_DEBUG(MODULE_WS, "Ping sent");
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
    data["latitude"] = String(lat, 5);
    data["longitude"] = String(lon, 5);
    data["speed"] = (int)speed;
    data["satellites_used"] = satellites;
    data["timestamp"] = timestamp;
    data["battery_level"] = 12.5;
    // Don't include null fields to save space
    
    String message;
    serializeJson(doc, message);
    
    LOG_DEBUG(MODULE_WS, "Sending: %s", message.c_str());
    wsClient->sendText(message);
    
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
};

#endif // WEBSOCKET_MANAGER_H