// HttpClient.cpp

#include "HttpClient.h"

HttpClientWrapper::HttpClientWrapper(TinyGsmClient& gsmClient) 
  : client(gsmClient), http(nullptr) {
}

HttpClientWrapper::~HttpClientWrapper() {
  if (http) {
    delete http;
  }
}

bool HttpClientWrapper::get(const char* path, String& response) {
  return performRequest("GET", path, nullptr, response);
}

bool HttpClientWrapper::post(const char* path, const String& payload, String& response) {
  return performRequest("POST", path, &payload, response);
}

bool HttpClientWrapper::patch(const char* path, const String& payload, String& response) {
  return performRequest("PATCH", path, &payload, response);
}

bool HttpClientWrapper::performRequest(const char* method, const char* path,
                                      const String* payload, String& response) {
  // Ensure path starts with /
  String fullPath = path;
  if (!fullPath.startsWith("/")) {
    fullPath = "/" + fullPath;
  }

  LOG_INFO(MODULE_HTTP, "→ %s %s://%s:%d%s", method, "http", SERVER_HOST, SERVER_PORT, fullPath.c_str());
  if (payload && payload->length() > 0) {
    LOG_DEBUG(MODULE_HTTP, "Request payload size: %d bytes", payload->length());
  }
  
  // Create new HTTP client for each request
  if (http) {
    delete http;
  }
  http = new HttpClient(client, SERVER_HOST, SERVER_PORT);
  http->setTimeout(HTTP_TIMEOUT);
  
  Utils::feedWatchdog();
  
  // Start request
  int err = 0;
  
  if (strcmp(method, "GET") == 0) {
    http->beginRequest();
    http->get(fullPath);
    http->endRequest();
  } else if (strcmp(method, "POST") == 0 && payload) {
    http->beginRequest();
    http->post(fullPath);
    http->sendHeader("Content-Type", "application/json");
    http->sendHeader("Content-Length", String(payload->length()));
    http->beginBody();
    http->print(*payload);
    http->endRequest();
  } else if (strcmp(method, "PATCH") == 0 && payload) {
    http->beginRequest();
    http->patch(fullPath);
    http->sendHeader("Content-Type", "application/json");
    http->sendHeader("Content-Length", String(payload->length()));
    http->beginBody();
    http->print(*payload);
    http->endRequest();
  }
  
  Utils::feedWatchdog();
  
  // Check for connection errors
  if (err != 0) {
    logHttpError(err, method);
    http->stop();
    return false;
  }
  
  // Get response
  int statusCode = http->responseStatusCode();
  response = http->responseBody();
  
  LOG_INFO(MODULE_HTTP, "← Response: %d (%s), body: %d bytes",
           statusCode,
           (statusCode >= 200 && statusCode < 300) ? "SUCCESS" : "ERROR",
           response.length());
  
  if (response.length() > 0 && response.length() < 200) {
    LOG_DEBUG(MODULE_HTTP, "Response body: %s", response.c_str());
  }
  
  // Close connection
  http->stop();
  delay(500);
  
  // Check status code
  if (statusCode >= 200 && statusCode < 300) {
    LOG_INFO(MODULE_HTTP, "✓ HTTP %s request successful", method);
    return true;
  } else {
    LOG_ERROR(MODULE_HTTP, "✗ HTTP %s failed with status %d", method, statusCode);
    if (response.length() > 0) {
      LOG_DEBUG(MODULE_HTTP, "Error response: %s", response.c_str());
    }
    return false;
  }
}

void HttpClientWrapper::logHttpError(int error, const char* operation) {
  LOG_ERROR(MODULE_HTTP, "%s error: %d (%s)", 
            operation, error, Utils::getHttpErrorString(error));
}

bool HttpClientWrapper::testConnection() {
  LOG_INFO(MODULE_HTTP, "Testing server connectivity...");
  
  String response;
  bool result = get("/", response);
  
  if (result) {
    LOG_INFO(MODULE_HTTP, "Server connectivity test successful");
  } else {
    LOG_ERROR(MODULE_HTTP, "Server connectivity test failed");
  }
  
  return result;
}