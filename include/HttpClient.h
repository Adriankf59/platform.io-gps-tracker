// HttpClient.h

#ifndef HTTP_CLIENT_WRAPPER_H
#define HTTP_CLIENT_WRAPPER_H

#include <Arduino.h>
#include <ArduinoHttpClient.h>
#include <Client.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

class HttpClientWrapper {
private:
  Client& client;
  HttpClient* http;
  
public:
  HttpClientWrapper(Client& netClient);
  ~HttpClientWrapper();
  
  // HTTP operations
  bool get(const char* path, String& response);
  bool post(const char* path, const String& payload, String& response);
  bool patch(const char* path, const String& payload, String& response);
  
  // Test connectivity
  bool testConnection();
  
private:
  bool performRequest(const char* method, const char* path, 
                     const String* payload, String& response);
  void logHttpError(int error, const char* operation);
};

#endif // HTTP_CLIENT_WRAPPER_H
