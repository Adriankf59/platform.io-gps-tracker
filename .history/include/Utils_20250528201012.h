// Utils.h

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "Logger.h"

class Utils {
public:
  // Watchdog management
  static void initWatchdog(uint32_t timeout_ms = 120000);
  static void feedWatchdog();
  
  // Time formatting
  static void formatISO8601(char* buffer, size_t size, int year, int month, int day, 
                           int hour, int minute, int second, int utcOffset);
  static String formatUptime(unsigned long millis);
  
  // Serial buffer management
  static int clearSerialBuffer(Stream& serial);
  
  // Delay with watchdog feed
  static void safeDelay(unsigned long ms);
  
  // Connection retry helper
  template<typename Func>
  static bool retryOperation(const char* module, const char* operation, 
                            Func func, int maxRetries = 3, 
                            unsigned long retryDelay = 2000);
  
  // Signal quality interpretation
  static const char* getSignalQualityString(int csq);
  
  // HTTP error string helper
  static const char* getHttpErrorString(int error);
};

// Template implementation must be in header
template<typename Func>
bool Utils::retryOperation(const char* module, const char* operation, 
                          Func func, int maxRetries, unsigned long retryDelay) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (attempt > 0) {
      LOG_INFO(module, "Retry #%d for %s", attempt, operation);
      safeDelay(retryDelay);
    }
    
    if (func()) {
      if (attempt > 0) {
        LOG_INFO(module, "%s succeeded after %d retries", operation, attempt);
      }
      return true;
    }
    
    if (attempt < maxRetries - 1) {
      LOG_WARN(module, "%s failed, retrying...", operation);
    }
  }
  
  LOG_ERROR(module, "%s failed after %d attempts", operation, maxRetries);
  return false;
}

#endif // UTILS_H