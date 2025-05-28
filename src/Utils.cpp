// Utils.cpp
#include "Utils.h"
#include "Config.h"

void Utils::initWatchdog(uint32_t timeout_ms) {
  // Convert milliseconds to seconds
  uint32_t timeout_s = timeout_ms / 1000;
  if (timeout_s == 0) timeout_s = 1; // Minimum 1 second
  
  // Initialize watchdog timer with timeout in seconds
  esp_err_t err = esp_task_wdt_init(timeout_s, true);
  
  if (err != ESP_OK) {
    LOG_ERROR(MODULE_SYS, "Failed to initialize watchdog: %d", err);
    return;
  }
  
  // Add current task to watchdog
  err = esp_task_wdt_add(NULL);
  if (err != ESP_OK) {
    LOG_ERROR(MODULE_SYS, "Failed to add task to watchdog: %d", err);
    return;
  }
  
  LOG_INFO(MODULE_SYS, "Watchdog initialized with %lu seconds timeout", timeout_s);
}

void Utils::feedWatchdog() {
  esp_task_wdt_reset();
}

void Utils::formatISO8601(char* buffer, size_t size, int year, int month, int day,
                         int hour, int minute, int second, int utcOffset) {
  // Add UTC offset to hour
  hour += utcOffset;
  
  // Handle day rollover
  if (hour >= 24) {
    hour -= 24;
    day++;
    
    // Simple month-end handling
    int daysInMonth;
    switch (month) {
      case 2:  // February
        daysInMonth = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : 28;
        break;
      case 4: case 6: case 9: case 11:  // 30-day months
        daysInMonth = 30;
        break;
      default:  // 31-day months
        daysInMonth = 31;
        break;
    }
    
    // Handle month rollover
    if (day > daysInMonth) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }
  
  // Format with timezone offset
  snprintf(buffer, size, "%04d-%02d-%02dT%02d:%02d:%02d+%02d:00",
           year, month, day, hour, minute, second, utcOffset);
}

String Utils::formatUptime(unsigned long millis) {
  unsigned long seconds = millis / 1000;
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;
  
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%dh %dm %ds", hours, minutes, secs);
  return String(buffer);
}

int Utils::clearSerialBuffer(Stream& serial) {
  int clearedBytes = 0;
  while (serial.available()) {
    serial.read();
    clearedBytes++;
  }
  return clearedBytes;
}

void Utils::safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    feedWatchdog();
    yield();
    delay(10);
  }
}

const char* Utils::getSignalQualityString(int csq) {
  if (csq < 0) return "Unknown";
  if (csq < 10) return "Weak";
  if (csq < 15) return "OK";
  if (csq < 20) return "Good";
  return "Excellent";
}

const char* Utils::getHttpErrorString(int error) {
  switch(error) {
    case -1: return "CONNECTION_FAILED";
    case -2: return "API_UNAVAILABLE";
    case -3: return "CONNECTION_REFUSED";
    case -4: return "SEND_HEADER_FAILED";
    case -5: return "SEND_PAYLOAD_FAILED";
    case -6: return "NOT_CONNECTED";
    case -7: return "CONNECTION_LOST";
    case -8: return "SERVER_DIDN'T_RESPOND";
    case -9: return "TIMED_OUT";
    default: return "UNKNOWN_ERROR";
  }
}