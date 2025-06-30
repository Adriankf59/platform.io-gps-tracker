// Utils.cpp - Implementasi Utilitas Sistem
#include "Utils.h"
#include "Config.h"

// ===== WATCHDOG MANAGEMENT =====
void Utils::initWatchdog(uint32_t timeout_ms) {
  // Konversi milidetik ke detik
  uint32_t timeout_s = timeout_ms / 1000;
  if (timeout_s == 0) timeout_s = 1; // Minimal 1 detik
  if (timeout_s > 30) timeout_s = 30; // Maksimal 30 detik untuk ESP32
  
  // Inisialisasi watchdog timer
  esp_err_t err = esp_task_wdt_init(timeout_s, true);
  
  if (err != ESP_OK) {
    LOG_ERROR(MODULE_SYS, "Gagal inisialisasi watchdog: %d", err);
    return;
  }
  
  // Tambahkan task saat ini ke watchdog
  err = esp_task_wdt_add(NULL);
  if (err != ESP_OK) {
    LOG_ERROR(MODULE_SYS, "Gagal menambahkan task ke watchdog: %d", err);
    return;
  }
  
  LOG_INFO(MODULE_SYS, "Watchdog diinisialisasi dengan timeout %lu detik", timeout_s);
}

void Utils::feedWatchdog() {
  esp_task_wdt_reset();
}

// ===== FORMAT WAKTU =====
void Utils::formatISO8601(char* buffer, size_t size, int year, int month, int day,
                          int hour, int minute, int second, int utcOffset) {
  // Tambahkan offset UTC ke jam
  hour += utcOffset;
  
  // Handle pergantian hari
  if (hour >= 24) {
    hour -= 24;
    day++;
    
    // Hitung hari dalam bulan
    int daysInMonth;
    switch (month) {
      case 2:  // Februari
        daysInMonth = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : 28;
        break;
      case 4: case 6: case 9: case 11:  // Bulan 30 hari
        daysInMonth = 30;
        break;
      default:  // Bulan 31 hari
        daysInMonth = 31;
        break;
    }
    
    // Handle pergantian bulan
    if (day > daysInMonth) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  } else if (hour < 0) {
    // Handle jam negatif (untuk offset negatif)
    hour += 24;
    day--;
    
    if (day < 1) {
      month--;
      if (month < 1) {
        month = 12;
        year--;
      }
      
      // Set hari ke hari terakhir bulan sebelumnya
      switch (month) {
        case 2:
          day = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : 28;
          break;
        case 4: case 6: case 9: case 11:
          day = 30;
          break;
        default:
          day = 31;
          break;
      }
    }
  }
  
  // Format dengan timezone - gunakan Z untuk UTC (offset 0)
  if (utcOffset == 0) {
    snprintf(buffer, size, "%04d-%02d-%02dT%02d:%02d:%02dZ",
             year, month, day, hour, minute, second);
  } else {
    // Format dengan offset
    char sign = (utcOffset >= 0) ? '+' : '-';
    int absOffset = abs(utcOffset);
    snprintf(buffer, size, "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:00",
             year, month, day, hour, minute, second, sign, absOffset);
  }
}

String Utils::formatUptime(unsigned long millis) {
  unsigned long seconds = millis / 1000;
  int days = seconds / 86400;
  int hours = (seconds % 86400) / 3600;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;
  
  char buffer[64];
  if (days > 0) {
    snprintf(buffer, sizeof(buffer), "%dd %dh %dm %ds", days, hours, minutes, secs);
  } else {
    snprintf(buffer, sizeof(buffer), "%dh %dm %ds", hours, minutes, secs);
  }
  return String(buffer);
}

// ===== SERIAL MANAGEMENT =====
int Utils::clearSerialBuffer(Stream& serial) {
  int clearedBytes = 0;
  while (serial.available()) {
    serial.read();
    clearedBytes++;
  }
  return clearedBytes;
}

// ===== SAFE DELAY =====
void Utils::safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    feedWatchdog();
    yield();  // Beri kesempatan task lain
    delay(10);
  }
}

// ===== SIGNAL QUALITY =====
const char* Utils::getSignalQualityString(int csq) {
  // CSQ: 0-31, 99 = tidak diketahui
  // 0-9 = Lemah, 10-14 = OK, 15-19 = Bagus, 20-31 = Sangat Bagus
  if (csq == 99 || csq < 0) return "Tidak Diketahui";
  if (csq < 10) return "Lemah";
  if (csq < 15) return "Cukup";
  if (csq < 20) return "Bagus";
  return "Sangat Bagus";
}

// ===== BATTERY STATUS =====
const char* Utils::getBatteryStatus(float voltage) {
  // Status baterai berdasarkan tegangan (12V system)
  if (voltage >= 12.6) return "Penuh";
  if (voltage >= 12.4) return "Baik";
  if (voltage >= 12.2) return "Sedang";
  if (voltage >= 12.0) return "Rendah";
  if (voltage >= 11.8) return "Kritis";
  return "Sangat Kritis";
}

// ===== COORDINATE FORMAT =====
String Utils::formatCoordinate(float coord, bool isLatitude) {
  char buffer[32];
  char direction;
  
  if (isLatitude) {
    direction = (coord >= 0) ? 'N' : 'S';
  } else {
    direction = (coord >= 0) ? 'E' : 'W';
  }
  
  float absCoord = fabs(coord);
  int degrees = (int)absCoord;
  float minutes = (absCoord - degrees) * 60;
  
  snprintf(buffer, sizeof(buffer), "%d°%.4f'%c", degrees, minutes, direction);
  return String(buffer);
}

// ===== MEMORY INFO =====
void Utils::printMemoryInfo() {
  uint32_t heapSize = ESP.getHeapSize();
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  uint32_t maxAllocHeap = ESP.getMaxAllocHeap();
  
  LOG_INFO(MODULE_SYS, "=== INFO MEMORY ===");
  LOG_INFO(MODULE_SYS, "Total Heap   : %u bytes", heapSize);
  LOG_INFO(MODULE_SYS, "Free Heap    : %u bytes (%.1f%%)", 
           freeHeap, (freeHeap * 100.0) / heapSize);
  LOG_INFO(MODULE_SYS, "Min Free Heap: %u bytes", minFreeHeap);
  LOG_INFO(MODULE_SYS, "Max Alloc    : %u bytes", maxAllocHeap);
  
  // Warning jika memory rendah
  if (freeHeap < 20000) {
    LOG_WARN(MODULE_SYS, "⚠️ Memory rendah! Free heap < 20KB");
  }
}

uint32_t Utils::getFreeHeap() {
  return ESP.getFreeHeap();
}

uint32_t Utils::getMinFreeHeap() {
  return ESP.getMinFreeHeap();
}

// ===== HEX DUMP =====
void Utils::hexDump(const char* desc, const uint8_t* data, size_t len) {
  if (!Logger::isEnabled(LOG_TRACE)) return;
  
  LOG_TRACE(MODULE_SYS, "HEX DUMP - %s (%d bytes):", desc, len);
  
  char hexStr[49];  // 16 * 3 + 1
  char asciiStr[17];
  
  for (size_t i = 0; i < len; i++) {
    if (i % 16 == 0) {
      if (i > 0) {
        LOG_TRACE(MODULE_SYS, "%04X: %-48s |%s|", 
                  i - 16, hexStr, asciiStr);
      }
      memset(hexStr, 0, sizeof(hexStr));
      memset(asciiStr, 0, sizeof(asciiStr));
    }
    
    snprintf(hexStr + (i % 16) * 3, 4, "%02X ", data[i]);
    asciiStr[i % 16] = (data[i] >= 32 && data[i] <= 126) ? data[i] : '.';
  }
  
  // Print baris terakhir
  if (len % 16 != 0) {
    LOG_TRACE(MODULE_SYS, "%04X: %-48s |%s|", 
              (len / 16) * 16, hexStr, asciiStr);
  }
}