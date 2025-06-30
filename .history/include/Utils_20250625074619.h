// Utils.h - Utilitas Sistem GPS Tracker
#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "Logger.h"

class Utils {
public:
  // ===== MANAJEMEN WATCHDOG =====
  // Inisialisasi watchdog timer dengan timeout dalam milidetik
  static void initWatchdog(uint32_t timeout_ms = 120000);
  
  // Reset watchdog timer (harus dipanggil secara berkala)
  static void feedWatchdog();
  
  // ===== FORMAT WAKTU =====
  // Format waktu ke ISO8601 (YYYY-MM-DDTHH:MM:SSZ)
  static void formatISO8601(char* buffer, size_t size, int year, int month, int day, 
                           int hour, int minute, int second, int utcOffset);
  
  // Format uptime menjadi string yang mudah dibaca (Xh Xm Xs)
  static String formatUptime(unsigned long millis);
  
  // ===== MANAJEMEN SERIAL =====
  // Bersihkan buffer serial dan return jumlah byte yang dibersihkan
  static int clearSerialBuffer(Stream& serial);
  
  // ===== DELAY DENGAN WATCHDOG =====
  // Delay yang aman dengan tetap feed watchdog
  static void safeDelay(unsigned long ms);
  
  // ===== HELPER RETRY OPERATION =====
  // Template function untuk retry operasi dengan delay
  template<typename Func>
  static bool retryOperation(const char* module, const char* operation, 
                            Func func, int maxRetries = 3, 
                            unsigned long retryDelay = 2000);
  
  // ===== INTERPRETASI SIGNAL =====
  // Konversi nilai CSQ ke string kualitas sinyal
  static const char* getSignalQualityString(int csq);
  
  // ===== HELPER BATERAI =====
  // Menentukan status baterai berdasarkan tegangan
  static const char* getBatteryStatus(float voltage);
  
  // ===== KONVERSI KOORDINAT =====
  // Format koordinat GPS untuk tampilan
  static String formatCoordinate(float coord, bool isLatitude);
  
  // ===== MEMORY INFO =====
  // Mendapatkan info memory ESP32
  static void printMemoryInfo();
  static uint32_t getFreeHeap();
  static uint32_t getMinFreeHeap();
  
  // ===== HELPER DEBUGGING =====
  // Print hex dump untuk debugging data serial
  static void hexDump(const char* desc, const uint8_t* data, size_t len);
};

// Template implementation harus di header
template<typename Func>
bool Utils::retryOperation(const char* module, const char* operation, 
                          Func func, int maxRetries, unsigned long retryDelay) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (attempt > 0) {
      LOG_INFO(module, "Percobaan #%d untuk %s", attempt + 1, operation);
      safeDelay(retryDelay);
    }
    
    if (func()) {
      if (attempt > 0) {
        LOG_INFO(module, "%s berhasil setelah %d percobaan", operation, attempt + 1);
      }
      return true;
    }
    
    if (attempt < maxRetries - 1) {
      LOG_WARN(module, "%s gagal, mencoba lagi...", operation);
    }
  }
  
  LOG_ERROR(module, "%s gagal setelah %d percobaan", operation, maxRetries);
  return false;
}

#endif // UTILS_H