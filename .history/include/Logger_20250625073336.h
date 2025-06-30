// Logger.h - Sistem Logging untuk ESP32 GPS Tracker
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Level logging dari yang paling penting ke yang paling detail
enum LogLevel {
  LOG_ERROR = 0,    // Error kritis yang harus ditangani
  LOG_WARN = 1,     // Peringatan yang perlu diperhatikan
  LOG_INFO = 2,     // Informasi operasional normal
  LOG_DEBUG = 3,    // Informasi debugging
  LOG_TRACE = 4     // Informasi sangat detail untuk tracing
};

class Logger {
private:
  static LogLevel currentLevel;
  static Stream* output;
  static const char* levelStrings[];
  static char timeBuffer[32];
  
  // Buffer untuk format string yang lebih besar
  static const size_t LOG_BUFFER_SIZE = 512;  // Diperbesar dari 256
  
public:
  // Inisialisasi logger dengan output stream dan level
  static void init(Stream* stream, LogLevel level = LOG_INFO);
  
  // Mengatur level logging (hanya log dengan level <= currentLevel yang ditampilkan)
  static void setLevel(LogLevel level);
  static LogLevel getLevel();
  
  // Fungsi logging utama dengan format printf-style
  static void log(LogLevel level, const char* module, const char* format, ...);
  
  // Fungsi helper untuk setiap level
  static void error(const char* module, const char* format, ...);
  static void warn(const char* module, const char* format, ...);
  static void info(const char* module, const char* format, ...);
  static void debug(const char* module, const char* format, ...);
  static void trace(const char* module, const char* format, ...);
  
  // Mendapatkan string waktu dalam format [HH:MM:SS.mmm]
  static const char* getTimeString();
  
  // Fungsi tambahan untuk improvement
  static void flush();  // Force flush output buffer
  static bool isEnabled(LogLevel level);  // Check if level is enabled
};

// Macro untuk memudahkan penggunaan
#define LOG_ERROR(module, ...) Logger::error(module, __VA_ARGS__)
#define LOG_WARN(module, ...) Logger::warn(module, __VA_ARGS__)
#define LOG_INFO(module, ...) Logger::info(module, __VA_ARGS__)
#define LOG_DEBUG(module, ...) Logger::debug(module, __VA_ARGS__)
#define LOG_TRACE(module, ...) Logger::trace(module, __VA_ARGS__)

// Macro tambahan untuk conditional logging
#define LOG_IF(condition, level, module, ...) \
  if (condition) Logger::level(module, __VA_ARGS__)

// Macro untuk log dengan automatic function name
#define LOG_FUNC_ENTRY(module) \
  LOG_TRACE(module, ">>> %s()", __FUNCTION__)
  
#define LOG_FUNC_EXIT(module) \
  LOG_TRACE(module, "<<< %s()", __FUNCTION__)

#endif // LOGGER_H