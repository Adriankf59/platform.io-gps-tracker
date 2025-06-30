// Logger.cpp - Implementasi Sistem Logging
#include "Logger.h"
#include <stdarg.h>

// Inisialisasi static members
LogLevel Logger::currentLevel = LOG_INFO;
Stream* Logger::output = nullptr;
const char* Logger::levelStrings[] = {
  "ERROR",  // Merah (jika terminal support)
  "WARN ",  // Kuning  
  "INFO ",  // Hijau
  "DEBUG",  // Biru
  "TRACE"   // Abu-abu
};
char Logger::timeBuffer[32];

// Inisialisasi logger
void Logger::init(Stream* stream, LogLevel level) {
  output = stream;
  currentLevel = level;
  
  // Log pesan inisialisasi
  if (output != nullptr) {
    output->println("\n========================================");
    output->println("Logger initialized");
    output->print("Log level: ");
    output->println(levelStrings[level]);
    output->println("========================================\n");
  }
}

// Mengatur level logging
void Logger::setLevel(LogLevel level) {
  currentLevel = level;
  info("LOGGER", "Log level changed to: %s", levelStrings[level]);
}

// Mendapatkan level logging saat ini
LogLevel Logger::getLevel() {
  return currentLevel;
}

// Mengecek apakah level tertentu aktif
bool Logger::isEnabled(LogLevel level) {
  return output != nullptr && level <= currentLevel;
}

// Format waktu dalam [HH:MM:SS.mmm]
const char* Logger::getTimeString() {
  unsigned long ms = millis();
  unsigned long totalSeconds = ms / 1000;
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  unsigned long milliseconds = ms % 1000;
  
  snprintf(timeBuffer, sizeof(timeBuffer), "[%02lu:%02lu:%02lu.%03lu]", 
           hours % 100, minutes, seconds, milliseconds);
  return timeBuffer;
}

// Fungsi logging utama
void Logger::log(LogLevel level, const char* module, const char* format, ...) {
  if (!isEnabled(level)) return;
  
  // Print timestamp
  output->print(getTimeString());
  output->print(" ");
  
  // Print level dengan padding
  output->print("[");
  output->print(levelStrings[level]);
  output->print("] ");
  
  // Print module dengan padding fixed width
  output->print("[");
  output->printf("%-8s", module);  // Left-aligned, 8 karakter
  output->print("] ");
  
  // Print formatted message
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  output->println(buffer);
  
  // Auto-flush untuk ERROR
  if (level == LOG_ERROR) {
    flush();
  }
}

// Force flush output buffer
void Logger::flush() {
  if (output != nullptr) {
    output->flush();
  }
}

// Implementasi fungsi helper untuk setiap level
void Logger::error(const char* module, const char* format, ...) {
  if (!isEnabled(LOG_ERROR)) return;
  
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_ERROR, module, "%s", buffer);
}

void Logger::warn(const char* module, const char* format, ...) {
  if (!isEnabled(LOG_WARN)) return;
  
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_WARN, module, "%s", buffer);
}

void Logger::info(const char* module, const char* format, ...) {
  if (!isEnabled(LOG_INFO)) return;
  
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_INFO, module, "%s", buffer);
}

void Logger::debug(const char* module, const char* format, ...) {
  if (!isEnabled(LOG_DEBUG)) return;
  
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_DEBUG, module, "%s", buffer);
}

void Logger::trace(const char* module, const char* format, ...) {
  if (!isEnabled(LOG_TRACE)) return;
  
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_TRACE, module, "%s", buffer);
}