// Logger.cpp
#include "Logger.h"
#include <stdarg.h>

LogLevel Logger::currentLevel = LOG_INFO;
HardwareSerial* Logger::output = nullptr;
const char* Logger::levelStrings[] = {"ERROR", "WARN", "INFO", "DEBUG", "TRACE"};
char Logger::timeBuffer[32];

void Logger::init(HardwareSerial* serial, LogLevel level) {
  output = serial;
  currentLevel = level;
}

void Logger::setLevel(LogLevel level) {
  currentLevel = level;
}

LogLevel Logger::getLevel() {
  return currentLevel;
}

const char* Logger::getTimeString() {
  unsigned long ms = millis();
  unsigned long seconds = ms / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  snprintf(timeBuffer, sizeof(timeBuffer), "[%02lu:%02lu:%02lu.%03lu]", 
           hours % 24, minutes % 60, seconds % 60, ms % 1000);
  return timeBuffer;
}

void Logger::log(LogLevel level, const char* module, const char* format, ...) {
  if (output == nullptr || level > currentLevel) return;
  
  output->print(getTimeString());
  output->print(" [");
  output->print(levelStrings[level]);
  output->print("] [");
  output->print(module);
  output->print("] ");
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  output->println(buffer);
}

void Logger::error(const char* module, const char* format, ...) {
  if (output == nullptr || LOG_ERROR > currentLevel) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_ERROR, module, "%s", buffer);
}

void Logger::warn(const char* module, const char* format, ...) {
  if (output == nullptr || LOG_WARN > currentLevel) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_WARN, module, "%s", buffer);
}

void Logger::info(const char* module, const char* format, ...) {
  if (output == nullptr || LOG_INFO > currentLevel) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_INFO, module, "%s", buffer);
}

void Logger::debug(const char* module, const char* format, ...) {
  if (output == nullptr || LOG_DEBUG > currentLevel) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_DEBUG, module, "%s", buffer);
}

void Logger::trace(const char* module, const char* format, ...) {
  if (output == nullptr || LOG_TRACE > currentLevel) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  log(LOG_TRACE, module, "%s", buffer);
}