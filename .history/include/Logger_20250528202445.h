// Logger.h

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

enum LogLevel {
  LOG_ERROR = 0,
  LOG_WARN = 1,
  LOG_INFO = 2,
  LOG_DEBUG = 3,
  LOG_TRACE = 4
};

class Logger {
private:
  static LogLevel currentLevel;
  static Stream* output;
  static const char* levelStrings[];
  static char timeBuffer[32];
  
public:
  static void init(Stream* stream, LogLevel level = LOG_INFO);
  static void setLevel(LogLevel level);
  static LogLevel getLevel();
  
  static void log(LogLevel level, const char* module, const char* format, ...);
  static void error(const char* module, const char* format, ...);
  static void warn(const char* module, const char* format, ...);
  static void info(const char* module, const char* format, ...);
  static void debug(const char* module, const char* format, ...);
  static void trace(const char* module, const char* format, ...);
  
  static const char* getTimeString();
};

// Convenience macros
#define LOG_ERROR(module, ...) Logger::error(module, __VA_ARGS__)
#define LOG_WARN(module, ...) Logger::warn(module, __VA_ARGS__)
#define LOG_INFO(module, ...) Logger::info(module, __VA_ARGS__)
#define LOG_DEBUG(module, ...) Logger::debug(module, __VA_ARGS__)
#define LOG_TRACE(module, ...) Logger::trace(module, __VA_ARGS__)

#endif // LOGGER_H