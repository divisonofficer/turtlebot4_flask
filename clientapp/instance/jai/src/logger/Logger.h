#pragma once
#include <iostream>
#include <sstream>  // Include the necessary header file

enum class LogLevel { debug, info, warning, error };

class Logger {
 public:
  Logger(LogLevel level);

  template <typename T>
  Logger& operator<<(const T& data) {
    if (levelEnabled(logLevel)) {
      logStream << data;
    }
    return *this;
  }
  ~Logger();
  static void setLogLevel(LogLevel level) { globalLogLevel = level; }

  static void setLogPrefix(const std::string& prefix) { logPrefix = prefix; }

  static void setLogSuffix(const std::string& suffix) { logSuffix = suffix; }

  static void setLogNewline(bool newline) { logNewline = newline; }

 private:
  LogLevel logLevel;
  std::ostringstream logStream;

  static bool levelEnabled(LogLevel level) { return level >= globalLogLevel; }

  static LogLevel globalLogLevel;
  static std::string logPrefix;
  static std::string logSuffix;
  static bool logNewline;
};

#define Debug Logger(LogLevel::debug)
#define Info Logger(LogLevel::info)
#define Warning Logger(LogLevel::warning)
#define Error Logger(LogLevel::error)