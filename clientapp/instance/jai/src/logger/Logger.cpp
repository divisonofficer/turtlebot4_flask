#include <Logger.h>

std::string Logger::logPrefix = "";
std::string Logger::logSuffix = "";
bool Logger::logNewline = true;
LogLevel Logger::globalLogLevel = LogLevel::debug;
Logger::Logger(LogLevel level) : logLevel(level) {
  if (levelEnabled(logLevel)) {
    logStream << std::boolalpha;
    switch (level) {
      case LogLevel::debug:
        logStream << "[DEBUG] ";
        break;
      case LogLevel::info:
        logStream << "[INFO] ";
        break;
      case LogLevel::warning:
        logStream << "[WARNING] ";
        break;
      case LogLevel::error:
        logStream << "[ERROR] ";
        break;
    }
  }
}
Logger::~Logger() {
  if (levelEnabled(logLevel)) {
    std::cout << logPrefix << logStream.str() << logSuffix;
    if (logNewline) {
      std::cout << std::endl;
    }
  }
}