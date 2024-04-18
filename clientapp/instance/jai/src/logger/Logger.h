#pragma once

#include <iostream>

enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error
};

class Logger
{
public:
    Logger(LogLevel level) : logLevel(level) {}

    template <typename T>
    Logger &operator<<(const T &data)
    {
        if (levelEnabled(logLevel))
        {
            std::cout << data;
        }
        return *this;
    }

    static void setLogLevel(LogLevel level)
    {
        globalLogLevel = level;
    }

private:
    LogLevel logLevel;

    static LogLevel globalLogLevel;

    static bool levelEnabled(LogLevel level)
    {
        return level >= globalLogLevel;
    }
};

inline LogLevel Logger::globalLogLevel = LogLevel::Debug;

extern Logger Debug;
extern Logger Info;
extern Logger Warning;
extern Logger Error;
