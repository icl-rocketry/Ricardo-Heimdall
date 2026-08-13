#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <exception>
#include <functional>

#include <libriccore/logging/loggers/loggerbase.h>
#include <libriccore/storage/wrappedfile.h>

#include "Loggers/TelemetryLogger/telemetrylogframe.h"

class TelemetryLogger : public LoggerBase
{
    public:
        TelemetryLogger();

        bool initialize(std::unique_ptr<WrappedFile> file, std::string file_header, std::function<void(std::string_view message)> logcb=nullptr);

        void log(TelemetryLogframe& logframe);

    private:

        std::unique_ptr<WrappedFile> _file;

        std::function<void(std::string_view message)> internalLogCB;

    public:

        class LogException : public std::runtime_error
        {
            public:
                using std::runtime_error::runtime_error;
        };

};
