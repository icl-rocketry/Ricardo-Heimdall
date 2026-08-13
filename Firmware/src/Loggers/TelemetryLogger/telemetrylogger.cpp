#include "Loggers/TelemetryLogger/telemetrylogger.h"



TelemetryLogger::TelemetryLogger():
_file(nullptr),
internalLogCB()
{};

bool TelemetryLogger::initialize(std::unique_ptr<WrappedFile> file, std::string file_header, std::function<void(std::string_view message)> logcb)
{
    if (logcb)
    {
        internalLogCB = logcb;
    }

    if (file == nullptr){return false;};
    _file = std::move(file);
    initialized=true;
    const std::string header = file_header + "\n";
    std::vector<uint8_t> dataframe_bytes(header.begin(),header.end());
    _file->append(dataframe_bytes);
    return true;
}

void TelemetryLogger::log(TelemetryLogframe& logframe)
{
    if (!initialized){return;};
    if (!enabled){return;};

    std::string dataframe_string = logframe.stringify();

    std::vector<uint8_t> dataframe_bytes(dataframe_string.begin(),dataframe_string.end());

    //if there is any exception we want to force the user to re-initialize the file, as a wrapped file will
    //automatically close itself when an exception is thrown
    try{
        _file->append(dataframe_bytes);
    }
    catch(std::exception &e)
    {
        initialized=false;
        internalLogCB(e.what());
    }

}
