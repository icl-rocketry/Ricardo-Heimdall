#include "States/idle.h"

Idle::Idle(Types::CoreTypes::SystemStatus_t& systemtatus, Types::CoreTypes::CommandHandler_t& commandhandler):
State(SYSTEM_FLAG::STATE_IDLE,systemtatus),
_commandhandler(commandhandler)
{};

void Idle::initialize()
{
    Types::CoreTypes::State_t::initialize(); // call parent initialize first!
};

Types::CoreTypes::State_ptr_t Idle::update()
{
    // if (millis()-prevLogMessageTime > 1000)
    // {
    //     RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Idle heartbeat!");
    //     prevLogMessageTime = millis();
    // }

    return nullptr;
};

void Idle::exit()
{
    Types::CoreTypes::State_t::exit(); // call parent exit last!
};