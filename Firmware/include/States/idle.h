#pragma once

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"



class Idle : public Types::CoreTypes::State_t
{
    public:
        Idle(Types::CoreTypes::SystemStatus_t& systemtatus, Types::CoreTypes::CommandHandler_t& commandhandler);

        void initialize() override;

        Types::CoreTypes::State_ptr_t update() override;

        void exit() override;

    private:
        Types::CoreTypes::CommandHandler_t& _commandhandler;
        uint32_t prevLogMessageTime;
};