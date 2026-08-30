/**
 * @file Controlled.h
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief Main controller state of the e reg state machine.
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Regulator/heimdalltypes.h"
#include "Regulator/nrcheimdall.h"
#include "Regulator/States/shutdown.h"

class Controlled : public Types::EREGTypes::State_t
{
    public:
        Controlled(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall);

        void initialize() override;

        Types::EREGTypes::State_ptr_t update() override;

        void exit() override;

    private:
        uint32_t prevLogMessageTime;
        Types::ServoAdapter_t& _servoAdaptor;
        Heimdall::DefaultStateInit& _DefaultStateParams;
        NRCHeimdall& _Heimdall;
        uint32_t _cutoffTime = 14500;
        uint32_t _stateEntry = 0;
        float nextAngle();
};