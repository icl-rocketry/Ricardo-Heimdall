/**
 * @file debug.h
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief Debug state where the ereg can be moved to any position.
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

class NRCHeimdall;

class Debug : public Types::EREGTypes::State_t
{
    public:
        Debug(Heimdall::DefaultStateInit& DefaultInitParams);

        void initialize() override;

        Types::EREGTypes::State_ptr_t update() override;

        void exit() override;

    private:

        Types::ServoAdapter_t& _servoAdaptor;
        uint32_t _servoClosedAngle;
};