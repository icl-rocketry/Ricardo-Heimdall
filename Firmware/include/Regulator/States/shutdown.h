/**
 * @file shutdown.h
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief Shutdown state of the e-reg state machine. Compared to the default state, 
 * this state can not be exited directly. Switching away from this state requires disarming
 * the engine.
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

class Shutdown : public Types::EREGTypes::State_t
{
    public:
        Shutdown(Heimdall::DefaultStateInit& DefaultInitParams);

        void initialize() override;

        Types::EREGTypes::State_ptr_t update() override;

        void exit() override;

    private:

        Types::ServoAdapter_t& _servoAdaptor;
        uint32_t _servoClosedAngle;
};