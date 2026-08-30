/**
 * @file halfabort.h
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief Half abort state for issues that aren't show stoppers.
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

class Halfabort : public Types::EREGTypes::State_t
{
    public:
        Halfabort(Heimdall::DefaultStateInit& DefaultInitParams, uint32_t HalfAbortAngle);

        void initialize() override;

        Types::EREGTypes::State_ptr_t update() override;

        void exit() override;

    private:

        Types::ServoAdapter_t& _servoAdaptor;
        uint32_t _servoClosedAngle;
        uint32_t _servoHalfAbortAngle;
};