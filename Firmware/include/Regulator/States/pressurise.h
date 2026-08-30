/**
 * @file Pressurise.h
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief Open loop filling
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

class Pressurise : public Types::EREGTypes::State_t
{
    public:
        Pressurise(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall);

        void initialize() override;

        Types::EREGTypes::State_ptr_t update() override;

        void exit() override;

    private:
        NRCHeimdall& _Heimdall;
        Heimdall::DefaultStateInit& _DefaultStateParams;
        Heimdall::PressuriseParams _PressuriseParams;
};