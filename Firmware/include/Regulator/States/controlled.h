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
        /**
         * @brief Idle state constructor. All states require the systemstatus object to be passed in, as well as any other system level objects required. For example, if
         * we want to control the available commands, we need to pass in the command handler from the riccoresystem.
         * 
         */
        Controlled(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall);

        /**
         * @brief Perform any initialization required for the state
         * 
         */
        void initialize() override;

        /**
         * @brief Function called every update cycle, use to implement periodic actions such as checking sensors. If nullptr is returned, the statemachine will loop the state,
         * otherwise pass a new state ptr to transition to a new state.
         * 
         * @return std::unique_ptr<State> 
         */
        Types::EREGTypes::State_ptr_t update() override;

        /**
         * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
         * 
         */
        void exit() override;

    private:
        uint32_t prevLogMessageTime;
        Types::ServoAdapter_t& m_regAdapter;
        Heimdall::DefaultStateInit& m_defaultParams;
        NRCHeimdall& m_Heimdall;
        uint32_t m_cutoffTime = 14500;
        uint32_t m_stateEntry = 0;
        float nextAngle();
};