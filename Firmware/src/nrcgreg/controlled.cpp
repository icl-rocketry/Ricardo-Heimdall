#include "Controlled.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "system.h"
#include "shutdown.h"


Controlled::Controlled(Greg::DefaultStateInit& DefaultInitParams, NRCGreg& Greg):
State(GREG_FLAGS::STATE_CONTROLLED,DefaultInitParams.gregstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_defaultParams(DefaultInitParams),
m_Greg(Greg)
{};

void Controlled::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    m_regAdapter.arm(0);
    m_stateEntry = millis();
    m_Greg.buckOn();
};

Types::EREGTypes::State_ptr_t Controlled::update()
{
    if(millis() - m_stateEntry < 500){
        m_regAdapter.execute(std::min(static_cast<uint32_t>(m_Greg.nextAngle()),m_Greg.getLowerMaxAngle())); //lower angle for the first half second after startup
    }
    else{
        m_regAdapter.execute(static_cast<uint32_t>(m_Greg.nextAngle())); 
    }

    // if(millis() - m_stateEntry > m_cutoffTime){
    //     return std::make_unique<Shutdown>(m_defaultParams);
    // }
    return nullptr;
};

void Controlled::exit()
{   
    m_regAdapter.disarm();
    Types::EREGTypes::State_t::exit(); // call parent exit last!
    m_Greg.buckOff(2000);
};