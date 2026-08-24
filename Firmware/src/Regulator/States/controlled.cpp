#include "Regulator/States/controlled.h"

Controlled::Controlled(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall):
State(HEIMDALL_FLAGS::STATE_CONTROLLED,DefaultInitParams.heimdallstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_defaultParams(DefaultInitParams),
m_Heimdall(Heimdall)
{};

void Controlled::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    m_regAdapter.arm(0);
    m_stateEntry = millis();
};

Types::EREGTypes::State_ptr_t Controlled::update()
{
    if(millis() - m_stateEntry < 500){
        m_regAdapter.execute(std::min(static_cast<uint32_t>(m_Heimdall.nextAngle()),m_Heimdall.getLowerMaxAngle())); //lower angle for the first half second after startup
    }
    else{
        m_regAdapter.execute(static_cast<uint32_t>(m_Heimdall.nextAngle())); 
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
};