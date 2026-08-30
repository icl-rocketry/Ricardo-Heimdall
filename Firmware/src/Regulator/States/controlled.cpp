#include "Regulator/States/controlled.h"

Controlled::Controlled(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall):
State(HEIMDALL_FLAGS::STATE_CONTROLLED,DefaultInitParams.heimdallstatus),
_servoAdaptor(DefaultInitParams.servoAdaptor),
_DefaultStateParams(DefaultInitParams),
_Heimdall(Heimdall)
{};

void Controlled::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    _servoAdaptor.arm(0);
    _stateEntry = millis();
};

Types::EREGTypes::State_ptr_t Controlled::update()
{
    if((millis() - _stateEntry) < 500) {
        _servoAdaptor.execute(std::min(static_cast<uint32_t>(_Heimdall.nextAngle()),_Heimdall.getLowerMaxAngle())); //lower angle for the first half second after startup
    }
    else {
        _servoAdaptor.execute(static_cast<uint32_t>(_Heimdall.nextAngle())); 
    }

    // if(millis() - m_stateEntry > m_cutoffTime){
    //     return std::make_unique<Shutdown>(m_defaultParams);
    // }
    return nullptr;
};

void Controlled::exit()
{   
    _servoAdaptor.disarm();
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};