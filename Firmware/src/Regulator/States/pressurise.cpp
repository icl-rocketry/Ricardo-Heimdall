#include "Regulator/States/pressurise.h"

Pressurise::Pressurise(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall):
State(HEIMDALL_FLAGS::STATE_PRESSURISE,DefaultInitParams.heimdallstatus),
_Heimdall(Heimdall),
_DefaultStateParams(DefaultInitParams)
{};

void Pressurise::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    _DefaultStateParams.servoAdaptor.arm(0);
};

Types::EREGTypes::State_ptr_t Pressurise::update()
{
    _PressuriseParams = _Heimdall.getPressuriseParams();

    // Open reg valve to filling angle. Hold open until _lptankP reaches P_set and then close
    _DefaultStateParams.servoAdaptor.execute(_PressuriseParams.PressAngle);

    if (_PressuriseParams.OxTankP >= (_PressuriseParams.P_Setpoint + _PressuriseParams.P_extra))
    {
        return std::make_unique<Default>(_DefaultStateParams);
    }

    return nullptr;
};

void Pressurise::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};