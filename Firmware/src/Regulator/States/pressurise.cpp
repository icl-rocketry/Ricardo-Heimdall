#include "Regulator/States/pressurise.h"

Pressurise::Pressurise(Heimdall::DefaultStateInit& DefaultInitParams, NRCHeimdall& Heimdall):
State(HEIMDALL_FLAGS::STATE_PRESSURISE,DefaultInitParams.heimdallstatus),
m_Heimdall(Heimdall),
m_DefaultInitParams(DefaultInitParams)
{};

void Pressurise::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    m_DefaultInitParams.regAdapter.arm(0);
};

Types::EREGTypes::State_ptr_t Pressurise::update()
{
    m_PressuriseParams = m_Heimdall.getPressuriseParams();

    // Open reg valve to filling angle. Hold open until _lptankP reaches P_set and then close
    m_DefaultInitParams.regAdapter.execute(m_PressuriseParams.PressAngle);

    if (m_PressuriseParams.OxTankP >= (m_PressuriseParams.P_Setpoint + m_PressuriseParams.P_extra))
    {
        return std::make_unique<Default>(m_DefaultInitParams);
    }

    return nullptr;
};

void Pressurise::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};