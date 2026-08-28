#include "Pressurise.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "default.h"

Pressurise::Pressurise(Greg::DefaultStateInit& DefaultInitParams, NRCGreg& Greg):
State(GREG_FLAGS::STATE_PRESSURISE,DefaultInitParams.gregstatus),
m_Greg(Greg),
m_DefaultInitParams(DefaultInitParams)
{};

void Pressurise::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    m_DefaultInitParams.regAdapter.arm(0);
    m_Greg.buckOn();
};

Types::EREGTypes::State_ptr_t Pressurise::update()
{
    m_PressuriseParams = m_Greg.getPressuriseParams();

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
    m_Greg.buckOff(1000);
};