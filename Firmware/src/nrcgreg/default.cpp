#include "default.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "system.h"


Default::Default(Greg::DefaultStateInit& DefaultInitParams):
State(GREG_FLAGS::STATE_DEFAULT,DefaultInitParams.gregstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_regClosedAngle(DefaultInitParams.regClosedAngle),
m_DefaultInitParams(DefaultInitParams)
{};

void Default::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    m_regAdapter.arm(0); //Arm the servo
    m_regAdapter.execute(m_regClosedAngle); //Drive the E-Reg to its closed position.
    m_regAdapter.disarm(); //No reason to keep actuator armed
    m_DefaultInitParams.Greg.buckOff(2000);
};

Types::EREGTypes::State_ptr_t Default::update()
{
    return nullptr; //Remain in default state indefinitely. The transition away from default is accessed through the actuator command handler.
};

void Default::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};