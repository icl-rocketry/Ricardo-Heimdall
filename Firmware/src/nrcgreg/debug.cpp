#include "debug.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "system.h"


Debug::Debug(Greg::DefaultStateInit& DefaultInitParams):
State(GREG_FLAGS::STATE_DEBUG,DefaultInitParams.gregstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_regClosedAngle(DefaultInitParams.regClosedAngle),
m_DefaultInitParams(DefaultInitParams)
{};

void Debug::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    m_regAdapter.arm(0); //Arm the servo
    m_DefaultInitParams.Greg.buckOn();
};

Types::EREGTypes::State_ptr_t Debug::update()
{
    return nullptr; //Remain in debug state indefinitely. The transition away from debug is accessed through the actuator command handler.
};

void Debug::exit()
{
    m_regAdapter.disarm(); //No reason to keep actuator armed
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};