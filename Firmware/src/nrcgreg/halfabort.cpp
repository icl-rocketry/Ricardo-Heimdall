#include "halfabort.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "system.h"


Halfabort::Halfabort(Greg::DefaultStateInit& DefaultInitParams, uint32_t HalfAbortAngle):
State(GREG_FLAGS::STATE_HALFABORT,DefaultInitParams.gregstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_regClosedAngle(DefaultInitParams.regClosedAngle),
m_regHalfAbortAngle(HalfAbortAngle)
{};

void Halfabort::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    m_regAdapter.arm(0); //Arm the servo
    m_regAdapter.execute(m_regHalfAbortAngle); //Drive the E-Reg to the halfabort predefined angle
    m_regAdapter.disarm(); //No reason to keep actuator armed
};

Types::EREGTypes::State_ptr_t Halfabort::update()
{
    return nullptr; //The transition away from halfabort is accessed through the actuator command handler.
};

void Halfabort::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};