#include "Regulator/States/debug.h"

Debug::Debug(Heimdall::DefaultStateInit& DefaultInitParams):
State(HEIMDALL_FLAGS::STATE_DEBUG,DefaultInitParams.heimdallstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_regClosedAngle(DefaultInitParams.regClosedAngle)
{};

void Debug::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    m_regAdapter.arm(0); //Arm the servo
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