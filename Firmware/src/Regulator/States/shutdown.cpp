#include "Regulator/States/shutdown.h"

Shutdown::Shutdown(Heimdall::DefaultStateInit& DefaultInitParams):
State(HEIMDALL_FLAGS::STATE_SHUTDOWN,DefaultInitParams.heimdallstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_regClosedAngle(DefaultInitParams.regClosedAngle)
{};

void Shutdown::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    m_regAdapter.arm(0); //Arm the servo
    m_regAdapter.execute(m_regClosedAngle); //Drive the E-Reg to its closed position.
    m_regAdapter.disarm(); //No reason to keep actuator armed
};

Types::EREGTypes::State_ptr_t Shutdown::update()
{
    return nullptr; //Remain in default state indefinitely. The transition away from default is accessed through the actuator command handler.
};

void Shutdown::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};