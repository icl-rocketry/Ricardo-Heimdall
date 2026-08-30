#include "Regulator/States/shutdown.h"

#include "Regulator/nrcheimdall.h"

Shutdown::Shutdown(Heimdall::DefaultStateInit& DefaultInitParams):
State(HEIMDALL_FLAGS::STATE_SHUTDOWN,DefaultInitParams.heimdallstatus),
_servoAdaptor(DefaultInitParams.servoAdaptor),
_servoClosedAngle(DefaultInitParams.servoClosedAngle)
{};

void Shutdown::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    _servoAdaptor.arm(0); //Arm the servo
    _servoAdaptor.execute(_servoClosedAngle); //Drive the E-Reg to its closed position.
    _servoAdaptor.disarm(); //No reason to keep actuator armed
};

Types::EREGTypes::State_ptr_t Shutdown::update()
{
    return nullptr; //Remain in default state indefinitely. The transition away from default is accessed through the actuator command handler.
};

void Shutdown::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};