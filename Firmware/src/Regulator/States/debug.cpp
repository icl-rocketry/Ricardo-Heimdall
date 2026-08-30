#include "Regulator/States/debug.h"

#include "Regulator/nrcheimdall.h"

Debug::Debug(Heimdall::DefaultStateInit& DefaultInitParams):
State(HEIMDALL_FLAGS::STATE_DEBUG,DefaultInitParams.heimdallstatus),
_servoAdaptor(DefaultInitParams.servoAdaptor),
_servoClosedAngle(DefaultInitParams.servoClosedAngle)
{};

void Debug::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    _servoAdaptor.arm(0); //Arm the servo
};

Types::EREGTypes::State_ptr_t Debug::update()
{
    return nullptr; //Remain in debug state indefinitely. The transition away from debug is accessed through the actuator command handler.
};

void Debug::exit()
{
    _servoAdaptor.disarm(); //No reason to keep actuator armed
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};