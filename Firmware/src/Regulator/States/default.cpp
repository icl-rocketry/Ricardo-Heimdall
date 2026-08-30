#include "Regulator/States/default.h"

Default::Default(Heimdall::DefaultStateInit& DefaultInitParams):
State(HEIMDALL_FLAGS::STATE_DEFAULT,DefaultInitParams.heimdallstatus),
_servoAdaptor(DefaultInitParams.servoAdaptor),
_servoClosedAngle(DefaultInitParams.servoClosedAngle)
{};

void Default::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    _servoAdaptor.arm(0); //Arm the servo
    _servoAdaptor.execute(_servoClosedAngle); //Drive the E-Reg to its closed position.
    _servoAdaptor.disarm(); //No reason to keep actuator armed
};

Types::EREGTypes::State_ptr_t Default::update()
{
    return nullptr; //Remain in default state indefinitely. The transition away from default is accessed through the actuator command handler.
};

void Default::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};