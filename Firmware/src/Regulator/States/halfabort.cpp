#include "Regulator/States/halfabort.h"

Halfabort::Halfabort(Heimdall::DefaultStateInit& DefaultInitParams, uint32_t HalfAbortAngle):
State(HEIMDALL_FLAGS::STATE_HALFABORT,DefaultInitParams.heimdallstatus),
_servoAdaptor(DefaultInitParams.servoAdaptor),
_servoClosedAngle(DefaultInitParams.servoClosedAngle),
_servoHalfAbortAngle(HalfAbortAngle)
{};

void Halfabort::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    _servoAdaptor.arm(0); //Arm the servo
    _servoAdaptor.execute(_servoHalfAbortAngle); //Drive the E-Reg to the halfabort predefined angle
    _servoAdaptor.disarm(); //No reason to keep actuator armed
};

Types::EREGTypes::State_ptr_t Halfabort::update()
{
    return nullptr; //The transition away from halfabort is accessed through the actuator command handler.
};

void Halfabort::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};