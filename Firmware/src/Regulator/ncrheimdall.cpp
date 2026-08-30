#include "Regulator/nrcheimdall.h"

// Setup for the E-Reg Controller
void NRCHeimdall::setup()
{
    _servo.setup();
    _servo.setAngleLims(0, 850);
    _HeimdallMachine.initalize(std::make_unique<Default>(_DefaultStateParams));
}

//old
float NRCHeimdall::feedforward()
{
    float FF = _FF_0 + _FF_Alpha / getPressurantP();
    return std::max(std::min(FF, _FF_max), _FF_min); // Set bounds on FF angle before returning.
}

// new
// float NRCHeimdall::feedforward()
// {
//     float FF = _FF_0 + _FF_Alpha / getPressurantP();
//     return std::max(std::min(FF, _FF_max), _FF_min); // Set bounds on FF angle before returning.
// }

float NRCHeimdall::Kp()
{
    float Kp = _Kp_0 + _Kp_Beta / getPressurantP();

    return std::max(std::min(Kp, _Kp_max), _Kp_min); // Set bounds on Kp before returning.
}

uint32_t NRCHeimdall::nextAngle()
{

    float error = _P_setpoint - getTankP(); // Calculate error in tank pressure

    _P_angle = (float)Kp() * (float)error;

    uint32_t servo_angle = static_cast<uint32_t>((_P_angle + feedforward()) * 10.0f);

    return std::max(std::min(servo_angle, _servoMaxOpenAngle), _servoMinOpenAngle); // Set bounds on angle during operation.
}

uint32_t lastlog;

void NRCHeimdall::update()
{
    _value = _HeimdallStatus.getStatus();

    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED) && !_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT))
    {
        _HeimdallMachine.changeState(std::make_unique<Default>(_DefaultStateParams)); // Return to defualt if the engine is disarmed
    }

    _tankAvg.update(getTankP());

    _HeimdallMachine.update();


    checkPressures();
}

void NRCHeimdall::shutdown()
{
    _HeimdallMachine.changeState(std::make_unique<Shutdown>(_DefaultStateParams));
}

void NRCHeimdall::halfabort()
{
    _HeimdallMachine.changeState(std::make_unique<Halfabort>(_DefaultStateParams, _FF_min * 10));
}

void NRCHeimdall::checkPressures()
{
    // Check if any sensors are disconnect 
    checkDisconnect(getTankP(), HEIMDALL_FLAGS::ERROR_TANKP_DC, "Tank PT");
    checkDisconnect(getPressurantP(), HEIMDALL_FLAGS::ERROR_PRESSURANTP_DC, "Pressurant PT");

    // Check if any sensors are above the critical overpressure threshold
    checkCOverPressure(getTankP(), HEIMDALL_FLAGS::ERROR_TANKP_COVP, "Tank PT");

    // Check if any sensors are above the half abort overpressure threshold
    checkHOverPressure(getTankP(), HEIMDALL_FLAGS::ERROR_TANKP_HOVP, "Tank PT");

    // Assert the generic flags if any of the specific error flags are set
    checkGenericPTFlag(HEIMDALL_FLAGS::ERROR_TANKP, "tank", HEIMDALL_FLAGS::ERROR_TANKP_DC, HEIMDALL_FLAGS::ERROR_TANKP_COVP, HEIMDALL_FLAGS::ERROR_TANKP_HOVP);
    checkGenericPTFlag(HEIMDALL_FLAGS::ERROR_PRESSURANTP, "pressurant", HEIMDALL_FLAGS::ERROR_PRESSURANTP_DC);


    if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_TANKP_COVP) && !_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_CRITICALOVP))
    {
        _HeimdallStatus.newFlag(HEIMDALL_FLAGS::ERROR_CRITICALOVP, "Tank pressure is above the critical threshold!");
    }

    if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_TANKP_HOVP) && !_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_HALFABORT))
    {
        _HeimdallStatus.newFlag(HEIMDALL_FLAGS::ERROR_HALFABORT, "Tank pressure is above the half abort threshold!");
    }
    if (_HeimdallStatus.flagSetOr(HEIMDALL_FLAGS::ERROR_TANKP_DC, HEIMDALL_FLAGS::ERROR_PRESSURANTP_DC) && !_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_HALFABORT))
    {
        _HeimdallStatus.newFlag(HEIMDALL_FLAGS::ERROR_HALFABORT, "Half abort triggered by sensor disconnects or sensors not responding!");
    }

    if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_CRITICALOVP))
    {
        if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT))
        {
            return;
        }
        shutdown(); // Abort in the case of a critical overpressure event.
        return;
    }

    if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_HALFABORT) && _HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_CONTROLLED))
    {
        if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT))
        {
            return;
        }
        halfabort(); // Abort if any of the half abort conditions are met
        return;
    }

}

void NRCHeimdall::checkDisconnect(float value, HEIMDALL_FLAGS err_flag, std::string err_name) // will be current sensor so should be able to detect DC
{
    if (value < _P_disconnect)
    {
        if (!_HeimdallStatus.flagSet(err_flag))
        {
            _HeimdallStatus.newFlag(err_flag, err_name + std::string(" disconnected!"));
        }
    }

    else if (value > _P_disconnect && _HeimdallStatus.flagSet(err_flag))
    {
        _HeimdallStatus.deleteFlag(err_flag, err_name + std::string(" reading back in expected range!"));
    }
}

void NRCHeimdall::checkCOverPressure(float value, HEIMDALL_FLAGS err_flag, std::string err_name)
{
    if (value > _P_full_abort)
    {
        if (!_HeimdallStatus.flagSet(err_flag))
        {
            _HeimdallStatus.newFlag(err_flag, err_name + std::string(" exceeded critical pressure!"));
        }
    }

    else if (value < _P_full_abort && _HeimdallStatus.flagSet(err_flag))
    {
        _HeimdallStatus.deleteFlag(err_flag, err_name + std::string(" reading back below critical pressure!"));
    }
}

void NRCHeimdall::checkHOverPressure(float value, HEIMDALL_FLAGS err_flag, std::string err_name)
{
    if (value > _P_half_abort)
    {
        if (!_HeimdallStatus.flagSet(err_flag))
        {
            _HeimdallStatus.newFlag(err_flag, err_name + std::string(" exceeded half abort pressure!"));
        }
    }

    else if (value < _P_half_abort && _HeimdallStatus.flagSet(err_flag))
    {
        _HeimdallStatus.deleteFlag(err_flag, err_name + std::string(" reading back below half abort pressure!"));
    }
}

template <typename... Flags>
void NRCHeimdall::checkGenericPTFlag(HEIMDALL_FLAGS generic_flag, std::string err_name, Flags... err_flags)
{
    if (_HeimdallStatus.flagSetOr(err_flags...) && !_HeimdallStatus.flagSet(generic_flag))
    {
        _HeimdallStatus.newFlag(generic_flag, std::string("One or more errors found with ") + err_name + std::string("!"));
    }
    else if (!_HeimdallStatus.flagSetOr(err_flags...) && _HeimdallStatus.flagSet(generic_flag)){
        _HeimdallStatus.deleteFlag(generic_flag, std::string("Component ") + err_name + std::string(" is no longer in error state!"));
    }
}

void NRCHeimdall::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1: // Controlled command
    {
        if (!_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT)) // Can only go to the controlled state from default.
        {
            break;
        }
        _HeimdallMachine.changeState(std::make_unique<Controlled>(_DefaultStateParams, *this)); // Can always shut down
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Test Start");
        break;
    }
    case 2: // Shutdown command
    {
        _HeimdallMachine.changeState(std::make_unique<Shutdown>(_DefaultStateParams)); // Can always shut down
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3: // Debug command
    {
        if (!_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT)) // Can only debug from default.
        {
            break;
        }
        // DEBUG COMMAND
        _HeimdallMachine.changeState(std::make_unique<Debug>(_DefaultStateParams));
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    case 4: // Pressurise command
    {
        if (!_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT)) // Can only pressurise from default.
        {
            break;
        }

        _HeimdallMachine.changeState(std::make_unique<Pressurise>(_DefaultStateParams, *this));
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pressurisation Start");
        break;
    }
    }
}

// Extra states for use during system debugging
void NRCHeimdall::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr)
{
    SimpleCommandPacket command_packet(*packetptr);
    switch (static_cast<uint8_t>(commandID))
    {
    case 6:
    {
        if (_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEBUG))
        {
            _servoAdaptor.execute(command_packet.arg);
        }
        else
        {
            break;
        }
    }
    case 7: // command to set pressurise angle
    {
        if (command_packet.arg > (_servoPressuriseAngle + 50))
        {
            break;
        }
        else
        {
            _servoPressuriseAngle = command_packet.arg;
        }
        break;
    }

    default:
    {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
        break;
    }
    }
}