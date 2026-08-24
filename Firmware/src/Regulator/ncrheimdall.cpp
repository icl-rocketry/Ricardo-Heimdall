#include "Regulator/nrcheimdall.h"

// Setup for the E-Reg Controller
void NRCHeimdall::setup()
{
    m_regServo.setup();
    m_regServo.setAngleLims(0, 850);
    m_HeimdallMachine.initalize(std::make_unique<Default>(m_DefaultStateParams));
}

float NRCHeimdall::feedforward()
{
    float FF = m_FF_0 + m_FF_Alpha / m_PressTankPoller.getVal();
    return std::max(std::min(FF, m_FF_max), m_FF_min); // Set bounds on FF angle before returning.
}

float NRCHeimdall::Kp()
{
    float Kp = m_Kp_0 + m_Kp_Beta / m_PressTankPoller.getVal();

    return std::max(std::min(Kp, m_Kp_max), m_Kp_min); // Set bounds on Kp before returning.
}

uint32_t NRCHeimdall::nextAngle()
{

    float error = m_P_setpoint - getFuelTankP(); // Calculate error in tank pressure

    m_P_angle = (float)Kp() * (float)error;

    uint32_t reg_angle = static_cast<uint32_t>((m_P_angle + feedforward()) * 10.0f);

    return std::max(std::min(reg_angle, m_regMaxOpenAngle), m_regMinOpenAngle); // Set bounds on angle during operation.
}

uint32_t lastlog;

void NRCHeimdall::update()
{
    _value = m_HeimdallStatus.getStatus();

    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED) && !m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT))
    {
        m_HeimdallMachine.changeState(std::make_unique<Default>(m_DefaultStateParams)); // Return to defualt if the engine is disarmed
    }

    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL))
    {
        if (millis() - m_lastPollSlow > 500)
        {
            try
            {
                m_PressTankPoller.update();
                m_FuelTankPoller.update();
                m_OxTankPoller.update();
            }
            catch (const std::exception &e)
            {}
            m_lastPollSlow = millis();
        }
    }

    m_FuelTankAvg.update(m_FuelPT.getPressure());

    m_HeimdallMachine.update();

    updateRemoteP();

    checkPressures();
}

void NRCHeimdall::shutdown()
{
    m_HeimdallMachine.changeState(std::make_unique<Shutdown>(m_DefaultStateParams));
}

void NRCHeimdall::halfabort()
{
    m_HeimdallMachine.changeState(std::make_unique<Halfabort>(m_DefaultStateParams, m_FF_min * 10));
}

void NRCHeimdall::checkPressures()
{
    // Check if any sensors are below the disconnect threshold
    checkDisconnect(m_FuelPT.getPressure(), HEIMDALL_FLAGS::ERROR_FTP_LOCAL_DC, "Local fuel tank PT");
    checkDisconnect(m_PressTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_N2P_REMOTE_DC, "Remote nitrogen PT");
    checkDisconnect(m_FuelTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_FTP_REMOTE_DC, "Remote fuel tank PT");
    checkDisconnect(m_OxTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_OXP_REMOTE_DC, "Remote ox tank PT");

    // Check if any sensors are above the critical overpressure threshold
    checkCOverPressure(m_FuelPT.getPressure(), HEIMDALL_FLAGS::ERROR_FTP_LOCAL_COVP, "Local fuel tank PT");
    checkCOverPressure(m_FuelTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_FTP_REMOTE_COVP, "Remote fuel tank PT");
    checkCOverPressure(m_OxTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_OXP_REMOTE_COVP, "Remote ox tank PT");

    // Check if any sensors are above the half abort overpressure threshold
    checkHOverPressure(m_FuelPT.getPressure(), HEIMDALL_FLAGS::ERROR_FTP_LOCAL_HOVP, "Local fuel tank PT");
    checkHOverPressure(m_FuelTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_FTP_REMOTE_HOVP, "Remote fuel tank PT");
    checkHOverPressure(m_OxTankPoller.getVal(), HEIMDALL_FLAGS::ERROR_OXP_REMOTE_HOVP, "Remote ox tank PT");

    // Assert the generic flags if any of the specific error flags are set
    checkGenericPTFlag(HEIMDALL_FLAGS::ERROR_FUELTANKP_LOCAL, "fuel tank local", HEIMDALL_FLAGS::ERROR_FTP_LOCAL_COVP, HEIMDALL_FLAGS::ERROR_FTP_LOCAL_DC, HEIMDALL_FLAGS::ERROR_FTP_LOCAL_HOVP);
    checkGenericPTFlag(HEIMDALL_FLAGS::ERROR_FUELTANKP_REMOTE, "fuel tank remote", HEIMDALL_FLAGS::ERROR_FTP_REMOTE_COVP, HEIMDALL_FLAGS::ERROR_FTP_REMOTE_DC, HEIMDALL_FLAGS::ERROR_FTP_REMOTE_HOVP, HEIMDALL_FLAGS::ERROR_FTP_REMOTE_NORESPONSE);
    checkGenericPTFlag(HEIMDALL_FLAGS::ERROR_OXTANKP_REMOTE, "ox tank", HEIMDALL_FLAGS::ERROR_OXP_REMOTE_COVP, HEIMDALL_FLAGS::ERROR_OXP_REMOTE_DC, HEIMDALL_FLAGS::ERROR_OXP_REMOTE_HOVP, HEIMDALL_FLAGS::ERROR_OXP_REMOTE_NORESPONSE);
    checkGenericPTFlag(HEIMDALL_FLAGS::ERROR_N2P_REMOTE, "n2 tank", HEIMDALL_FLAGS::ERROR_N2P_REMOTE_DC, HEIMDALL_FLAGS::ERROR_N2P_REMOTE_NORESPONSE);


    if (m_HeimdallStatus.flagSetOr(HEIMDALL_FLAGS::ERROR_FTP_LOCAL_COVP, HEIMDALL_FLAGS::ERROR_FTP_REMOTE_COVP, HEIMDALL_FLAGS::ERROR_OXP_REMOTE_COVP) && !m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_CRITICALOVP))
    {
        m_HeimdallStatus.newFlag(HEIMDALL_FLAGS::ERROR_CRITICALOVP, "One or more pressures above the critical threshold!");
    }

    if (m_HeimdallStatus.flagSetOr(HEIMDALL_FLAGS::ERROR_FTP_LOCAL_HOVP, HEIMDALL_FLAGS::ERROR_FTP_REMOTE_HOVP, HEIMDALL_FLAGS::ERROR_OXP_REMOTE_HOVP) && !m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_HALFABORT))
    {
        m_HeimdallStatus.newFlag(HEIMDALL_FLAGS::ERROR_HALFABORT, "One or more pressures above the half abort threshold!");
    }
    if ((m_DC_count > 2 || m_NORESP_count > 2) && !m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_HALFABORT))
    {
        m_HeimdallStatus.newFlag(HEIMDALL_FLAGS::ERROR_HALFABORT, "Half abort triggered by sensor disconnects or sensors not responding!");
    }
    // else
    // {
    //     m_HeimdallStatus.deleteFlag(HEIMDALL_FLAGS::ERROR_HALFABORT, "No half abort conditions are true.");
    // }


    if (m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_CRITICALOVP))
    {
        if (m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT))
        {
            return;
        }
        // shutdown(); // Abort in the case of a critical overpressure event.
        return;
    }

    if (m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::ERROR_HALFABORT) && m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_CONTROLLED))
    {
        if (m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT))
        {
            return;
        }
        halfabort(); // Abort if any of the half abort conditions are met
        return;
    }

}

void NRCHeimdall::checkDisconnect(float value, HEIMDALL_FLAGS err_flag, std::string err_name) // will be current sensor so should be able to detect DC
{
    if (value < m_P_disconnect)
    {
        if (!m_HeimdallStatus.flagSet(err_flag))
        {
            m_HeimdallStatus.newFlag(err_flag, err_name + std::string(" disconnected!"));
            m_DC_count += 1; // Iterate counter that keeps track of how many sensors have disconnected
        }
    }

    else if (value > m_P_disconnect && m_HeimdallStatus.flagSet(err_flag))
    {
        m_HeimdallStatus.deleteFlag(err_flag, err_name + std::string(" reading back in expected range!"));
        m_DC_count -= 1;
    }
}

void NRCHeimdall::checkCOverPressure(float value, HEIMDALL_FLAGS err_flag, std::string err_name)
{
    if (value > m_P_full_abort)
    {
        if (!m_HeimdallStatus.flagSet(err_flag))
        {
            m_HeimdallStatus.newFlag(err_flag, err_name + std::string(" exceeded critical pressure!"));
        }
    }

    else if (value < m_P_full_abort && m_HeimdallStatus.flagSet(err_flag))
    {
        m_HeimdallStatus.deleteFlag(err_flag, err_name + std::string(" reading back below critical pressure!"));
    }
}

void NRCHeimdall::checkHOverPressure(float value, HEIMDALL_FLAGS err_flag, std::string err_name)
{
    if (value > m_P_half_abort)
    {
        if (!m_HeimdallStatus.flagSet(err_flag))
        {
            m_HeimdallStatus.newFlag(err_flag, err_name + std::string(" exceeded half abort pressure!"));
        }
    }

    else if (value < m_P_half_abort && m_HeimdallStatus.flagSet(err_flag))
    {
        m_HeimdallStatus.deleteFlag(err_flag, err_name + std::string(" reading back below half abort pressure!"));
    }
}

template <typename... Flags>
void NRCHeimdall::checkGenericPTFlag(HEIMDALL_FLAGS generic_flag, std::string err_name, Flags... err_flags)
{
    if (m_HeimdallStatus.flagSetOr(err_flags...) && !m_HeimdallStatus.flagSet(generic_flag))
    {
        m_HeimdallStatus.newFlag(generic_flag, std::string("One or more errors found with ") + err_name + std::string("!"));
    }
    else if (!m_HeimdallStatus.flagSetOr(err_flags...) && m_HeimdallStatus.flagSet(generic_flag)){
        m_HeimdallStatus.deleteFlag(generic_flag, std::string("Component ") + err_name + std::string(" is no longer in error state!"));
    }
}

void NRCHeimdall::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1: // Controlled command
    {
        if (!m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT)) // Can only go to the controlled state from default.
        {
            break;
        }
        m_HeimdallMachine.changeState(std::make_unique<Controlled>(m_DefaultStateParams, *this)); // Can always shut down
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Test Start");
        break;
    }
    case 2: // Shutdown command
    {
        m_HeimdallMachine.changeState(std::make_unique<Shutdown>(m_DefaultStateParams)); // Can always shut down
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3: // Debug command
    {
        if (!m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT)) // Can only debug from default.
        {
            break;
        }
        // DEBUG COMMAND
        m_HeimdallMachine.changeState(std::make_unique<Debug>(m_DefaultStateParams));
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    case 4: // Pressurise command
    {
        if (!m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEFAULT)) // Can only pressurise from default.
        {
            break;
        }

        m_HeimdallMachine.changeState(std::make_unique<Pressurise>(m_DefaultStateParams, *this));
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
        if (m_HeimdallStatus.flagSet(HEIMDALL_FLAGS::STATE_DEBUG))
        {
            m_regAdapter.execute(command_packet.arg);
        }
        else
        {
            break;
        }
    }
    case 7: // command to set pressurise angle
    {
        if (command_packet.arg > (m_regPressuriseAngle + 50))
        {
            break;
        }
        else
        {
            m_regPressuriseAngle = command_packet.arg;
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