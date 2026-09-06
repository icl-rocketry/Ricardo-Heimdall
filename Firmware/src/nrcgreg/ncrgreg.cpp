#include "nrcgreg.h"
#include <math.h>
#include <Arduino.h>

#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/services_config.h"
#include "default.h"
#include "pressurise.h"
#include "shutdown.h"
#include "controlled.h"
#include "debug.h"
#include "halfabort.h"
#include <limits>

// Setup for the E-Reg Controller
void NRCGreg::setup()
{
    m_regServo.setup();
    buckOn();
    m_regServo.setAngleLims(0, m_regMaxOpenAngle);
    buckOff(1000); // turn buck off after 2 seconds
    m_GregMachine.initalize(std::make_unique<Default>(m_DefaultStateParams));
    m_networkmanager.registerService(static_cast<uint8_t>(Services::ID::Reg_Servo),m_regServo.getThisNetworkCallback());
}

void NRCGreg::buckOn()
{
    m_Buck.setEN(true);
    m_buckOffTime = std::numeric_limits<uint32_t>::max();
    m_prevBuckTime = millis();
}

void NRCGreg::buckOff(uint32_t deadline)
{
    m_buckOffTime = millis() + deadline;
}

void NRCGreg::buckManager()
{
    if (millis() - m_prevBuckTime > 10000){
        buckOn();
        m_buckOffTime = millis() + 1000;
        return;
    }

    if (millis() > m_buckOffTime)
    {
        m_Buck.setEN(false);
    }
}

float NRCGreg::getFeedbackP()
{
    return m_FeedbackPT.getPressure();
}

float NRCGreg::feedforward()
{
    const float n2_press_pascal = std::max(m_N2PT.getPressure(),0.0f) * 10e+5;

    float current_CdA = (m_ox_m_dot/m_ox_Rho) * (1/std::sqrt(n2_press_pascal)) * 0.673f;

    current_CdA *= m_Kc; // correction factor when we didnt have cda measurements

    float m = 1.4424e+06f; float c = 19.5771;

    float FF_angle = m*current_CdA + c;

    m_savedFF_angle = std::clamp(FF_angle, m_FF_min, m_FF_max);
    return m_savedFF_angle; // Set bounds on FF angle before returning.
}

float NRCGreg::proportional()
{
    const float non_dimensional_error = 1 - (getFeedbackP()/m_P_setpoint);

    const float angle_difference = (static_cast<float>(m_regFullBoreAngle) - static_cast<float>(m_regMinOpenAngle)) / 10;

    float proportional_angle = m_Kp * non_dimensional_error * angle_difference;

    m_savedProportional_angle = std::clamp(proportional_angle, m_proportional_min, m_proportional_max); // Clamp the proportional angle to be within the range of 0 to angle_difference

    return m_savedProportional_angle;
}

float NRCGreg::getFF() {
    return m_savedFF_angle;
}

float NRCGreg::getKp() {
    return m_savedProportional_angle;
}



// uint32_t NRCGreg::nextAngle()
// {

//     float error = m_P_setpoint - getFeedbackP(); // Calculate error in tank pressure

//     m_P_angle = (float)Kp() * (float)error;

//     uint32_t reg_angle = static_cast<uint32_t>((m_P_angle + feedforward()) * 10.0f);

//     return std::max(std::min(reg_angle, m_regMaxOpenAngle), m_regMinOpenAngle); // Set bounds on angle during operation.
// }

uint32_t NRCGreg::nextAngle()
{
    // This change prevents unsigned integer casting underflow.
    const float target_angle_scaled = (proportional() + feedforward()) * 10.0f;
    const float clamped_angle = std::clamp(target_angle_scaled, static_cast<float>(m_regMinOpenAngle), static_cast<float>(m_regMaxOpenAngle));

    return static_cast<uint32_t>(clamped_angle);
}

void NRCGreg::update()
{
    _value = m_GregStatus.getStatus();
    buckManager();
    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED) && !m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT))
    {
        m_GregMachine.changeState(std::make_unique<Default>(m_DefaultStateParams)); // Return to default if the controller is disarmed
    }

    m_GregMachine.update();

    checkPressures();
}

void NRCGreg::shutdown()
{
    m_GregMachine.changeState(std::make_unique<Shutdown>(m_DefaultStateParams, m_networkmanager));
}

void NRCGreg::halfabort()
{
    m_GregMachine.changeState(std::make_unique<Halfabort>(m_DefaultStateParams, m_halfAbortAngle));
}

void NRCGreg::checkPressures()
{
    // Check if any sensors are below the disconnect threshold
    checkDisconnect(getFeedbackP(), GREG_FLAGS::ERROR_FBP_DC, "Feedback PT");
    checkDisconnect(m_N2PT.getPressure(), GREG_FLAGS::ERROR_N2P_DC, "Local nitrogen PT");

    // Check if any sensors are above the critical overpressure threshold
    checkCOverPressure(getFeedbackP(), GREG_FLAGS::ERROR_CRITICALOVP, "Feedback PT - One or more pressures above the critical threshold!");

    // Check if any sensors are above the half abort overpressure threshold
    checkHOverPressure(getFeedbackP(), GREG_FLAGS::ERROR_HALFABORT, "Feedback PT - One or more pressures above the halfabort threshold!");

    // Assert the generic flags if any of the specific error flags are set
    checkGenericPTFlag(GREG_FLAGS::ERROR_FEEDBACK_P, "feedback pressure local", GREG_FLAGS::ERROR_FBP_DC);
    checkGenericPTFlag(GREG_FLAGS::ERROR_N2_P, "n2 tank", GREG_FLAGS::ERROR_N2P_DC);

    if (m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP) &&
        !m_GregStatus.flagSet(GREG_FLAGS::STATE_SHUTDOWN) &&
        !m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT))
    {
        shutdown();
        return;
    }

    if (m_GregStatus.flagSetOr(GREG_FLAGS::ERROR_HALFABORT, GREG_FLAGS::ERROR_FEEDBACK_P, GREG_FLAGS::ERROR_N2_P) && m_GregStatus.flagSet(GREG_FLAGS::STATE_CONTROLLED))
    {
        halfabort(); // Abort if any of the half abort conditions are met
        return;
    }
}

void NRCGreg::checkDisconnect(float value, GREG_FLAGS err_flag, std::string err_name)
{
    if (value < m_P_disconnect)
    {
        if (!m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.newFlag(err_flag, err_name + std::string(" disconnected!"));
            m_DC_count += 1; // Iterate counter that keeps track of how many sensors have disconnected
        }
    }

    else if (value > m_P_disconnect && m_GregStatus.flagSet(err_flag))
    {
        m_GregStatus.deleteFlag(err_flag, err_name + std::string(" reading back in expected range!"));
        m_DC_count -= 1;
    }
}

void NRCGreg::checkCOverPressure(float value, GREG_FLAGS err_flag, std::string err_name)
{
    if (value > m_P_full_abort)
    {
        if (!m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.newFlag(err_flag, err_name + std::string(" exceeded critical pressure!"));
        }
    }

    else if (value < m_P_full_abort && m_GregStatus.flagSet(err_flag))
    {
        m_GregStatus.deleteFlag(err_flag, err_name + std::string(" reading back below critical pressure!"));
    }
}

void NRCGreg::checkHOverPressure(float value, GREG_FLAGS err_flag, std::string err_name)
{
    if (value > m_P_half_abort)
    {
        if (!m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.newFlag(err_flag, err_name + std::string(" exceeded half abort pressure!"));
        }
    }

    else if (value < m_P_half_abort && m_GregStatus.flagSet(err_flag))
    {
        m_GregStatus.deleteFlag(err_flag, err_name + std::string(" reading back below half abort pressure!"));
    }
}

template <typename... Flags>
void NRCGreg::checkGenericPTFlag(GREG_FLAGS generic_flag, std::string err_name, Flags... err_flags)
{
    if (m_GregStatus.flagSetOr(err_flags...) && !m_GregStatus.flagSet(generic_flag))
    {
        m_GregStatus.newFlag(generic_flag, std::string("One or more errors found with ") + err_name + std::string("!"));
    }
    else if (!m_GregStatus.flagSetOr(err_flags...) && m_GregStatus.flagSet(generic_flag))
    {
        m_GregStatus.deleteFlag(generic_flag, std::string("Component ") + err_name + std::string(" is no longer in error state!"));
    }
}

void NRCGreg::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1: // Controlled command
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only go to the controlled state from default.
        {
            break;
        }
        m_GregMachine.changeState(std::make_unique<Controlled>(m_DefaultStateParams, *this)); // Can always shut down
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Test Start");
        break;
    }
    case 2: // Shutdown command
    {
        m_GregMachine.changeState(std::make_unique<Shutdown>(m_DefaultStateParams, m_networkmanager)); // Can always shut down
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3: // Debug command
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only debug from default.
        {
            break;
        }
        // DEBUG COMMAND
        m_GregMachine.changeState(std::make_unique<Debug>(m_DefaultStateParams));
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    case 4: // Pressurise command
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only pressurise from default.
        {
            break;
        }

        m_GregMachine.changeState(std::make_unique<Pressurise>(m_DefaultStateParams, *this));
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pressurisation Start");
        break;
    }
    }
}

// Extra states for use during system debugging
void NRCGreg::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr)
{
    SimpleCommandPacket command_packet(*packetptr);
    switch (static_cast<uint8_t>(commandID))
    {
    case 6:
    {
        if (m_GregStatus.flagSet(GREG_FLAGS::STATE_DEBUG))
        {
            m_regAdapter.execute(command_packet.arg);
        }
        break;
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