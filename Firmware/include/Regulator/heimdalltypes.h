#pragma once
#include <cstdint>
#include <memory>

#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/fsm/state.h>
#include <libriccore/fsm/statemachine.h>

/**
 * @brief Templated struct with type aliases inside to provide convient type access. Some of the template paramters might require
 * forward declaration to prevent cylic dependancies.
 *
 * @tparam HEIMDALL_FLAGS_T Enum of system flags
 */
enum class HEIMDALL_FLAGS : uint32_t
{
    // state flags
    STATE_DEFAULT = (1 << 0),
    STATE_PRESSURISE = (1 << 1),
    STATE_CONTROLLED = (1 << 2),
    STATE_SHUTDOWN = (1 << 3),
    STATE_HALFABORT = (1 << 4),
    STATE_DEBUG = (1 << 5),

    // critical messages
    ERROR_CRITICALOVP = (1 << 6),   // Critical overpressure
    ERROR_HALFABORT = (1 << 7), // Half abort flag - can be triggered by overpressure or multiple sensor disconnects
    // Local fuel tank codes
    ERROR_FUELTANKP_LOCAL = (1 << 8),     // Abort triggered by some local fuel tank error
    ERROR_FTP_LOCAL_DC = (1 << 9),        // Trigerred by disconnect
    ERROR_FTP_LOCAL_COVP = (1 << 10),     // Critical Overpressure
    ERROR_FTP_LOCAL_HOVP = (1 << 11),     // Half abort overpressure
    // Remote fuel tank codes
    ERROR_FUELTANKP_REMOTE = (1 << 12),    // Abort triggered by some remote fuel tank error
    ERROR_FTP_REMOTE_DC = (1 << 13),        // Disconnect
    ERROR_FTP_REMOTE_NORESPONSE = (1 << 14),// No response
    ERROR_FTP_REMOTE_COVP = (1 << 15),       // Critical Overpressure
    ERROR_FTP_REMOTE_HOVP = (1 << 16),
    // Ox tank error codes
    ERROR_OXTANKP_REMOTE = (1 << 17),      // Some ox tank error
    ERROR_OXP_REMOTE_DC = (1 << 18),        // Disconnect
    ERROR_OXP_REMOTE_NORESPONSE = (1 << 19),// No response
    ERROR_OXP_REMOTE_COVP = (1 << 20),       // Critical Overpressure
    ERROR_OXP_REMOTE_HOVP = (1 << 21),       
    // Nitrogen tank error codes
    ERROR_N2P_REMOTE = (1 << 22),          // Some NITROGEN tank error
    ERROR_N2P_REMOTE_DC = (1 << 23),        // Disconnect
    ERROR_N2P_REMOTE_NORESPONSE = (1 << 24),// No response
};

template <typename HEIMDALL_FLAGS_T>
struct HeimdallTypes
{
    using SystemStatus_t = SystemStatus<HEIMDALL_FLAGS_T>;
    using State_t = State<HEIMDALL_FLAGS_T>;
    using State_ptr_t = std::unique_ptr<State_t>;
    using StateMachine_t = StateMachine<HEIMDALL_FLAGS_T>;
};

namespace Types
{
    using EREGTypes = HeimdallTypes<HEIMDALL_FLAGS>;
    using Servo_t = NRCRemoteServo<LocalPWM>;
    using ServoAdapter_t = RemoteActuatorAdapter<Types::Servo_t>;
    using Thermocouple_t = NRCRemoteThermocouple<MAX31856>;
};

namespace Heimdall
{
    struct DefaultStateInit
    {
        Types::EREGTypes::SystemStatus_t &heimdallstatus;
        Types::ServoAdapter_t &regAdapter;
        const uint32_t regClosedAngle;
    };
    
    struct PressuriseParams
    {
        float OxTankP;
        uint32_t PressAngle;
        float P_Setpoint;
        float P_extra;
    };
}