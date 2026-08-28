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
 * @tparam GREG_FLAGS_T Enum of system flags
 */
enum class GREG_FLAGS : uint32_t
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
    // Feedback tank codes
    ERROR_FEEDBACK_P = (1 << 8),     // Abort triggered by some local fuel tank error
    ERROR_FBP_DC = (1 << 9),        // Trigerred by disconnect
    ERROR_FBP_COVP = (1 << 10),     // Critical Overpressure
    ERROR_FBP_HOVP = (1 << 11),     // Half abort overpressure    
    // Nitrogen tank error codes
    ERROR_N2_P = (1 << 22),          // Some NITROGEN tank error
    ERROR_N2P_DC = (1 << 23),        // Disconnect
};

template <typename GREG_FLAGS_T>
struct GregTypes
{
    using SystemStatus_t = SystemStatus<GREG_FLAGS_T>;
    using State_t = State<GREG_FLAGS_T>;
    using State_ptr_t = std::unique_ptr<State_t>;
    using StateMachine_t = StateMachine<GREG_FLAGS_T>;
};

namespace Types
{
    using EREGTypes = GregTypes<GREG_FLAGS>;
    using Servo_t = NRCRemoteServo<LocalPWM>;
    using ServoAdapter_t = RemoteActuatorAdapter<Types::Servo_t>;
    using ServoADPMap_t = std::array<ServoAdapter_t *, 1>;
};

//fORWARD DEC
class NRCGreg;
namespace Greg
{
    struct DefaultStateInit
    {
        Types::EREGTypes::SystemStatus_t &gregstatus;
        Types::ServoAdapter_t &regAdapter;
        const uint32_t regClosedAngle;
        NRCGreg& Greg;
    };
    
    struct PressuriseParams
    {
        float OxTankP;
        uint32_t PressAngle;
        float P_Setpoint;
        float P_extra;
    };
}