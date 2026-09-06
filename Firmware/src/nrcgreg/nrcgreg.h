#pragma once
/**
 * @file nrcgreg.h
 * @author Martin England
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief The greg class is responsible for all control related to the E-Reg.
 * @version 0.1
 * @date 2024-09-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <librrc/Remote/nrcremoteactuatorbase.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>
#include <Config/services_config.h>


#include <librrc/HAL/localpwm.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/fsm/statemachine.h>
#include <libriccore/riccorelogging.h>
#include <libriccore/filtering/movingAvg.h>
#include <SiC43x.h>
#include "gregtypes.h"

// template <RicCoreLoggingConfig::LOGGERS LOGGING_TARGET = RicCoreLoggingConfig::LOGGERS::SYS>
class NRCGreg : public NRCRemoteActuatorBase<NRCGreg>
{

    public:

        NRCGreg(RnpNetworkManager &networkmanager,
                    uint8_t regServoGPIO,
                    uint8_t regServoChannel,
                    NRCRemotePTap& FBPT,
                    NRCRemotePTap& N2PT,
                    SiC43x& Buck
                    ):
            NRCRemoteActuatorBase(networkmanager),
            m_networkmanager(networkmanager),
            m_reg_PWM(regServoGPIO,regServoChannel),
            m_regServo(m_reg_PWM,networkmanager,"Srvo0",0,0,1800,500,2500,0,1800), //! All angles x10 for better precision.
            m_regAdapter(0,m_regServo,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
            m_FeedbackPT(FBPT),
            m_N2PT(N2PT),
            m_Buck(Buck)
            {};

        void setup();
        void update();

        //Controller functions

        /**
        * @brief Function that calculates the proportional angle based on input pressure.
        * @return proportional angle as a float. */
        float proportional();

        /**
        * @brief Function that calculates the feedforward angle based on input pressure.
        * @return feedforward angle as a float. */
        float feedforward();

        /**
        * @brief Function that calculates the next angle the controller should move to.
        * @return Next controlled angle as an integer. */
        uint32_t nextAngle();

        //Getters
        uint32_t getRegClosedAngle(){return m_regClosedAngle;};
        float getFeedbackP();
        uint32_t getRegAngle(){return m_regServo.getValue();};
        float getPAngle(){return m_P_angle;};

        Greg::PressuriseParams getPressuriseParams(){
            Greg::PressuriseParams Params = {getFeedbackP(),
            m_regPressuriseAngle,
            m_P_setpoint,
            m_P_press_extra};
            return Params;
        }

        uint32_t getLowerMaxAngle(){return m_regMaxOpenFirstStart;};
        float getHalfAbortP(){return m_P_half_abort;};
        float getFullAbortP(){return m_P_full_abort;};

        void buckManager();
        void buckOn();
        void buckOff(uint32_t deadline);

        float getFF();
        float getKp();

        const bool isOxReg = false;
    protected:

        //Networking
        RnpNetworkManager &m_networkmanager;
        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        //NRC components
        //Actuators
        LocalPWM m_reg_PWM;
        Types::Servo_t m_regServo;
        Types::ServoAdapter_t m_regAdapter;

        //Connected locally
        //Sensors
        NRCRemotePTap& m_FeedbackPT;
        NRCRemotePTap& m_N2PT;

        SiC43x& m_Buck;

        void execute_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);


        //Pressure check helpers
        void checkPressures(); //Method to be called during update. Checks all pressures are within operating limits.
        void checkDisconnect(float sensorvalue, GREG_FLAGS err_flag, std::string err_name);
        void checkCOverPressure(float sensorvalue, GREG_FLAGS err_flag, std::string err_name);
        void checkHOverPressure(float sensorvalue, GREG_FLAGS err_flag, std::string err_name);
        template<typename... Flags>
        void checkGenericPTFlag(GREG_FLAGS generic_flag, std::string err_name, Flags... err_flags); //Method asserts generic_flag if any of err_flags input are asserted, and deasserts generic if no err_flags are asserted.

        //Helpers to aid state transitions
        void shutdown();
        void halfabort();

        // FSM related stuff
        Types::EREGTypes::StateMachine_t m_GregMachine;
        Types::EREGTypes::SystemStatus_t m_GregStatus;

        Greg::DefaultStateInit m_DefaultStateParams = {m_GregStatus, m_regAdapter, m_regClosedAngle, *this};

        // ---------- Controller Parameters ----------
        // FF Params
        const float m_FF_min = 13.0 + static_cast<float>(m_regClosedAngle) / 10; // deg
        const float m_FF_max = 28.0 + static_cast<float>(m_regClosedAngle) / 10; // deg

        // Proportional Params
        float m_proportional_min = -10.0; // deg
        float m_proportional_max =  10.0; // deg

        const float m_oxKp = 1;
        const float m_oxKc = 1;

        const float m_fuelKp = 1;
        const float m_fuelKc = 1;

        const float m_Kp = isOxReg ? m_oxKp : m_fuelKp;
        const float m_Kc = isOxReg ? m_oxKc : m_fuelKc;

        // Controller setpoints
        const float m_oxPSetpoint = 45;
        const float m_fuelPSetpoint = 45;

        const float m_P_setpoint = isOxReg ? m_oxPSetpoint : m_fuelPSetpoint; // Running pressure setpoint.
        float m_P_press_extra = 1.5; //Extra pressure to add during pressurisation to make sure setpoint is reached.

        // Operating pressure limits
        float m_P_disconnect = -10; //Below this value, the PT is considered disconnected.
        float m_P_half_abort = 53; //Above this value, a half abort will be triggered.
        float m_P_full_abort = 4; //Above this value, a full abort will be triggered.

        float m_savedFF_angle = 0;
        float m_savedProportional_angle = 0;

        //        --- HARDWARE LIMITS ---
        //! NOTE - All angles are x10 to allow for 0.1 degree precision in servo movements while still using integers

        const uint32_t m_fuelRegClosedAngle = 950;
        const uint32_t m_fuelRegMaxOpenAngle = 1400;
        const uint32_t m_fuelRegMaxOpenFirstStart = m_fuelRegMinOpenAngle + 50; //Lower maximum angle during the starting period of the controlled state to prevent pressure spikes.
        const uint32_t m_fuelRegFullBoreAngle = 1400;
        const uint32_t m_fuelRegMinOpenAngle = 1080; // cracking angle
        const uint32_t m_fuelHalfAbortAngle = m_fuelRegMinOpenAngle + 50;
        const uint32_t m_fuelRegPressuriseAngle = m_fuelRegMinOpenAngle;

        const uint32_t m_oxRegClosedAngle = 0;
        const uint32_t m_oxRegMaxOpenAngle = 450; // full bore
        const uint32_t m_oxRegMaxOpenFirstStart = m_oxRegMinOpenAngle + 50; //Lower maximum angle during the starting period of the controlled state to prevent pressure spikes.
        const uint32_t m_oxRegFullBoreAngle = 450;
        const uint32_t m_oxRegMinOpenAngle = 130; // cracking angle
        const uint32_t m_oxHalfAbortAngle = m_oxRegMinOpenAngle + 50;
        const uint32_t m_oxRegPressuriseAngle = m_oxRegMinOpenAngle;

        const uint32_t m_regClosedAngle = isOxReg ? m_oxRegClosedAngle : m_fuelRegClosedAngle;
        const uint32_t m_regMaxOpenAngle = isOxReg ? m_oxRegMaxOpenAngle : m_fuelRegMaxOpenAngle;
        const uint32_t m_regMaxOpenFirstStart = isOxReg ? m_oxRegMaxOpenFirstStart : m_fuelRegMaxOpenFirstStart; //Lower maximum angle during the starting period of the controlled state to prevent pressure spikes.
        const uint32_t m_regFullBoreAngle = isOxReg ? m_oxRegFullBoreAngle : m_fuelRegFullBoreAngle;
        const uint32_t m_regMinOpenAngle = isOxReg ? m_oxRegMinOpenAngle : m_fuelRegMinOpenAngle; // cracking angle
        const uint32_t m_halfAbortAngle = isOxReg ? m_oxHalfAbortAngle : m_fuelHalfAbortAngle;
        uint32_t m_regPressuriseAngle = isOxReg ? m_oxRegPressuriseAngle : m_fuelRegPressuriseAngle;

        uint32_t m_prevBuckTime = 0;
        uint32_t m_buckOffTime = 0;

        //Variables to log out
        float m_P_angle;

        //Variable to track which pressure source we're using for the controller. 0 is local, 1 is remote.
        bool m_P_source = 0;

        //Variables to track how many sensors have disconnected or are not responding
        uint8_t m_DC_count = 0;

        //Half abort timeout
        uint32_t m_halfAbortTimeout = 500; //After this timeout, the half abort can be exited and normal operation resumed.

        const float m_oxRho = 963.0;
        const float m_oxMDot = 2.35;

        const float m_fuelRho = 790.0;
        const float m_fuelMDot = 0.79;

        const float m_propRho = isOxReg ? m_oxRho : m_fuelRho; //SI
        const float m_propMDot = isOxReg ? m_oxMDot : m_fuelMDot; //SI

};