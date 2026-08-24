#pragma once

#include <math.h>
#include <Arduino.h>

#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>
#include <librrc/Remote/nrcremotethermocouple.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/fsm/statemachine.h>
#include <libriccore/riccorelogging.h>
#include <libriccore/filtering/movingAvg.h>

#include "Config/services_config.h"

#include "Regulator/heimdalltypes.h"
#include "Regulator/States/default.h"
#include "Regulator/States/pressurise.h"
#include "Regulator/States/shutdown.h"
#include "Regulator/States/controlled.h"
#include "Regulator/States/debug.h"
#include "Regulator/States/halfabort.h"

// template <RicCoreLoggingConfig::LOGGERS LOGGING_TARGET = RicCoreLoggingConfig::LOGGERS::SYS>
class NRCHeimdall : public NRCRemoteActuatorBase<NRCHeimdall>
{

    public:

        NRCHeimdall(RnpNetworkManager &networkmanager,
                    uint8_t pwmPin,
                    NRCRemotePTap& PT0,
                    NRCRemotePTap& PT1,
                    NRCRemotePTap& PT2,
                    Types::Thermocouple_t& TC0,
                    Types::Thermocouple_t& TC1
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _networkmanager(networkmanager),  
            _servoPWM(pwmPin),    
            _servo(_servoPWM, networkmanager, "Servo"),
            _servoAdaptor(0, _servo, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
            _pt0(PT0),
            _pt1(PT1),
            _pt2(PT2),
            _tc0(TC0),
            _tc1(TC1),
            _tankAvg(20)
            {};

        void setup();
        void update();

        //Controller functions

        /**
        * @brief Function that calculates the feedforward angle based on input pressure.
        * @return feedforward angle as a float. */
        float feedforward();

        /**
        * @brief Function that calculates the proportional gain based on input pressure.
        * @return KP as a float */
        float Kp();

        /**
        * @brief Function that calculates the next angle the controller should move to.
        * @return Next controlled angle as an integer. */
        uint32_t nextAngle();

        //Getters
        uint32_t getRegClosedAngle(){return m_regClosedAngle;};
        float getFuelTankP();
        uint32_t getRegAngle(){return _servo.getValue();};
        float getPAngle(){return m_P_angle;};

        Heimdall::PressuriseParams getPressuriseParams(){
            Heimdall::PressuriseParams Params = {getFuelTankP(), 
            m_regPressuriseAngle, 
            m_P_setpoint, 
            m_P_press_extra};
            return Params;
        }

        float getAvgP(){return _tankAvg.getAvg();};
        uint32_t getLowerMaxAngle(){return m_regMaxOpenFirstStart;};
        float getHalfAbortP(){return m_P_half_abort;};
        float getFullAbortP(){return m_P_full_abort;};

    protected:

        //Networking
        RnpNetworkManager &_networkmanager;
        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        LocalPWM _servoPWM;
        Types::Servo_t _servo;
        Types::ServoAdapter_t _servoAdaptor;

        NRCRemotePTap _pt0;
        NRCRemotePTap _pt1;
        NRCRemotePTap _pt2;

        Types::Thermocouple_t _tc0; 
        Types::Thermocouple_t _tc1;

        //Tank pressure moving average
        MovingAvg _tankAvg;

        void execute_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);


        //Pressure check helpers
        void checkPressures(); //Method to be called during update. Checks all pressures are within operating limits.
        void checkTemperatures(); //Method to be called during update. Checks all pressures are within operating limits.


        void checkDisconnect(float sensorvalue, HEIMDALL_FLAGS err_flag, std::string err_name);
        void checkCOverPressure(float sensorvalue, HEIMDALL_FLAGS err_flag, std::string err_name);
        void checkHOverPressure(float sensorvalue, HEIMDALL_FLAGS err_flag, std::string err_name);
        template<typename... Flags>
        void checkGenericPTFlag(HEIMDALL_FLAGS generic_flag, std::string err_name, Flags... err_flags); //Method asserts generic_flag if any of err_flags input are asserted, and deasserts generic if no err_flags are asserted.
        
        //Helperes to aid state transitions
        void shutdown();
        void halfabort();

        // FSM related stuff
        Types::EREGTypes::StateMachine_t m_HeimdallMachine;
        Types::EREGTypes::SystemStatus_t m_HeimdallStatus;

        Heimdall::DefaultStateInit m_DefaultStateParams = {m_HeimdallStatus, _servoAdaptor, m_regClosedAngle};

        // ---------- Controller Parameters ----------
        // FF Params
        float m_FF_min = 55.0;
        float m_FF_max = 80.0;
        float m_FF_0 = 34.0;
        float m_FF_Alpha = 5570.0;

        // KP calculation Params
        float m_Kp_min = 2.0;
        float m_Kp_max = 3.0;
        float m_Kp_0 = 1.143;
        float m_Kp_Beta = 222.9;

        // Controller setpoints
        float m_P_setpoint = 40; //Running pressure setpoint.
        float m_P_press_extra = 1.5; //Extra pressure to add during pressurisation to make sure setpoint is reached.

        // Operating pressure limits
        float m_P_disconnect = -10; //Below this value, the PT is considered disconnected.
        float m_P_half_abort = 57.5; //Above this value, a half abort will be triggered.
        float m_P_full_abort = 65; //Above this value, a full abort will be triggered.

        //        --- HARDWARE LIMITS ---
        //! NOTE - All angles are x10 to allow for 0.1 degree precision in servo movements while still using integers
        const uint32_t m_regClosedAngle = 0;
        const uint32_t m_regMaxOpenAngle = 850;
        const uint32_t m_regMaxOpenFirstStart = 600; //Lower maximum angle during the starting period of the controlled state to prevent pressure spikes.sss
        const uint32_t m_regMinOpenAngle = 400;
        const uint32_t m_halfAbortAngle = 400;
        uint32_t m_regPressuriseAngle = 350;

        //Variables to log out
        float m_P_angle;

        //Variable to track which pressure source we're using for the controller. 0 is local, 1 is remote.
        bool m_P_source = 0;

        //Variables to track how many sensors have disconnected or are not responding
        uint8_t m_DC_count = 0;
        uint8_t m_NORESP_count = 0;

        //Variable for updating network sensor time to prevent timeout straight away.
        uint32_t m_lastPollSlow = 0;

        //Half abort timeout
        uint32_t m_halfAbortTimeout = 500; //After this timeout, the half abort can be exited and normal operation resumed.

};