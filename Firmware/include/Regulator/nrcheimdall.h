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
            _pt0(PT0), // pressurant
            _pt1(PT1),
            _pt2(PT2), // ox/fuel
            _tc0(TC0),
            _tc1(TC1),
            _tankAvg(20)
            {};

        void setup();
        void update();

        float feedforward();
        float Kp();
        uint32_t nextAngle();

        //Getters
        uint32_t getServoClosedAngle(){return _servoClosedAngle;};
        float getPressurantP() { return _pt0.getProcessed(); }; // n2
        float getTankP() { return _pt0.getProcessed(); }; // ox/fuel
        uint32_t getServoAngle(){return _servo.getValue();};
        float getPAngle(){return _P_angle;};

        Heimdall::PressuriseParams getPressuriseParams(){
            Heimdall::PressuriseParams Params = {getTankP(), 
            _servoPressuriseAngle, 
            _P_setpoint, 
            _P_press_extra};
            return Params;
        }

        float getAvgP(){return _tankAvg.getAvg();};
        uint32_t getLowerMaxAngle(){return _servoMaxOpenFirstStart;};
        float getHalfAbortP(){return _P_half_abort;};
        float getFullAbortP(){return _P_full_abort;};

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
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);


        void checkPressures(); //Method to be called during update. Checks all pressures are within operating limits.
        void checkDisconnect(float sensorvalue, HEIMDALL_FLAGS err_flag, std::string err_name);
        void checkCOverPressure(float sensorvalue, HEIMDALL_FLAGS err_flag, std::string err_name);
        void checkHOverPressure(float sensorvalue, HEIMDALL_FLAGS err_flag, std::string err_name);
        template<typename... Flags>
        void checkGenericPTFlag(HEIMDALL_FLAGS generic_flag, std::string err_name, Flags... err_flags); //Method asserts generic_flag if any of err_flags input are asserted, and deasserts generic if no err_flags are asserted.
        
        //Helperes to aid state transitions
        void shutdown();
        void halfabort();

        // FSM related stuff
        Types::EREGTypes::StateMachine_t _HeimdallMachine;
        Types::EREGTypes::SystemStatus_t _HeimdallStatus;

        Heimdall::DefaultStateInit _DefaultStateParams = {_HeimdallStatus, _servoAdaptor, _servoClosedAngle};

        // ---------- Controller Parameters ----------
        // FF Params
        float _FF_min = 55.0;
        float _FF_max = 80.0;
        float _FF_0 = 34.0;
        float _FF_Alpha = 5570.0;

        // KP calculation Params
        float _Kp_min = 2.0;
        float _Kp_max = 3.0;
        float _Kp_0 = 1.143;
        float _Kp_Beta = 222.9;

        // Controller setpoints
        float _P_setpoint = 40; //Running pressure setpoint.
        float _P_press_extra = 1.5; //Extra pressure to add during pressurisation to make sure setpoint is reached.

        // Operating pressure limits
        float _P_disconnect = -10; //Below this value, the PT is considered disconnected.
        float _P_half_abort = 57.5; //Above this value, a half abort will be triggered.
        float _P_full_abort = 65; //Above this value, a full abort will be triggered.

        //        --- HARDWARE LIMITS ---
        //! NOTE - All angles are x10 to allow for 0.1 degree precision in servo movements while still using integers
        const uint32_t _servoClosedAngle = 0;
        const uint32_t _servoMaxOpenAngle = 850;
        const uint32_t _servoMaxOpenFirstStart = 600; //Lower maximum angle during the starting period of the controlled state to prevent pressure spikes.sss
        const uint32_t _servoMinOpenAngle = 400;
        const uint32_t _servoHalfAbortAngle = 400;
        uint32_t _servoPressuriseAngle = 350;

        //Variables to log out
        float _P_angle;

        //Half abort timeout
        uint32_t _halfAbortTimeout = 500; //After this timeout, the half abort can be exited and normal operation resumed.

};