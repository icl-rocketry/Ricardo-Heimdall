#pragma once

#include <HardwareSerial.h>
#include <memory>

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>
#include <libriccore/drivers/adc/ADS131M04.h>
#include <libriccore/drivers/sensors/MAX31856.h>

#include <librrc/HAL/localpwm.h>

#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>

#include "Commands/commands.h"
#include "Config/systemflags_config.h"
#include "Config/services_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"

#include "States/idle.h"
#include "Storage/sdfat_store.h"
#include "Loggers/TelemetryLogger/telemetrylogframe.h"

class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        CanBus<SYSTEM_FLAG> _canbus;
        
        
    private:

        LocalPWM _servo0_pwm;
        NRCRemoteServo<LocalPWM> _servo0;

        SPIClass _sd_spi;       //SPI for the SD card
        SPIClass _sensor_spi;   //SPI for the sensors

        ADS131M04 _adc0;

        NRCRemotePTap _pt0;
        NRCRemotePTap _pt1;
        NRCRemotePTap _pt2;

        MAX31856 _tc0;
        MAX31856 _tc1;

        SdFat_Store _primarysd;

        void setupSPI();
        void initializeLoggers();
        void logReadings();

        const std::string _log_path = "/Logs";

        uint32_t _telemetry_log_delta = 1000;
        uint32_t _prev_telemetry_log_time;


};