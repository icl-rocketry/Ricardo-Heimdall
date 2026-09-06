#include "system.h"

#include <memory>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

#include "Commands/commands.h"

#include "States/idle.h"

#include <cstdlib>

#include "Loggers/TelemetryLogger/telemetrylogframe.h"

static constexpr int VSPI_BUS_NUM = 0;
static constexpr int HSPI_BUS_NUM = 1;

System::System() : RicCoreSystem(Commands::command_map, Commands::defaultEnabledCommands, Serial),
                   canbus(systemstatus, PinMap::TxCan, PinMap::RxCan, 3),
                   SDSPI(VSPI_BUS_NUM),
                   SNSRSPI(HSPI_BUS_NUM),
                   TC0(SNSRSPI, PinMap::TC0_Cs),
                   TC1(SNSRSPI, PinMap::TC1_Cs),
                   ADC0(SNSRSPI, PinMap::ADC0_Cs, PinMap::ADC_CLK),
                   FB_PT(networkmanager, 0),
                   N2_PT(networkmanager, 2),
                   Buck(systemstatus, PinMap::BuckPGOOD, PinMap::BuckEN, true),
                   Heimdall(networkmanager, PinMap::ServoPWM, 50, FB_PT, N2_PT, Buck),
                   primarysd(SDSPI,PinMap::SdCs_0,SD_SCK_MHZ(20),false,&systemstatus){};

void System::systemSetup()
{
    //Base Ricardo setup
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);

    //Initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus, commandhandler));

    //Can bus setup
    canbus.setup();
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST, {1, 3});
    networkmanager.addInterface(&canbus);

    //Project specific feedback
    pinMode(PinMap::SdCs_0, OUTPUT);
    pinMode(PinMap::ADC0_Cs, OUTPUT);
    pinMode(PinMap::TC0_Cs, OUTPUT);
    pinMode(PinMap::TC1_Cs, OUTPUT);
    pinMode(PinMap::SD_EN, OUTPUT);

    pinMode(PinMap::BuckEN, OUTPUT);
    pinMode(PinMap::BuckPGOOD, INPUT);

    digitalWrite(PinMap::SdCs_0, HIGH);
    digitalWrite(PinMap::ADC0_Cs, HIGH);
    digitalWrite(PinMap::TC0_Cs, HIGH);
    digitalWrite(PinMap::TC1_Cs, HIGH);
    digitalWrite(PinMap::SD_EN, LOW);

    setupSPI();

    // Thermocouples:
    TC0.setup();
    TC1.setup();
    // ADC:
    ADC0.setup();
    ADC0.setOSR(ADS131M04::OSROPT::OSR8192);
    ADC0.setGain(0,ADS131M04::GAIN::GAIN1);
    ADC0.setGain(1,ADS131M04::GAIN::GAIN1);
    ADC0.setGain(2,ADS131M04::GAIN::GAIN1);

    serviceSetup();

    remoteSensorSetup();

    primarysd.setup();

    initializeLoggers();

    Heimdall.setup();
};

void System::systemUpdate()
{
    deviceUpdate();

    remoteSensorUpdate();

    logReadings();

    Heimdall.update();
};

void System::serviceSetup()
{
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::Heimdall), Heimdall.getThisNetworkCallback());
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::FB_PT), FB_PT.getThisNetworkCallback());
    // networkmanager.registerService(static_cast<uint8_t>(Services::ID::Middle_PT), PT1.getThisNetworkCallback());
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::N2_PT), N2_PT.getThisNetworkCallback());
}

void System::initializeLoggers()
{
    // check if sd card is mounted
    if (primarysd.getState() != StoreBase::STATE::NOMINAL)
    {
        loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(nullptr, networkmanager);

        return;
    }

    // open log files
    // get unique directory for logs
    std::string log_directory_path = primarysd.generateUniquePath(log_path, "");
    // make new directory
    primarysd.mkdir(log_directory_path);

    std::unique_ptr<WrappedFile> syslogfile = primarysd.open(log_directory_path + "/syslog.txt", static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END));
    std::unique_ptr<WrappedFile> telemetrylogfile = primarysd.open(log_directory_path + "/telemetrylog.csv", static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END),100);

    // intialize sys logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(std::move(syslogfile), networkmanager);

    // initialize telemetry logger
    std::string file_header = "cmd_angle(deg),fb_pt(bar),n2_pt(bar),tc0(C),tc1(C),time(us)";
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::TELEMETRY>().initialize(std::move(telemetrylogfile),file_header,[](std::string_view msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);});
}

void System::deviceUpdate()
{

    ADC0.update();

    TC0.update();
    TC1.update();
}

void System::remoteSensorUpdate()
{

    FB_PT.update(ADC0.getOutput(fb_pt_adc_ch));
    N2_PT.update(ADC0.getOutput(n2_pt_adc_ch));
}

void System::logReadings()
{
    if (esp_timer_get_time() - prev_telemetry_log_time > telemetry_log_delta)
    {
        TelemetryLogframe logframe;

        logframe.ch0sens = FB_PT.getPressure();
        // logframe.ch1sens = PT1.getPressure();
        logframe.ch2sens = N2_PT.getPressure();

        logframe.cmdAngle = Heimdall.getRegAngle();

        logframe.temp0 = TC0.getTemp();
        logframe.temp1 = TC1.getTemp();

        logframe.timestamp = esp_timer_get_time();
        prev_telemetry_log_time = esp_timer_get_time();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::TELEMETRY>(logframe);
    }

    if((primarysd.getError() > 0) && !systemstatus.flagSet(SYSTEM_FLAG::ERROR_SD)){
        systemstatus.newFlag(SYSTEM_FLAG::ERROR_SD, "SD Card Failed with error: " + std::to_string(primarysd.getError()));
    };
}

void System::setupSPI(){
    SDSPI.begin(PinMap::SD_SCLK,PinMap::SD_MISO,PinMap::SD_MOSI);
    SDSPI.setFrequency(20e6);
    SDSPI.setBitOrder(MSBFIRST);
    SDSPI.setDataMode(SPI_MODE0);

    SNSRSPI.begin(PinMap::SNSR_SCLK, PinMap::SNSR_MISO, PinMap::SNSR_MOSI);
    SNSRSPI.setFrequency(5e6);
    SNSRSPI.setBitOrder(MSBFIRST);
    SNSRSPI.setDataMode(SPI_MODE1);
}

void System::remoteSensorSetup(){
    FB_PT.setup();
    N2_PT.setup();
}