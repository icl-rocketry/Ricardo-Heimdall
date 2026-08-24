#include "system.h"


System::System():
    RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
    _canbus(systemstatus,PinMap::TX_CAN,PinMap::RX_CAN,3),
    _sd_spi(FSPI), // VSPI
    _sensor_spi(HSPI),
    _adc0(_sensor_spi, PinMap::ADC_CS, PinMap::ADC_CLK),
    _pt0(networkmanager, 0),
    _pt1(networkmanager, 1),
    _pt2(networkmanager, 2),
    _max0(_sensor_spi, PinMap::T_CS_0),
    _max1(_sensor_spi, PinMap::T_CS_1),
    _tc0(_max0, networkmanager, "TC0"),
    _tc1(_max1, networkmanager, "TC1"),
    _regulator(networkmanager,PinMap::SERVO_PWM, _pt0, _pt1, _pt2, _tc0, _tc1),
    _primarysd(_sd_spi, PinMap::SD_CS,SD_SCK_MHZ(20),false,&systemstatus)
    {};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    
    _canbus.setup(); 
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST, {1, 3});
    networkmanager.addInterface(&_canbus);

    // TODO: these should be done in the drivers not here
    //ADC
    pinMode(PinMap::ADC_DRDY, INPUT);

    pinMode(PinMap::T_CS_0, OUTPUT);
    pinMode(PinMap::T_CS_1, OUTPUT);
    digitalWrite(PinMap::T_CS_0, HIGH);
    digitalWrite(PinMap::T_CS_1, HIGH);


    // to be integrated into the driver
    pinMode(PinMap::T_FAULT_0, INPUT);
    pinMode(PinMap::T_FAULT_1, INPUT);
    pinMode(PinMap::T_DRDY_0, INPUT);
    pinMode(PinMap::T_DRDY_1, INPUT);

    //SD card
    pinMode(PinMap::SD_DET, INPUT);
    pinMode(PinMap::SD_EN, OUTPUT);

    //Buck converter
    pinMode(PinMap::BUCK_EN, OUTPUT);
    pinMode(PinMap::BUCK_PGOOD, INPUT);

    setupSPI();

    _primarysd.setup();

    //needs the store mounted so it can open the log files
    initializeLoggers();

    _adc0.setup();
    _adc0.setOSR(ADS131M04::OSROPT::OSR8192);
    _adc0.setGain(0,ADS131M04::GAIN::GAIN1);
    _adc0.setGain(1,ADS131M04::GAIN::GAIN1);
    _adc0.setGain(2,ADS131M04::GAIN::GAIN1);

    _pt0.setup();
    _pt1.setup();
    _pt2.setup();

    _tc0.setup();
    _tc1.setup();

    uint8_t ptservice0 = static_cast<uint8_t>(Services::ID::PT0);
    uint8_t ptservice1 = static_cast<uint8_t>(Services::ID::PT1);
    uint8_t ptservice2 = static_cast<uint8_t>(Services::ID::PT2);

    uint8_t tcservice0 = static_cast<uint8_t>(Services::ID::TC0);
    uint8_t tcservice1 = static_cast<uint8_t>(Services::ID::TC1);
    

    networkmanager.registerService(ptservice0,_pt0.getThisNetworkCallback());
    networkmanager.registerService(ptservice1,_pt1.getThisNetworkCallback());
    networkmanager.registerService(ptservice2,_pt2.getThisNetworkCallback());
    networkmanager.registerService(tcservice0,_tc0.getThisNetworkCallback());
    networkmanager.registerService(tcservice1,_tc1.getThisNetworkCallback());

};

void System::systemUpdate(){

    _adc0.update();
    
    _pt0.update(_adc0.getOutput(0));
    _pt1.update(_adc0.getOutput(1));
    _pt2.update(_adc0.getOutput(2));

    _tc0.update();
    _tc1.update();
}

void System::setupSPI(){
    _sd_spi.begin(PinMap::VSPI_SCLK,PinMap::VSPI_MISO,PinMap::VSPI_MOSI);
    _sd_spi.setFrequency(20e6);
    _sd_spi.setBitOrder(MSBFIRST);
    _sd_spi.setDataMode(SPI_MODE0);

    _sensor_spi.begin(PinMap::HSPI_SCLK, PinMap::HSPI_MISO, PinMap::HSPI_MOSI);
    _sensor_spi.setFrequency(5000000);
    _sensor_spi.setBitOrder(MSBFIRST);
    _sensor_spi.setDataMode(SPI_MODE1);
}

void System::initializeLoggers()
{
    // check if sd card is mounted
    if (_primarysd.getState() != StoreBase::STATE::NOMINAL)
    {
        loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(nullptr, networkmanager);

        return;
    }

    // open log files
    // get unique directory for logs
    std::string log_directory_path = _primarysd.generateUniquePath(_log_path, "");
    // make new directory
    _primarysd.mkdir(log_directory_path);

    std::unique_ptr<WrappedFile> syslogfile = _primarysd.open(log_directory_path + "/syslog.txt", static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END));
    std::unique_ptr<WrappedFile> telemetrylogfile = _primarysd.open(log_directory_path + "/telemetrylog.csv", static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END),100);

    // intialize sys logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(std::move(syslogfile), networkmanager);

    // initialize telemetry logger
    std::string file_header = "ch0sens,ch1sens,ch2sens,tc0(C),tc1(C),time(us)";
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::TELEMETRY>().initialize(std::move(telemetrylogfile),file_header,[](std::string_view msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);});
}

void System::logReadings()
{
    if (esp_timer_get_time() - _prev_telemetry_log_time > _telemetry_log_delta)
    {
        TelemetryLogframe logframe;

        logframe.ch0sens = _pt0.getProcessed();
        logframe.ch1sens = _pt0.getProcessed();
        logframe.ch2sens = _pt0.getProcessed();

        logframe.temp0 = _tc0.getProcessed();
        logframe.temp1 = _tc0.getProcessed();

        logframe.timestamp = esp_timer_get_time();
        _prev_telemetry_log_time = esp_timer_get_time();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::TELEMETRY>(logframe);

    }
}