/*
**********************
* PINS               *
**********************
 */
#pragma once
#include <stdint.h>

namespace PinMap{
    
    static constexpr uint8_t ADC_DRDY = 1;
    static constexpr uint8_t ADC_CS   = 2;
    static constexpr uint8_t HSPI_MOSI = 4;
    static constexpr uint8_t HSPI_MISO = 5;
    static constexpr uint8_t HSPI_SCLK = 6;
    static constexpr uint8_t T_CS_0 = 7;
    static constexpr uint8_t T_FAULT_0 = 8;
    static constexpr uint8_t T_CS_1 = 9;
    static constexpr uint8_t T_FAULT_1 = 10;
    static constexpr uint8_t T_DRDY_1 = 11;
    static constexpr uint8_t T_DRDY_0 = 12;
    static constexpr uint8_t GPIO_1 = 13;
    static constexpr uint8_t GPIO_5 = 14;
    static constexpr uint8_t GPIO_6 = 15;
    static constexpr uint8_t GPIO_3 = 16;
    static constexpr uint8_t GPIO_7 = 17;
    static constexpr uint8_t GPIO_4 = 18;
    static constexpr uint8_t BUCK_EN = 21;

    static constexpr uint8_t SD_DET = 33;
    static constexpr uint8_t SD_CS = 34;
    static constexpr uint8_t VSPI_MOSI = 35;
    static constexpr uint8_t VSPI_SCLK = 36;
    static constexpr uint8_t VSPI_MISO = 37;
    static constexpr uint8_t SD_EN = 38;

    static constexpr uint8_t ADC_CLK = 45;
    static constexpr uint8_t SERVO_PWM = 47;
    static constexpr uint8_t BUCK_PGOOD = 48;




    static constexpr uint8_t TX_CAN = 42 ;
    static constexpr uint8_t RX_CAN = 41;

};
