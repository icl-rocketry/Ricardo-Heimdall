/*
**********************
* PINS               *
**********************
 */
#pragma once

namespace PinMap{

    static constexpr int SNSR_MISO = 5;
    static constexpr int SNSR_MOSI = 4;
    static constexpr int SNSR_SCLK = 6;

    static constexpr int TC0_Cs = 7;
    static constexpr int TC1_Cs = 8;

    static constexpr int ADC0_Cs = 2;
    static constexpr int ADC_CLK = 45;

    static constexpr int SD_SCLK = 36;
    static constexpr int SD_MISO = 37;
    static constexpr int SD_MOSI = 35;
    static constexpr int SdDet_0 = 33;
    static constexpr int SdCs_0 = 34;
    static constexpr int SD_EN = 38;

    static constexpr int TxCan = 42;
    static constexpr int RxCan = 41;

    static constexpr int BuckEN = 21;
    static constexpr int BuckPGOOD = 48;
    static constexpr int ServoPWM = 47;

    static constexpr int BuckSense = 13;
};
