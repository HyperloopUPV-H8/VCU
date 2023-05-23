#pragma once
#include "PinModel/Pin.hpp"
namespace VCU{
    namespace Pinout{
        //VEHICLE LEDs
        constexpr Pin& LEDR = PF1;
        constexpr Pin& LEDG = PF2;
        constexpr Pin& LEDB = PF3;

        //Temperatures
        constexpr Pin& BOTTLE_TEMP1 = PF6;
        constexpr Pin& BOTTLE_TEMP2 = PF7;
        constexpr Pin& BOTTLE_TEMP3 = PF8;
        constexpr Pin& BOTTLE_TEMP4 = PF9;

        //Environment
        constexpr Pin& ENVIRONMENT_TEMPERATURE = PC2;
        constexpr Pin& ENVIRONMENT_PRESSURE = PA0;

        //Regulator
        constexpr Pin& REGULATOR_IN = PA4;
        constexpr Pin& REGULATOR_OUT = PB9;

        //Valve 
        constexpr Pin& VALVE = PE7;

        //Pressure
        constexpr Pin& HIGH_PRESSURE = PA1;
        constexpr Pin& LOW_PRESSURE1 = PA2;
        constexpr Pin& LOW_PRESSURE2 = PA3;

        //Tapes
        constexpr Pin& TAPE1 = PC6;
        constexpr Pin& TAPE2 = PC7;
        constexpr Pin& EMERGENCY_TAPE  = PE0;
        constexpr Pin& EMERGENCY_TAPE_ENABLE = PG1;

        //IMU
        constexpr Pin& IMU_INTERRUPT = PD2;
        constexpr Pin& IMU_CHIP_SELECT = PD3;

        //Board LEDs
        constexpr Pin& SLEEP_LED = PG4;
        constexpr Pin& FLASH_LED = PG5;
        constexpr Pin& CAN_LED = PG6;
        constexpr Pin& FAULT_LED = PG7;
        constexpr Pin& OPERATIONAL_LED = PG8;

        //Reeds
        constexpr Pin& REED1 = PD11;
        constexpr Pin& REED2 = PD10;
        constexpr Pin& REED3 = PE3;
        constexpr Pin& REED4 = PE4;
    }
}