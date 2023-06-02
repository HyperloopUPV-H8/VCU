#pragma once

#include "ST-LIB.hpp"
#include "VCU_Utilities/VCU_Types.hpp"


namespace VCU{

    template<VCU::VCU_MODE> class Data;

    template<>
    class Data<VCU::BRAKE_VALIDATION>{
        public:
        double regulator_real_pressure = 0.0f;
        float regulator_reference_pressure = 0.0f;

        PinState emergeny_tape_enable = PinState::OFF;
        PinState emergency_tape = PinState::OFF;

        double high_pressure1 = 0.0f;
        double low_pressure1 = 0.0f;
        double low_pressure2 = 0.0f;

        double bottle_temperature1 = 0.0f;
        double bottle_temperature2 = 0.0f;

        REED_STATE reed1 = REED_STATE::RETRACTED;
        REED_STATE reed2 = REED_STATE::RETRACTED;
        REED_STATE reed3 = REED_STATE::RETRACTED;
        REED_STATE reed4 = REED_STATE::RETRACTED;
        bool reeds_ok = true;

        VALVE_STATE valve_state = VALVE_STATE::CLOSED;
    };

    template<>
    class Data<VCU::VEHICLE>{
        public:
        double regulator_real_pressure = 0.0f;
        float regulator_reference_pressure = 0.0f;

        PinState emergeny_tape_enable = PinState::OFF;
        PinState emergency_tape = PinState::OFF;

        double high_pressure1 = 0.0f;
        double low_pressure1 = 0.0f;
        double low_pressure2 = 0.0f;

        double bottle_temperature1 = 0.0f;
        double bottle_temperature2 = 0.0f;

        REED_STATE reed1 = REED_STATE::RETRACTED;
        REED_STATE reed2 = REED_STATE::RETRACTED;
        REED_STATE reed3 = REED_STATE::RETRACTED;
        REED_STATE reed4 = REED_STATE::RETRACTED;
        bool reeds_ok = true;

        VALVE_STATE valve_state = VALVE_STATE::CLOSED;

        double enviroment_temperature = 0.0f;
        double enviremont_pressure = 0.0f;
    };
}