#pragma once

#include "ST-LIB.hpp"
#include "VCU_Utilities/VCU_Types.hpp"
#include "VCU_Mode/VCU_Mode.hpp"


namespace VCU{

    template<VCU::VCU_MODE> class Data;

    template<>
    class Data<VCU::VCU_MODE::BRAKE_VALIDATION>{
        public:
    	float regulator_real_pressure = 0.0f;
        float regulator_reference_pressure = 0.0f;

        float high_pressure1 = 0.0f;
        float low_pressure1 = 0.0f;
        float low_pressure2 = 0.0f;

        double bottle_temperature1 = 0.0f;
        double bottle_temperature2 = 0.0f;

        REED_STATE reed = REED_STATE::RETRACTED;

        VALVE_STATE valve_state;
    };

    template<>
    class Data<VCU::VCU_MODE::VEHICLE>{
        public:
    	float regulator_real_pressure = 0.0f;
        float regulator_reference_pressure = 0.0f;

        PinState emergeny_tape_enable = PinState::OFF;
        PinState emergency_tape = PinState::OFF;

        float high_pressure1 = 0.0f;
        float low_pressure1 = 0.0f;
        float low_pressure2 = 0.0f;

        double bottle_temperature1 = 0.0f;
        double bottle_temperature2 = 0.0f;

        REED_STATE reed1 = REED_STATE::RETRACTED;
        REED_STATE reed2 = REED_STATE::RETRACTED;
        REED_STATE reed3 = REED_STATE::RETRACTED;
        REED_STATE reed4 = REED_STATE::RETRACTED;
        bool reeds_ok = true;

        VALVE_STATE valve_state = VALVE_STATE::CLOSED;
        float enviroment_temperature = 0.0f;
        float enviremont_pressure = 0.0f;

        double tapes_position = 0.0f;
        double tapes_direction = 0.0f;
        double tapes_speed = 0.0f;
        double tapes_acceleration = 0.0f;
    };
}
