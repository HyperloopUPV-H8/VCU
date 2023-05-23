#pragma once

#include "ST-LIB.hpp"

namespace VCU{
    template<VCU::VCU_MODE> class Data;

    template<>
    class Data<VCU::BRAKE_VALIDATION>{
        public:
        double regulator_real_pressure = 0.0f;
        double regulator_reference_pressure = 0.0f;

        bool reed1 = false;
        bool reed2 = false;
        bool reed3 = false;
        bool reed4 = false;
    };

    template<>
    class Data<VCU::VEHICLE>{
        public:
        double regulator_real_pressure = 0.0f;
        double regulator_reference_pressure = 0.0f;

        bool reed1 = false;
        bool reed2 = false;
        bool reed3 = false;
        bool reed4 = false;
    };
}