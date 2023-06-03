#pragma once

#include "ST-LIB.hpp"

namespace VCU{
    class RegulatorSensor{
        static constexpr float slope = 0.000190738;
        static constexpr float offset = -2.5;

        private:
        LinearSensor<float> sensor;

        public:
            RegulatorSensor(Pin& pin, float& pressure): sensor(pin, slope, offset, &pressure){}

            void read(){
                sensor.read();  
            }            
    };
}
