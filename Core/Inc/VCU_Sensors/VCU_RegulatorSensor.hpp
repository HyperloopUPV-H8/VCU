#pragma once

#include "ST-LIB.hpp"

namespace VCU{
    class RegulatorSensor{
        static constexpr double slope = 0.000190738;
        static constexpr double offset = -2.5;

        private:
        LinearSensor sensor;

        public:
            RegulatorSensor(Pin& pin, double* pressure): sensor(pin, slope, offset, pressure){}

            void read(){
                sensor.read();  
            }            
    };
}
