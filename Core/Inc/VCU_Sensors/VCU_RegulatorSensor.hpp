#pragma once

#include "ST-LIB.hpp"

namespace VCU{
    class RegulatorSensor{
        static constexpr double slope = 3.0;
        static constexpr double offset = 0.0;

        private:
        LinearSensor sensor;

        public:
            RegulatorSensor(Pin& pin, double* pressure): sensor(pin, slope, offset, pressure){}

            void read(){
                sensor.read();  
            }            
    };
}
