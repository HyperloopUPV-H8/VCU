#pragma once

#include "ST-LIB.hpp"

namespace VCU{
    class RegulatorSensor{
        static constexpr double slope = 3;
        static constexpr double offset = 0;

        private:
        Pin& pin;
        LinearSensor sensor;
        double* pressure;

        public:
            RegulatorSensor(Pin& pin, double* pressure): pin(pin), sensor(pin, slope, offset, pressure), pressure(pressure){}

            void read(){
                sensor.read();  
            }            

            float get_pressure(){
                return *pressure;
            }
    };
}
