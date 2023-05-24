#pragma once

#include "ST-LIB.hpp"
#include "VCU_Pinout/Pinout.hpp"

namespace VCU{
    
    class EnvironmentalSensors{
        constexpr static double temperature_sensor_slope = -0.000000413;
        constexpr static double temperature_sensor_offset = 193.902439024;

        constexpr static double pressure_sensor_slope = 0.000019091;
        constexpr static double pressure_sensor_offset = -0.125;
        
        private:
        Data<VCU::VCU_MODE::VEHICLE>& data;

        LinearSensor temperature_sensor;
        LinearSensor pressure_sensor;
       
        public:
        EnvironmentalSensors(Data<VCU::VCU_MODE::VEHICLE> &data):
            data(data),
            temperature_sensor(Pinout::ENVIRONMENT_TEMPERATURE, temperature_sensor_slope, temperature_sensor_offset, &data.enviroment_temperature),
            pressure_sensor(Pinout::ENVIRONMENT_PRESSURE, pressure_sensor_slope, pressure_sensor_offset , &data.enviremont_pressure)
        {}

        void read(){
            temperature_sensor.read();
            pressure_sensor.read();
        }
    };
}