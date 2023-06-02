#pragma once

#include "ST-LIB.hpp"
#include "VCU_Pinout/Pinout.hpp"

namespace VCU{
    
    class EnvironmentalSensors{
        constexpr static double temperature_sensor_slope = -121.9512195;
        constexpr static double temperature_sensor_offset = 196.3414634;

        constexpr static double pressure_sensor_slope = 0.379129905;
        constexpr static double pressure_sensor_offset = -0.125 - 0.03; //-0.03 Ajuste manual
        
        private:
        Data<VCU::VCU_MODE::VEHICLE>& data;

        LinearSensor temperature_sensor;
        LinearSensor pressure_sensor;
       
        public:
        EnvironmentalSensors(Data<VCU::VCU_MODE::VEHICLE> &data):
            data(data),
            temperature_sensor(Pinout::ENVIRONMENT_TEMPERATURE, temperature_sensor_slope, temperature_sensor_offset, &data.environment_temperature),
            pressure_sensor(Pinout::ENVIRONMENT_PRESSURE, pressure_sensor_slope, pressure_sensor_offset , &data.environment_pressure)
        {}

        void read(){
            temperature_sensor.read();
            pressure_sensor.read();
        }
    };
}
