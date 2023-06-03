#pragma once

#include "ST-LIB.hpp"
#include "VCU_Pinout/Pinout.hpp"
#include "VCU_Data/VCU_Data.hpp"

namespace VCU{
    
    class EnvironmentalSensors{
        constexpr static float temperature_sensor_slope = -121.9512195;
        constexpr static float temperature_sensor_offset = 196.3414634;

        constexpr static float pressure_sensor_slope = 0.379129905;
        constexpr static float pressure_sensor_offset = -0.125 - 0.03; //-0.03 Ajuste manual
        
        private:
        Data<VCU::VCU_MODE::VEHICLE>& data;

        LinearSensor<float> temperature_sensor;
        LinearSensor<float> pressure_sensor;
       
        public:
        EnvironmentalSensors(Data<VCU::VCU_MODE::VEHICLE> &data):
            data(data),
            temperature_sensor(Pinout::ENVIRONMENT_TEMPERATURE, temperature_sensor_slope, temperature_sensor_offset, &data.enviroment_temperature),
            pressure_sensor(Pinout::ENVIRONMENT_PRESSURE, pressure_sensor_slope, pressure_sensor_offset , &data.enviroment_temperature)
        {}

        void read(){
            temperature_sensor.read();
            pressure_sensor.read();
        }
    };
}
