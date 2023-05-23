#pragma once

#include "ST-LIB.hpp"
#include "VCU_Actuators/VCU_ValveActuator.hpp"
#include "VCU_Actuators/VCU_RegulatorActuator.hpp"
#include "VCU_Sensors/VCU_RegulatorSensor.hpp"
#include "VCU_Sensors/VCU_Reed.hpp"
#include "VCU_Data/VCU_Data.hpp"


namespace VCU{
    template<VCU::VCU_MODE> class Brakes;

    template<>
    class Brakes<VCU::VCU_MODE::BRAKE_VALIDATION>{
        constexpr static double temperature_sensors_slope = 0.0f;
        constexpr static double temperature_sensor1_offset = 0.0f;
        constexpr static double temperature_sensor2_offset = 0.0f;

        constexpr static double high_pressure_sensor_slope = 0.0f;
        constexpr static double high_pressure_sensor_offset = 0.0f;

        constexpr static double low_pressure_sensors_slope = 0.0f;
        constexpr static double low_pressure_sensor1_offset = 0.0f;
        constexpr static double low_pressure_sensor2_offset = 0.0f;


        private:
            Data<VCU::VCU_MODE::BRAKE_VALIDATION>& data;

            ValveActuator valve_actuator;
            RegulatorActuator regulator_actuator;
            RegulatorSensor regulator_sensor;
            
            Reed reed1;
            Reed reed2;
            Reed reed3;
            Reed reed4;

            LinearSensor temperature_sensor1;
            LinearSensor temperature_sensor2;

            LinearSensor high_pressure_sensor1;
            LinearSensor low_pressure_sensor1;
            LinearSensor low_pressure_sensor2;

        public:
            Brakes(Data<VCU::VCU_MODE::BRAKE_VALIDATION>& data):

                valve_actuator(valve_pin, &data.valve_state),
                regulator_actuator(regulator_output, &data.regulator_reference_pressure),
                regulator_sensor(regulator_input, &data.regulator_real_pressure),
                reed1(reed1, &data.reed1),
                reed2(reed2, &data.reed2),
                reed3(reed3, &data.reed3),
                reed4(reed4, &data.reed4),
                temperature_sensor1(temperature1, 0.0f, 0.0f, &data.bottle_temperature1),
                temperature_sensor2(temperature2, 0.0f, 0.0f, &data.bottle_temperature2),
                high_pressure_sensor1(high_pressure1, 0.0f, 0.0f, &data.high_pressure1),
                low_pressure_sensor1(low_pressure1, 0.0f, 0.0f, &data.low_pressure1),
                low_pressure_sensor2(low_pressure2, 0.0f, 0.0f, &data.low_pressure2),
                data(data)
            {}

            void init(){
                //TODO:
                //Hacer una primera lectura de todos los valores
                //Poner regulador a 8 bares
                //Poner valvula en closed (estado 0)
                //Volver a leer todos los valores
            }

            void read(){
                //TODO:
                //Leer todo
                regulator_sensor.read();
                reed1.read();
                reed2.read();
                reed3.read();
                reed4.read();
            }

            void brake(){

            }

            void not_brake(){

            }
    };
}