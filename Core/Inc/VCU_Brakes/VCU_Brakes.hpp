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
        private:
            ValveActuator valve_actuator;
            RegulatorActuator regulator_actuator;
            RegulatorSensor regulator_sensor;
            Data<VCU::VCU_MODE::BRAKE_VALIDATION>& data;

            Reed reed1;
            Reed reed2;
            Reed reed3;
            Reed reed4;

        public:
            Brakes(Pin& valve_pin, Pin& regulator_output, Pin& regulator_input, 
                   Pin& reed1, Pin& reed2, Pin& reed3, Pin& reed4,
                   Data<VCU::VCU_MODE::BRAKE_VALIDATION>& data):

                valve_actuator(valve_pin, &data.valve_state),
                regulator_actuator(regulator_output, &data.regulator_reference_pressure),
                regulator_sensor(regulator_input, &data.regulator_real_pressure),
                reed1(reed1, &data.reed1),
                reed2(reed2, &data.reed2),
                reed3(reed3, &data.reed3),
                reed4(reed4, &data.reed4),
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
            }

            void brake(){

            }

            void not_brake(){

            }
    };
}