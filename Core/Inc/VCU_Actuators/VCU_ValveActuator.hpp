#pragma once 

#include "ST-LIB.hpp"
#include "VCU_Utilities/VCU_Types.hpp"

namespace VCU{

    class ValveActuator{
        private:
        DigitalOutput digital_output;
        VALVE_STATE* valve_state;

        public:
            ValveActuator(Pin& pin, VALVE_STATE* valve_state): digital_output(pin), valve_state(valve_state){
                digital_output.turn_off();
            }

            void close(){
                digital_output.turn_off();
                *valve_state = VALVE_STATE::CLOSED;
            }

            void open(){
                digital_output.turn_on();
                *valve_state = VALVE_STATE::OPEN;
            }    
    };
}