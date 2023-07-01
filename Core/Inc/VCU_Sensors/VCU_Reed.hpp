#pragma once

#include "ST-LIB.hpp"
#include "VCU_Utilities/VCU_Types.hpp"

namespace VCU{
    class Reed{
        private:
        DigitalSensor reed;
        REED_STATE* reed_state;
        
        public:
        Reed(Pin& pin, REED_STATE* reed_state): reed(pin, (PinState*)reed_state), reed_state(reed_state){}

        void read(){
            reed.read();
        }
    };
}
