/*
 * VCU_RegulatorActuator.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: Pablo
 */


#pragma once 

#include "ST-LIB.hpp"

namespace VCU{
    
    class RegulatorActuator{
        static constexpr uint16_t pwm_frequency = 10000;
        static constexpr float max_pressure = 10.0f;
        static constexpr float min_pressure = 0.0f;
        
        private:
        PWM pwm;
        float& pressure;

        public:
            RegulatorActuator(Pin& pin, float& pressure): pwm(pin), pressure(pressure){
            }

            void init(){
                pwm.set_frequency(pwm_frequency);
                pwm.set_duty_cycle(0.0f);
                pwm.turn_on();
            }

            void set_pressure(float value){
                if(value < min_pressure || value > max_pressure){
                    ErrorHandler("Pressure value %f out of range, expected value between 0.0 and 10.0", pressure);
                    return;
                }

                pressure = value;
                pwm.set_duty_cycle(calculate_duty_cycle(pressure));
            }

            float get_pressure(){
                return pressure;
            }

        private:
            float static calculate_duty_cycle(float pressure){
                return ((1.6f * pressure + 4.0f) * ( 0.168f / 3.3f)) * 100.0f;
            }
    };
}
