/*
 * VCU_LedsActuator.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: Pablo & stefancostea
 */

#pragma once 

#include "ST-LIB.hpp"

namespace VCU{
	class RGBColor{
	public:
		uint8_t red, green, blue;
		RGBColor(uint8_t red, uint8_t green, uint8_t blue) : red(red), green(green), blue(blue){}
		static RGBColor RED, BLUE, GREEN, MAGENTA, PURPLE, ORANGE, TURQUOISE, PISTACCIO_GREEN, WHITE;
	};

    class LEDSActuator{
        private:
        PWM red;
        PWM green;
        PWM blue;

        public:
        LEDSActuator(Pin& red_pin, Pin& green_pin, Pin& blue_pin): red(red_pin), green(green_pin), blue(blue_pin){
            red.set_frequency(1000);
            green.set_frequency(1000);
            blue.set_frequency(1000);

            red.set_duty_cycle(0.0f);
            green.set_duty_cycle(0.0f);
            blue.set_duty_cycle(0.0f);

            red.turn_on();
            green.turn_on();
            blue.turn_on();
        }

        void set_color(uint8_t red, uint8_t green, uint8_t blue){
            if (red > 255 || green > 255 || blue > 255)
            {
            	ErrorHandler("Invalid color RGB code: %d , %d , %d", red, green, blue);
                return;
            }
            
            this->red.set_duty_cycle(red / 255.0f * 100.0f);
            this->green.set_duty_cycle(green / 255.0f * 100.0f);
            this->blue.set_duty_cycle(blue / 255.0f * 100.0f);
        }

        void set_color(RGBColor& color){
			set_color(color.red,color.green, color.blue);
		}

        void turn_off(){
            this->red.set_duty_cycle(0.0f);
            this->green.set_duty_cycle(0.0f);
            this->blue.set_duty_cycle(0.0f);
        }
    };

    RGBColor RGBColor::RED(255, 0,0);
    RGBColor RGBColor::BLUE(0,0,255);
    RGBColor RGBColor::GREEN(0,255,0);
	RGBColor RGBColor::MAGENTA(255,0,255);
	RGBColor RGBColor::PURPLE(160,32,240);
	RGBColor RGBColor::ORANGE(255,165,0);
	RGBColor RGBColor::TURQUOISE(93,193,185);
	RGBColor RGBColor::PISTACCIO_GREEN(147,197,114);
	RGBColor RGBColor::WHITE(255,255,255);

}
