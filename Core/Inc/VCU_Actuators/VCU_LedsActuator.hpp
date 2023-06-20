#pragma once 

#include "ST-LIB.hpp"

namespace VCU{
	class Color{
	public:
		uint8_t red, green, blue;
		Color(uint8_t red, uint8_t green, uint8_t blue) : red(red), green(green), blue(blue){}
		static Color RED, BLUE, GREEN, MAGENTA, PURPLE, ORANGE, TURQUOISE, PISTACCIO_GREEN, WHITE;
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

        void set_color(Color& color){
        	set_color(color.red,color.green, color.blue);
        }

        void turn_off(){
            this->red.set_duty_cycle(0.0f);
            this->green.set_duty_cycle(0.0f);
            this->blue.set_duty_cycle(0.0f);
        }
    };

    Color Color::RED(255, 0,0);
    Color Color::BLUE(0,0,255);
    Color Color::GREEN(0,255,0);
    Color Color::MAGENTA(255,0,255);
    Color Color::PURPLE(160,32,240);
    Color Color::ORANGE(255,165,0);
    Color Color::TURQUOISE(93,193,185);
    Color Color::PISTACCIO_GREEN(147,197,114);
    Color Color::WHITE(255,255,255);
}
