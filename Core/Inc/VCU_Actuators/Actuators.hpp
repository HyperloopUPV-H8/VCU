/*
 * Actuators.hpp
 *
 *  Created on: Jun 14, 2023
 *      Author: stefancostea
 */

#pragma once
#include "VCU_Brakes/VCU_Brakes.hpp"
namespace VCU{
	template<VCU_MODE MODE> class Actuators;

	template<>
	class Actuators<BRAKE_VALIDATION>{
	public:
		Brakes<BRAKE_VALIDATION> brakes;
    	DigitalOutput led_sleep, led_flash, led_fault, led_operational, led_can;
    	Actuators(Data<BRAKE_VALIDATION>& data) : brakes(data), led_sleep(Pinout::SLEEP_LED), led_flash(Pinout::FLASH_LED), led_fault(Pinout::FAULT_LED), led_operational(Pinout::OPERATIONAL_LED), led_can(Pinout::CAN_LED) {}
	};
}
