/*
 * VCU_IncomingOrders.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: stefancostea
 */

#pragma once

#include "Packets/Order.hpp"
#include "VCU.hpp"

namespace VCU{

	void hardware_reset(){
		HAL_NVIC_SystemReset();
	}

	void set_regulator_pressure();
	void brake();
	void unbrake();
	void disable_emergency_tape();
	void enable_emergency_tape();

	template<VCU_MODE> class IncomingOrders;

	template<>
	class IncomingOrders<BRAKE_VALIDATION>{
		StackOrder<0> hardware_reset_order;
		StackOrder<4,float> set_regulator_pressure_order;
		StackOrder<0> brake_order;
		StackOrder<0> unbrake_order;
		StackOrder<0> disable_emergency_tape_order;
		StackOrder<0> enable_emergency_tape_order;
	public:
		float new_pressure = 0;

		IncomingOrders(Data<BRAKE_VALIDATION>& data) :
			hardware_reset_order(209,hardware_reset),
			set_regulator_pressure_order(210, set_regulator_pressure, &new_pressure),
			brake_order(215, brake), unbrake_order(216, unbrake),
			disable_emergency_tape_order(217,disable_emergency_tape), enable_emergency_tape_order(218,enable_emergency_tape)
		{}
	};

	template<>
	class IncomingOrders<VEHICLE>{
		//TODO: Hacer todas las ordenes para el vehicle y hacer las states
		StackOrder<0> hardware_reset_order;
		StackOrder<4,float> set_regulator_pressure_order;
		StackOrder<0> brake_order;
		StackOrder<0> unbrake_order;
		StackOrder<0> disable_emergency_tape_order;
		StackOrder<0> enable_emergency_tape_order;

	public:
		float new_pressure = 0;

		IncomingOrders(Data<VEHICLE>& data) :
			hardware_reset_order(209, hardware_reset),
			set_regulator_pressure_order(210, set_regulator_pressure, &new_pressure),
			brake_order(215, brake), unbrake_order(216, unbrake),
			disable_emergency_tape_order(217, disable_emergency_tape), enable_emergency_tape_order(218,enable_emergency_tape)
		{}
	};
}
