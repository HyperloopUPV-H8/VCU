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

	enum class IncomingOrdersIDs: uint16_t{
		hardware_reset_order = 209,
		set_regulator_pressure_order = 210,
		brake_order = 216,
		unbrake_order = 217,
		disable_emergency_tape_order = 218,
		enable_emergency_tape_order = 219,

		heakthcheck_and_load = 220,
		healthcheck_and_unload = 221,
		start_static_lev_demostration = 222,
		start_dynamic_lev_demostration = 223,
		start_traction_demostration = 224,
		stop_demostration = 225,

		take_off = 226,
		landing = 227,
		start_crawling = 231,
		traction_data = 233,
		traction_end_data = 234,
	};


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
		StackOrder<0> hardware_reset_order;
		StackOrder<4,float> set_regulator_pressure_order;
		StackOrder<0> brake_order;
		StackOrder<0> unbrake_order;
		StackOrder<0> disable_emergency_tape_order;
		StackOrder<0> enable_emergency_tape_order;

	public:
		float new_pressure = 0;

		IncomingOrders(Data<VEHICLE>& data) :
			hardware_reset_order((uint16_t)IncomingOrdersIDs::hardware_reset_order, hardware_reset),
			set_regulator_pressure_order((uint16_t)IncomingOrdersIDs::set_regulator_pressure_order, set_regulator_pressure, &new_pressure),
			brake_order((uint16_t)IncomingOrdersIDs::brake_order, brake),
			unbrake_order((uint16_t)IncomingOrdersIDs::unbrake_order, unbrake),
			disable_emergency_tape_order((uint16_t)IncomingOrdersIDs::disable_emergency_tape_order, disable_emergency_tape),
			enable_emergency_tape_order((uint16_t)IncomingOrdersIDs::enable_emergency_tape_order,enable_emergency_tape)
		{}
	};
}
