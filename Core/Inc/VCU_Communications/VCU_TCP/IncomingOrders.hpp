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
		gui_keepalive = 69,
		hardware_reset_order = 209,
		set_regulator_pressure_order = 210,
		brake_order = 215,
		unbrake_order = 216,
		disable_emergency_tape_order = 217,
		enable_emergency_tape_order = 218,

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

		vehicle_reset = 250,

		start_levitating = 300,
		stop_levitating = 301,

		stop_traction = 609,

		open_contactors = 902,
		close_contactors = 903

	};


	void hardware_reset(){
		HAL_NVIC_SystemReset();
	}

	void set_regulator_pressure();
	void brake();
	void unbrake();
	void disable_emergency_tape();
	void enable_emergency_tape();

	void close_contactors();
	void open_contactors();
	void start_vertical_levitation();
	void start_lateral_levitation();
	void start_traction();
	void stop_levitation();
	void stop_traction();
	void emergency_stop();

	void vehicle_reset();

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
	public:

		enum TractionDirection{
			FORWARD,
			BACKWARD
		};
		StackOrder<0> gui_keepalive_order;
		StackOrder<0> hardware_reset_order;
		StackOrder<4,float> set_regulator_pressure_order;
		StackOrder<0> brake_order;
		StackOrder<0> unbrake_order;
		StackOrder<0> disable_emergency_tape_order;
		StackOrder<0> enable_emergency_tape_order;
		StackOrder<0> close_contactors_order;
		StackOrder<0> open_contactors_order;
		StackOrder<0> start_vertical_levitation_order;
		StackOrder<0> start_lateral_levitation_order;
		StackOrder<15,float,float,float, TractionDirection,bool, bool> start_traction_order;
		StackOrder<0> stop_traction_order;
		StackOrder<0> stop_levitation_order;
		StackOrder<0> emergency_stop_order;

		StackOrder<0> vehicle_reset_order;

	public:
		float new_pressure = 0;
		float traction_frequency = 0;
		float taget_traction_voltage = 0;
		float traction_vdc = 0;
		TractionDirection traction_direction = TractionDirection::FORWARD;
		bool dummy = false;

		IncomingOrders(Data<VEHICLE>& data) :
			gui_keepalive_order((uint16_t)IncomingOrdersIDs::hardware_reset_order, nullptr),
			hardware_reset_order((uint16_t)IncomingOrdersIDs::hardware_reset_order, hardware_reset),
			set_regulator_pressure_order((uint16_t)IncomingOrdersIDs::set_regulator_pressure_order, set_regulator_pressure, &new_pressure),
			brake_order((uint16_t)IncomingOrdersIDs::brake_order, brake),
			unbrake_order((uint16_t)IncomingOrdersIDs::unbrake_order, unbrake),
			disable_emergency_tape_order((uint16_t)IncomingOrdersIDs::disable_emergency_tape_order, disable_emergency_tape),
			enable_emergency_tape_order((uint16_t)IncomingOrdersIDs::enable_emergency_tape_order, enable_emergency_tape),
			close_contactors_order(903,close_contactors), open_contactors_order(902, open_contactors ),
			start_vertical_levitation_order(300, start_vertical_levitation),
			start_lateral_levitation_order(336, start_lateral_levitation),
			start_traction_order(315,start_traction, &traction_frequency, &traction_vdc, &taget_traction_voltage, &traction_direction,&dummy, &dummy),
			stop_traction_order(609, stop_traction),
			stop_levitation_order(316, stop_levitation),
			emergency_stop_order(200, emergency_stop),
			vehicle_reset_order((uint16_t)IncomingOrdersIDs::vehicle_reset, vehicle_reset)
		{
		}
	};
}
