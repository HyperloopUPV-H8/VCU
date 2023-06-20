#pragma once
#include "Packets/Order.hpp"
#include "VCU.hpp"
namespace VCU{

	void hardware_reset(){
		HAL_NVIC_SystemReset();
	}

	void set_regulator_pressure_brakes_validation();

	void brake_brakes_validation();

	void unbrake_brakes_validation();

	void disable_emergency_tape_brakes_validation();

	void enable_emergency_tape_brakes_validation();

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

		IncomingOrders(Data<BRAKE_VALIDATION>& data) : hardware_reset_order(209,hardware_reset),
		set_regulator_pressure_order(210, set_regulator_pressure_brakes_validation, &new_pressure),
		brake_order(215, brake_brakes_validation), unbrake_order(216, unbrake_brakes_validation),
		disable_emergency_tape_order(217,disable_emergency_tape_brakes_validation), enable_emergency_tape_order(218,enable_emergency_tape_brakes_validation){}
	};

	template<>
	class IncomingOrders<VEHICLE>{
		StackOrder<0> emergency_state;
	};
}
