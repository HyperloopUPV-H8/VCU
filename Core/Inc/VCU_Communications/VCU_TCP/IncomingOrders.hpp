#pragma once
#include "Packets/Order.hpp"
#include "VCU.hpp"
namespace VCU{

	void hardware_reset(){
		HAL_NVIC_SystemReset();
	}

	void set_regulator_pressure();

	template<VCU_MODE> class IncomingOrders;

	template<>
	class IncomingOrders<BRAKE_VALIDATION>{
		StackOrder<0> hardware_reset_order;
		StackOrder<4,float> set_regulator_pressure_order;

	public:
		float new_pressure = 0;

		IncomingOrders(Data<BRAKE_VALIDATION>& data) : hardware_reset_order(209,hardware_reset),
		set_regulator_pressure_order(210, set_regulator_pressure, &new_pressure){}
	};
}
