/*
 * VCU_IncomingOrders.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: stefancostea
 */

#pragma once

#include "VCU.hpp"

namespace VCU{
	enum class LevitationOrdes: uint16_t{
		TAKE_OFF = 300,
		LANDING = 301,
	};

	enum class TractionOrders: uint16_t{

	};

	enum class BatteryOrders: uint16_t{
		CLOSE_CONTACTORS = 903,
		OPEN_CONTACTORS = 902,
	};

	template<VCU_MODE> class OutgoingOrders;

	template<>
	class OutgoingOrders<VEHICLE>{
	public:

		StackOrder<0> take_off_order;
		StackOrder<0> landing_order;

		StackOrder<0> close_contactors;
		StackOrder<0> open_contactors;

		OutgoingOrders() :
			take_off_order((uint16_t)LevitationOrdes::TAKE_OFF),
			landing_order((uint16_t)LevitationOrdes::LANDING),
			close_contactors((uint16_t)BatteryOrders::CLOSE_CONTACTORS),
			open_contactors((uint16_t)BatteryOrders::OPEN_CONTACTORS)
		{}
	};

}
