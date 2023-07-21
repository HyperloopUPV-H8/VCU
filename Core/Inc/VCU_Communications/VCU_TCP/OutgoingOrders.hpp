/*
 * VCU_IncomingOrders.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: stefancostea
 */

#pragma once

#include "VCU.hpp"

namespace VCU{
	enum class LevitationOrders: uint16_t{
		TAKE_OFF = 300,
		LANDING = 301,
		RESET_ALL = 326

	};

	enum class TractionOrders: uint16_t{
		MOVE = 612,
		BRAKE = 613,
		TURN_OFF = 614,
		PCU_RESET = 602,
	};

	enum class BatteryOrders: uint16_t{
		CLOSE_CONTACTORS = 903,
		OPEN_CONTACTORS = 902,
		OBCCU_RESET = 906,
		BMSL_RESET = 804,
	};

	template<VCU_MODE> class OutgoingOrders;

	template<>
	class OutgoingOrders<VEHICLE>{
	public:
		StackOrder<0> lcu_reset_all;
		StackOrder<0> pcu_reset;
		StackOrder<0> bmsl_reset;
		StackOrder<0> obccu_reset;

		OutgoingOrders<VEHICLE>()
				:
				lcu_reset_all((uint16_t)LevitationOrders::RESET_ALL),
				pcu_reset((uint16_t)TractionOrders::PCU_RESET),
				bmsl_reset((uint16_t)BatteryOrders::BMSL_RESET),
				obccu_reset((uint16_t)BatteryOrders::OBCCU_RESET)
		{
		}

	};

}
