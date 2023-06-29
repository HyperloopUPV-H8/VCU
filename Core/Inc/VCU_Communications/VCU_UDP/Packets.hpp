/*
 * VCU_Packets.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: Pablo & stefancostea
 */

#pragma once
#include "VCU.hpp"

namespace VCU{

	template<VCU_MODE> class Packets;

	template<>
	class Packets<VCU_MODE::BRAKE_VALIDATION>{
	public:
		StackPacket<9,VALVE_STATE, float, float> regulator_packet;
		StackPacket<1,REED_STATE> reed_packet;
		StackPacket<16,double,double> bottle_temperature_packet;
		StackPacket<12,float,float,float> pressure_packets;

		Packets(Data<BRAKE_VALIDATION>& data) :
				regulator_packet(211, &data.valve_state, &data.regulator_reference_pressure, &data.regulator_real_pressure),
				reed_packet(212, &data.reed),
				bottle_temperature_packet(213, &data.bottle_temperature1, &data.bottle_temperature2),
				pressure_packets(214, &data.high_pressure1, &data.low_pressure1, &data.low_pressure2)
		{}
	};

	template<>
	class Packets<VCU_MODE::VEHICLE>{
	public:
		enum class PacketsIDs: uint16_t{
			vcu_regulator_packet = 211,
			vcu_bottle_temperature_packet = 213,
			vcu_pressure_packet = 214,
			vcu_environmental_packet = 215,
			states_packet = 232,

			lcu_master_state_machine_packet = 306,

			accelerations = 606,
		};

		uint8_t trash_u8 = 0;
		float trash_f = 0.0f;

		//INFO: Outgoing packets
		StackPacket<9,VALVE_STATE, float, float> regulator_packet;
		StackPacket<16,double,double> bottle_temperature_packet;
		StackPacket<12,float,float,float> pressure_packets;
		StackPacket<8, float,float> environmental_packet;
		StackPacket<7, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t> states_packet;

		//INFO: Incoming packets0
		StackPacket<4, uint8_t, uint8_t, LevitaionState, uint8_t> lcu_states_packet;
		//TODO: Un packete con el contactors state
		StackPacket<20, float, float, float, float, float> accelerations;

		Packets(Data<VEHICLE>& data) :
				regulator_packet((uint16_t)PacketsIDs::vcu_regulator_packet, &data.valve_state, &data.regulator_reference_pressure, &data.regulator_real_pressure),
				bottle_temperature_packet((uint16_t)PacketsIDs::vcu_bottle_temperature_packet, &data.bottle_temperature1, &data.bottle_temperature2),
				pressure_packets((uint16_t)PacketsIDs::vcu_pressure_packet, &data.high_pressure1, &data.low_pressure1, &data.low_pressure2),
				environmental_packet((uint16_t)PacketsIDs::vcu_environmental_packet, &data.enviremont_pressure, &data.enviroment_temperature),
				states_packet((uint16_t)PacketsIDs::states_packet, data.general_state, data.specific_state, data.load_state, data.unload_state, data.traction_state, data.dynamic_lev, data.specific_state),
				lcu_states_packet((uint16_t)PacketsIDs::lcu_master_state_machine_packet, &trash_u8, &trash_u8, &data.levitation_state, &trash_u8),
				accelerations((uint16_t)PacketsIDs::accelerations, &trash_f, &trash_f, &trash_f, &data.engine_acceleration, &data.engine_speed)
		{}
	};
}
