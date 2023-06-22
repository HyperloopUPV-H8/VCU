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
		//TODO: Hay que revisar todos los paquetes
		//Outgoing packets
		StackPacket<9,VALVE_STATE, float, float> regulator_packet;
		StackPacket<5,REED_STATE, REED_STATE, REED_STATE, REED_STATE, bool> reed_packet;
		StackPacket<16,double,double> bottle_temperature_packet;
		StackPacket<12,float,float,float> pressure_packets;
		StackPacket<8, float,float> environmental_packet;

		//Incoming packets

		Packets(Data<VEHICLE>& data) :
				regulator_packet(211, &data.valve_state, &data.regulator_reference_pressure, &data.regulator_real_pressure),
				reed_packet(212, &data.reed1, &data.reed2, &data.reed3, &data.reed4, &data.reeds_ok),
				bottle_temperature_packet(213, &data.bottle_temperature1, &data.bottle_temperature2),
				pressure_packets(214, &data.high_pressure1, &data.low_pressure1, &data.low_pressure2),
				environmental_packet(215, &data.enviremont_pressure, &data.enviroment_temperature)
		{}
	};
}
