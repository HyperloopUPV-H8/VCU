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
		Packets(Data<BRAKE_VALIDATION>& data) : regulator_packet(211, &data.valve_state, &data.regulator_reference_pressure, &data.regulator_real_pressure),
				reed_packet(212,&data.reed),
				bottle_temperature_packet(213, &data.bottle_temperature1, &data.bottle_temperature2),
				pressure_packets(214, &data.high_pressure1, &data.low_pressure1, &data.low_pressure2)
		{}
	};
}
