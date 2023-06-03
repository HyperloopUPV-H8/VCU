#pragma once

#include <VCU_Communications/VCU_TCP/VCU_TCP.hpp>
#include <VCU_Communications/VCU_UDP/VCU_UDP.hpp>
#include "VCU_Pinout/Pinout.hpp"
#include "VCU_Mode/VCU_Mode.hpp"
#include "VCU_Data/VCU_Data.hpp"
#include "VCU_Sensors/VCU_EnviromentalSensors.hpp"
#include "VCU_Sensors/VCU_RegulatorSensor.hpp"
#include "VCU_Sensors/VCU_Reed.hpp"
#include "VCU_Actuators/VCU_LedsActuator.hpp"
#include "VCU_Actuators/VCU_RegulatorActuator.hpp"
#include "VCU_Actuators/VCU_ValveActuator.hpp"
#include "VCU_Brakes/VCU_Brakes.hpp"
#include "VCU_Utilities/VCU_Types.hpp"
#include "VCU_Communications/VCU_TCP/IncomingOrders.hpp"
#include "VCU_Communications/VCU_UDP/Packets.hpp"


namespace VCU{
	template<VCU_MODE> class VCU_CLASS;

	template<> class VCU_CLASS<VCU_MODE::BRAKE_VALIDATION>{
	public:
		static VCU_CLASS* vcu;

		Data<VCU_MODE::BRAKE_VALIDATION> data;
		Brakes<VCU_MODE::BRAKE_VALIDATION> brakes;
		TCP<VCU_MODE::BRAKE_VALIDATION> tcp_handler;
		UDP<VCU_MODE::BRAKE_VALIDATION> udp_handler;
		IncomingOrders<VCU_MODE::BRAKE_VALIDATION> incoming_orders;
		Packets<VCU_MODE::BRAKE_VALIDATION> packets;

		VCU_CLASS():data(), brakes(data), tcp_handler(), udp_handler(), incoming_orders(data), packets(data){}

		void init(){
			STLIB::start();
			brakes.init();
			udp_handler.init();
			tcp_handler.init();
		}

		static void read_brakes_sensors(){
			vcu->brakes.read();
		}

		static void send_to_backend(){
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.regulator_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.pressure_packets);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.bottle_temperature_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.reed_packet);
		}
	};

	void set_regulator_pressure(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->brakes.set_regulator_pressure(VCU_CLASS<BRAKE_VALIDATION>::vcu->incoming_orders.new_pressure);
	}

	void brake(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->brakes.brake();
	}

	void unbrake(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->brakes.not_brake();
	}

	void disable_emergency_tape(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->brakes.disable_emergency_brakes();
	}

	void enable_emergency_tape(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->brakes.enable_emergency_brakes();
	}

}

VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>* VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>::vcu = nullptr;
