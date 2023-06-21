#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class VCU_CLASS;

	template<> class VCU_CLASS<VCU_MODE::BRAKE_VALIDATION>{
	public:
		static VCU_CLASS* vcu;

		Data<VCU_MODE::BRAKE_VALIDATION> data;
		Actuators<VCU_MODE::BRAKE_VALIDATION> actuators;
		TCP<VCU_MODE::BRAKE_VALIDATION> tcp_handler;
		UDP<VCU_MODE::BRAKE_VALIDATION> udp_handler;
		IncomingOrders<VCU_MODE::BRAKE_VALIDATION> incoming_orders;
		Packets<VCU_MODE::BRAKE_VALIDATION> packets;
		GeneralStateMachine<VCU_MODE::BRAKE_VALIDATION> general_state_machine;

		VCU_CLASS():
			data(), actuators(data), tcp_handler(), udp_handler(), incoming_orders(data), packets(data),
			general_state_machine(data, actuators, tcp_handler)
		{}

		void init(){
			STLIB::start();
			actuators.brakes.init();
			udp_handler.init();
			tcp_handler.init();
			general_state_machine.init();
			data.add_protections();
		}

		static void read_brakes_sensors(){
			vcu->actuators.brakes.read();
		}

		static void send_to_backend(){
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.regulator_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.pressure_packets);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.bottle_temperature_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.reed_packet);
		}

		static void update_state_machine(){
			vcu->general_state_machine.general_state_machine.check_transitions();
		}
	};

	template<> class VCU_CLASS<VCU_MODE::VEHICLE>{
	public:
		static VCU_CLASS* vcu;

		Data<VCU_MODE::VEHICLE> data;
		Actuators<VCU_MODE::VEHICLE> actuators;
		EnvironmentalSensors environmental_sensors;
		TCP<VCU_MODE::VEHICLE> tcp_handler;
		UDP<VCU_MODE::VEHICLE> udp_handler;
		IncomingOrders<VCU_MODE::VEHICLE> incoming_orders;
		Packets<VCU_MODE::VEHICLE> packets;
		EncoderSensor encoder;
		GeneralStateMachine<VCU_MODE::VEHICLE> state_machine_handler;

		VCU_CLASS():data(), actuators(data), environmental_sensors(data), tcp_handler(), udp_handler(), incoming_orders(data), packets(data),
					encoder(Pinout::TAPE1, Pinout::TAPE2, &data.tapes_position, &data.tapes_direction, &data.tapes_speed, &data.tapes_acceleration)
					,state_machine_handler(data, actuators, tcp_handler, encoder)
				{}

		void init(){
			STLIB::start();
			actuators.brakes.init();
			udp_handler.init();
			tcp_handler.init();
		}

		static void read_brakes_sensors(){
			vcu->actuators.brakes.read();
		}

		static void read_environmental_sensors(){
			vcu->environmental_sensors.read();
		}

		static void send_to_backend(){
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.regulator_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.pressure_packets);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.bottle_temperature_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.reed_packet);
			vcu->udp_handler.BACKEND_CONNECTION.send(vcu->packets.environmental_packet);
		}

		static void update_state_machine(){
			vcu->state_machine_handler.general_state_machine.check_transitions();
		}
	};

	//Esto hay que modificar Incoming orders para que pueda estar dentro de la clase VCU, aqui fuera no tiene sentido
	void set_regulator_pressure(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->actuators.brakes.set_regulator_pressure(VCU_CLASS<BRAKE_VALIDATION>::vcu->incoming_orders.new_pressure);
	}

	void brake(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->actuators.brakes.brake();
	}

	void unbrake(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->actuators.brakes.not_brake();
	}

	void disable_emergency_tape(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->actuators.brakes.disable_emergency_brakes();
	}

	void enable_emergency_tape(){
		VCU_CLASS<BRAKE_VALIDATION>::vcu->actuators.brakes.enable_emergency_brakes();
	}

}

VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>* VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>::vcu = nullptr;
