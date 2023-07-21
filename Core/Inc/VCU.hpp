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
		OutgoingOrders<VCU_MODE::VEHICLE> outgoing_orders;
		Packets<VCU_MODE::VEHICLE> packets;
		EncoderSensor encoder;
		GeneralStateMachine<VCU_MODE::SEC_TEST> state_machine_handler;

		VCU_CLASS():data(), actuators(data), environmental_sensors(data), tcp_handler(), udp_handler(), incoming_orders(data), packets(data),
					encoder(Pinout::TAPE1, Pinout::TAPE2, &data.tapes_position, &data.tapes_direction, &data.tapes_speed, &data.tapes_acceleration)
					,state_machine_handler(data, actuators, tcp_handler, incoming_orders)
				{}

		void init(){
			STLIB::start();
			encoder.start();
			actuators.brakes.init();
			tcp_handler.init();
			udp_handler.init();
		}

		static void read_brakes_sensors(){
			vcu->actuators.brakes.read();
		}

		static void read_environmental_sensors(){
			vcu->environmental_sensors.read();
		}

		static void read_encoder(){
			vcu->encoder.read();
		}

		static void send_to_backend(){
			vcu->udp_handler.send_to_backend(vcu->packets.regulator_packet);
			vcu->udp_handler.send_to_backend(vcu->packets.pressure_packets);
			vcu->udp_handler.send_to_backend(vcu->packets.bottle_temperature_packet);
			vcu->udp_handler.send_to_backend(vcu->packets.environmental_packet);
			vcu->udp_handler.send_to_backend(vcu->packets.reeds_packet);
//			vcu->udp_handler.send_to_backend(vcu->packets.states_packet);
			vcu->udp_handler.send_to_backend(vcu->packets.state_machines_packet);
			vcu->udp_handler.send_to_backend(vcu->packets.encoder_packet);

		}


		static void update_state_machine(){
			vcu->state_machine_handler.general_state_machine.check_transitions();
		}
	};

	void set_regulator_pressure(){
		float n_pressure = VCU_CLASS<VEHICLE>::vcu->incoming_orders.new_pressure;
		if( n_pressure < 0 || n_pressure > 10 ){
			ProtectionManager::warn("The new value for the regulator is out of range!");
			return;
		}

		VCU_CLASS<VEHICLE>::vcu->actuators.brakes.set_regulator_pressure(n_pressure);
	}

	void brake(){
		VCU_CLASS<VEHICLE>::vcu->actuators.brakes.brake();
		if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.is_connected()){
			VCU_CLASS<VEHICLE>::vcu->tcp_handler.send_to_pcu(VCU_CLASS<VEHICLE>::vcu->incoming_orders.stop_traction_order);
		}
	}

	bool first_unbrake_order = true;
	void unbrake(){
		if (first_unbrake_order) {
			first_unbrake_order = false;
		}

		VCU_CLASS<VEHICLE>::vcu->actuators.brakes.not_brake();
	}

	void disable_emergency_tape(){
		VCU_CLASS<VEHICLE>::vcu->actuators.brakes.disable_emergency_brakes();
	}

	void enable_emergency_tape(){
		VCU_CLASS<VEHICLE>::vcu->actuators.brakes.enable_emergency_brakes();
	}

	void close_contactors(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.OBCCU_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.OBCCU_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.close_contactors_order);
				VCU_CLASS<VEHICLE>::vcu->state_machine_handler.specific_state_machine_handler.contactor_state = true;
			}
			else ErrorHandler("OBCCU is not connected to VCU");
		}

		void open_contactors(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.OBCCU_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.OBCCU_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.open_contactors_order);
				VCU_CLASS<VEHICLE>::vcu->state_machine_handler.specific_state_machine_handler.contactor_state = false;
			}
			else ErrorHandler("OBCCU is not connected to VCU");
		}

		void start_vertical_levitation(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.start_vertical_levitation_order);
				VCU_CLASS<VEHICLE>::vcu->state_machine_handler.specific_state_machine_handler.is_levitating = true;
			}
			else ErrorHandler("LCU is not connected to VCU");
		}

		void start_lateral_levitation(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.is_connected())
			VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.start_lateral_levitation_order);
			else ErrorHandler("LCU is not connected to VCU");
		}

		void start_traction(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.start_traction_order);
				VCU_CLASS<VEHICLE>::vcu->state_machine_handler.specific_state_machine_handler.is_running = true;
			}
			else ErrorHandler("PCU is not connected to VCU");
		}

		void stop_levitation(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.stop_levitation_order);
				VCU_CLASS<VEHICLE>::vcu->state_machine_handler.specific_state_machine_handler.is_levitating = false;
			}
			else ErrorHandler("LCU is not connected to VCU");
		}

		void stop_traction(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->incoming_orders.stop_traction_order);
				VCU_CLASS<VEHICLE>::vcu->state_machine_handler.specific_state_machine_handler.is_running = false;
				VCU_CLASS<VEHICLE>::vcu->actuators.brakes.brake();
			}
			else ErrorHandler("PCU is not connected to VCU");
		}

		void emergency_stop(){
			ErrorHandler("Emergency stop");
		}

		void vehicle_reset(){
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.LCU_MASTER_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->outgoing_orders.lcu_reset_all);
			}
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.PCU_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->outgoing_orders.pcu_reset);
			}
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.OBCCU_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.OBCCU_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->outgoing_orders.obccu_reset);
			}
			if(VCU_CLASS<VEHICLE>::vcu->tcp_handler.BMSL_CONNECTION.is_connected()){
				VCU_CLASS<VEHICLE>::vcu->tcp_handler.BMSL_CONNECTION.send_order(VCU_CLASS<VEHICLE>::vcu->outgoing_orders.bmsl_reset);
			}
			HAL_NVIC_SystemReset();
		}
}



VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>* VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>::vcu = nullptr;
VCU::VCU_CLASS<VCU::VCU_MODE::VEHICLE>* VCU::VCU_CLASS<VCU::VCU_MODE::VEHICLE>::vcu = nullptr;
