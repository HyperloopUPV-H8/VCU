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
#include "VCU_StateMachine/VCU_StateMachine.hpp"


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

		//ÑAPAS
		PinState emergency_tape_pinstate;
		SensorInterrupt emergency_tape_detector{Pinout::EMERGENCY_TAPE, [&](){
			if(general_state_machine.general_state_machine.current_state == GeneralStateMachine<BRAKE_VALIDATION>::OPERATIONAL )data.emergency_tape_detected = true;
		}, emergency_tape_pinstate, ExternalInterrupt::FALLING};

		VCU_CLASS():data(), actuators(data), tcp_handler(), udp_handler(), incoming_orders(data), packets(data), general_state_machine(data,actuators,tcp_handler){}

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
