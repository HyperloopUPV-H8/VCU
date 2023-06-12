#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class GeneralStateMachine;

	template<>
	class GeneralStateMachine<VCU_MODE::VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Brakes<VEHICLE>& brakes;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		SpecificStateMachine<VEHICLE> specific_state_machine;
		StateMachine general_state_machine;

		bool tcp_timeout = false;

		static constexpr uint16_t max_tcp_connection_timeout = 30000; //ms

		enum States{
			INITIAL,
			OPERATIONAL,
			FAULT
		};

		GeneralStateMachine(Data<VEHICLE>& data, Brakes<VEHICLE>&brakes, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data),brakes(brakes),tcp_handler(tcp), encoder(encoder), specific_state_machine(data, brakes, tcp, encoder)
		{}

		void add_transitions(){
			general_state_machine.add_transition(INITIAL, OPERATIONAL, [&](){
				return tcp_handler.BACKEND_CONNECTION.state == ServerSocket::ServerState::ACCEPTED ;
			});
			general_state_machine.add_transition(INITIAL, FAULT, [&](){
				if(tcp_timeout){
					ErrorHandler("TCP connections could not be established in time and timed out");
				}
				return tcp_timeout;
			});
			general_state_machine.add_transition(OPERATIONAL, FAULT, [&](){
				if(tcp_handler.BACKEND_CONNECTION.state != ServerSocket::ServerState::ACCEPTED){
					ErrorHandler("TCP connections fell");
					return true;
				}
				return false;
			});
		}

		void add_on_enter_actions(){
			general_state_machine.add_enter_action([&](){
				Time::set_timeout(max_tcp_connection_timeout, [&](){
					if(not (tcp_handler.BACKEND_CONNECTION.state == ServerSocket::ServerState::ACCEPTED)){
								tcp_timeout = true;
					}
				});
			}, INITIAL);

			Time::set_timeout(max_tcp_connection_timeout, [&](){
				if(not (tcp_handler.BACKEND_CONNECTION.state == ServerSocket::ServerState::ACCEPTED)){
							tcp_timeout = true;
				}
			});

			general_state_machine.add_enter_action([&](){
				 //TODO: encender led fault
				 brakes.brake();
				 brakes.enable_emergency_brakes();

			}, FAULT);

			general_state_machine.add_enter_action([&](){
				//TODO: encender led operational
				brakes.enable_emergency_brakes();
			}, OPERATIONAL);
		}

		void add_on_exit_actions(){
			general_state_machine.add_exit_action([&](){
				//TODO: apagar led fault
				brakes.not_brake();
			}, FAULT);
			general_state_machine.add_exit_action([&](){
				//TODO: apagar led operational
			}, OPERATIONAL);
			general_state_machine.add_exit_action([&](){
				//TODO: apagar led operational
			}, INITIAL);
		}

		void register_timed_actions(){
			//TODO: general_state_machine.add_low_precision_cyclic_action([&](){actuators.led_operational.toggle();}, 150ms, INITIAL);
			general_state_machine.add_low_precision_cyclic_action(ProtectionManager::check_protections, (ms)1, OPERATIONAL);
		}

		void init(){
			general_state_machine = {INITIAL};
			general_state_machine.add_state(OPERATIONAL);
			general_state_machine.add_state(FAULT);

			ProtectionManager::link_state_machine(general_state_machine, FAULT);
			ProtectionManager::set_id(Boards::ID::VCU);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();

			general_state_machine.add_state_machine(specific_state_machine.state_machine, OPERATIONAL);
		}
	};
}


