#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class GeneralStateMachine;

	template<> class GeneralStateMachine<BRAKE_VALIDATION>{
	public:
		Actuators<BRAKE_VALIDATION>& actuators;
		TCP<BRAKE_VALIDATION>& tcp_handler;
		StateMachine general_state_machine;
		bool tcp_timeout = false;

		static constexpr uint16_t max_tcp_connection_timeout = 25000; //ms

		GeneralStateMachine(Data<BRAKE_VALIDATION>& data, Actuators<BRAKE_VALIDATION>& actuators , TCP<BRAKE_VALIDATION>& tcp_handler) : actuators(actuators), tcp_handler(tcp_handler)
		{}

		enum States{
			INITIAL,
			OPERATIONAL,
			FAULT
		};


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
					actuators.led_fault.turn_on();
					actuators.brakes.brake();
			}, FAULT);

			general_state_machine.add_enter_action([&](){
				actuators.led_operational.turn_on();
			}, OPERATIONAL);
		}

		void add_on_exit_actions(){
			general_state_machine.add_exit_action([&](){
				actuators.led_fault.turn_off();
			}, FAULT);
			general_state_machine.add_exit_action([&](){
				actuators.led_operational.turn_off();
			}, OPERATIONAL);
		}

		void add_transitions(){
			general_state_machine.add_transition(INITIAL, OPERATIONAL, [&](){
				return tcp_handler.BACKEND_CONNECTION.state == ServerSocket::ServerState::ACCEPTED;
			});
			general_state_machine.add_transition(INITIAL, FAULT, [&](){
				if(tcp_timeout) ErrorHandler("TCP connections could not be established in time and timed out");
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

		void register_timed_actions(){
			general_state_machine.add_low_precision_cyclic_action([&](){actuators.led_operational.toggle();}, (ms)150, INITIAL);
			general_state_machine.add_low_precision_cyclic_action(ProtectionManager::check_protections, (ms)1, OPERATIONAL);
		}

	};

	template<>
	class GeneralStateMachine<VCU_MODE::VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
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

		GeneralStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), encoder(encoder), specific_state_machine(data, actuators, tcp, encoder)
		{}

		void add_transitions(){
			//todo: COMPROBAR que estan todos conectados antes de pasar a operational
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
				actuators.led_fault.turn_on();
				actuators.brakes.brake();
				actuators.brakes.enable_emergency_brakes();
			}, FAULT);

			general_state_machine.add_enter_action([&](){
				actuators.led_operational.turn_on();
				actuators.brakes.enable_emergency_brakes();
			}, OPERATIONAL);
		}

		void add_on_exit_actions(){
			general_state_machine.add_exit_action([&](){
				actuators.led_fault.turn_off();
				actuators.brakes.not_brake();
			}, FAULT);
			general_state_machine.add_exit_action([&](){
				actuators.led_operational.turn_on();
			}, OPERATIONAL);
			general_state_machine.add_exit_action([&](){
				actuators.led_operational.turn_off();
			}, INITIAL);
		}

		void register_timed_actions(){
			general_state_machine.add_low_precision_cyclic_action([&](){actuators.led_operational.toggle();}, (ms)150, INITIAL);
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


