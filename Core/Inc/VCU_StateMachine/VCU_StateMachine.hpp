/*
 * VCU_StateMachine.hpp
 *
 *  Created on: Jun 14, 2023
 *      Author: stefancostea
 */

#pragma once
#include "VCU_Mode/VCU_Mode.hpp"
#include "StateMachine/StateMachine.hpp"
#include "Protections/ProtectionManager.hpp"
#include "BoardID/BoardID.hpp"
#include "VCU_Brakes/VCU_Brakes.hpp"
#include "VCU_Communications/VCU_TCP/VCU_TCP.hpp"
#include "VCU_Actuators/Actuators.hpp"

using namespace std::chrono;

namespace VCU{
	template<VCU::VCU_MODE> class GeneralStateMachine;

	template<> class GeneralStateMachine<BRAKE_VALIDATION>{
	public:
		Actuators<BRAKE_VALIDATION>& actuators;
		TCP<BRAKE_VALIDATION>& tcp_handler;
		StateMachine general_state_machine;
		bool tcp_timeout = false;

		static constexpr uint16_t max_tcp_connection_timeout = 25000;

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
			general_state_machine.add_low_precision_cyclic_action([&](){
				actuators.led_operational.toggle();
			}, 150ms);
			general_state_machine.add_low_precision_cyclic_action(ProtectionManager::check_protections, 1ms, OPERATIONAL);
		}

	};

}
