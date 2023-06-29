#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class StaticLevStateMachine;

	template<>
	class StaticLevStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		CloseContactorsStateMachine<VEHICLE> close_contactors_state_machine;
		OpenContactorsStateMachine<VEHICLE> open_contactors_state_machine;

		StateMachine state_machine;

		bool ended = false;

		static bool start_levitation_requested;
		static bool stop_levitation_requested;

		enum StaticLevStates{
			Idle,
			CloseContactors,
			LevOff,
			LevOn,
			OpenContactors,
		};

		StaticLevStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),
			close_contactors_state_machine(data, tcp_handler, outgoing_orders),
			open_contactors_state_machine(data, tcp_handler, outgoing_orders)
		{
			init();
		}

		static void start_levitation(){
			start_levitation_requested = true;
		}

		static void stop_levitation(){
			stop_levitation_requested = true;
		}

		void add_transitions(){
			state_machine.add_transition(Idle, CloseContactors, [&](){
				if(not ended){
					return true;
				}
				return false;
			});

			state_machine.add_transition(CloseContactors, LevOff, [&](){
				return close_contactors_state_machine.ended;
			});

			state_machine.add_transition(LevOff, LevOn, [&](){
				if(start_levitation_requested){
					start_levitation_requested = false;
					return true;
				}
				return false;
			});

			state_machine.add_transition(LevOn, OpenContactors, [&](){
				if(stop_levitation_requested){
					stop_levitation_requested = false;
					return true;
				}
				return false;
			});

			state_machine.add_transition(OpenContactors, Idle, [&](){
				return open_contactors_state_machine.ended;
			});
		}

		void add_on_enter_actions(){

			state_machine.add_enter_action([&](){
				actuators.brakes.not_brake();
				tcp_handler.send_to_lcu(outgoing_orders.take_off_order);
			}, LevOn);

			state_machine.add_enter_action([&](){
				tcp_handler.send_to_lcu(outgoing_orders.landing_order);
				actuators.brakes.brake();
			}, LevOff);

		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
				ended = true;
			}, OpenContactors);
		}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(CloseContactors);
			state_machine.add_state(LevOff);
			state_machine.add_state(LevOn);
			state_machine.add_state(OpenContactors);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};

	bool StaticLevStateMachine<VEHICLE>::start_levitation_requested = false;
	bool StaticLevStateMachine<VEHICLE>::stop_levitation_requested = false;
}
