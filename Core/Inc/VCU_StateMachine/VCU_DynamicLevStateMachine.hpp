#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class DynamicLevStateMachine;

	template<>
	class DynamicLevStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;
		StateMachine state_machine;

		bool ended = false;

		enum DynamicLevStates{
			Idle,
			CloseContactors,
			TakeOff,
			Accelerating,
			Brake,
			Landing,
			OpenContactors,
		};

		DynamicLevStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder)
		{}

		void add_transitions(){}

		void add_on_enter_actions(){}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(CloseContactors);
			state_machine.add_state(TakeOff);
			state_machine.add_state(Accelerating);
			state_machine.add_state(Brake);
			state_machine.add_state(Landing);
			state_machine.add_state(OpenContactors);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
