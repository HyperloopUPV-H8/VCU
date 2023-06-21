#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class TractionLevStateMachine;

	template<>
	class TractionLevStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		StateMachine state_machine;

		bool ended = false;

		enum DynamicLevStates{
			Idle,
			Accelerating,
			Brake,
		};

		TractionLevStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>&actuators, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data),actuators(actuators),tcp_handler(tcp), encoder(encoder)
		{}

		void add_transitions(){}

		void add_on_enter_actions(){}

		void add_on_exit_actions(){

		}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Accelerating);
			state_machine.add_state(Brake);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
