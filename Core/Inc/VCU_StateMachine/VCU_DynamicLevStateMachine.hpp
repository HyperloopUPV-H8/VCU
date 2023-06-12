#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class DynamicLevStateMachine;

	template<>
	class DynamicLevStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Brakes<VEHICLE>& brakes;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		StateMachine state_machine;

		enum DynamicLevStates{
			Idle,
			TakeOff,
			Accelerating,
			Brake,
			Landing,
		};

		DynamicLevStateMachine(Data<VEHICLE>& data, Brakes<VEHICLE>&brakes, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data),brakes(brakes),tcp_handler(tcp), encoder(encoder)
		{}

		void add_transitions(){}

		void add_on_enter_actions(){}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(TakeOff);
			state_machine.add_state(Accelerating);
			state_machine.add_state(Brake);
			state_machine.add_state(Landing);


			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
