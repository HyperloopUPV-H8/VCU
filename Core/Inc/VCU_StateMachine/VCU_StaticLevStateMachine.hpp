#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class StaticLevStateMachine;

	template<>
	class StaticLevStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Brakes<VEHICLE>& brakes;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		StateMachine state_machine;

		enum DynamicLevStates{
			LevOff,
			LevOn,
		};

		StaticLevStateMachine(Data<VEHICLE>& data, Brakes<VEHICLE>&brakes, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data),brakes(brakes),tcp_handler(tcp), encoder(encoder)
		{}

		void add_transitions(){
			//TODO: Las transiciones son enteramente con state orders

		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				brakes.not_brake();
				//TODO: set lev parameters
				//TODO: send lev order
			}, LevOn);

			state_machine.add_enter_action([&](){
				//TODO: send stop lev order
				brakes.brake();
			}, LevOff);

		}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine = {LevOff};
			state_machine.add_state(LevOn);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
