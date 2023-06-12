#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class SpecificStateMachine;

	template<>
	class SpecificStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Brakes<VEHICLE>& brakes;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		LoadStateMachine<VEHICLE> health_load_state_machine;
		UnloadStateMachine<VEHICLE> health_unload_state_machine;
		TractionLevStateMachine<VEHICLE> traction_state_machine;
		StaticLevStateMachine<VEHICLE> static_lev_state_machine;
		DynamicLevStateMachine<VEHICLE> dynamic_lev_state_machine;
		StateMachine state_machine;
		//TODO: Añadir todas las statemachines

		enum SpecificStates{
			Idle,
			Unload,
			Load,
			DynamicLev,
			StaticLev,
			Traction,
		};

		SpecificStateMachine(Data<VEHICLE>& data, Brakes<VEHICLE>&brakes, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
							data(data),brakes(brakes),tcp_handler(tcp), encoder(encoder),
							health_load_state_machine(data, brakes, tcp, encoder),
							health_unload_state_machine(data, brakes, tcp, encoder),
							traction_state_machine(data, brakes, tcp, encoder),
							static_lev_state_machine(data, brakes, tcp, encoder),
							dynamic_lev_state_machine(data, brakes, tcp, encoder)
		{}

		void add_transitions(){}

		void add_on_enter_actions(){}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Unload);
			state_machine.add_state(Load);
			state_machine.add_state(DynamicLev);
			state_machine.add_state(StaticLev);
			state_machine.add_state(Traction);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();

			//TODO: Añadir todas las state machines
			//state_machine.add_state_machine(.state_machine, OPERATIONAL);
		}

	};
}
