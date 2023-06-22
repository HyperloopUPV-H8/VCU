#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class SpecificStateMachine;

	template<>
	class SpecificStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		LoadStateMachine<VEHICLE> health_load_state_machine;
		UnloadStateMachine<VEHICLE> health_unload_state_machine;
		TractionStateMachine<VEHICLE> traction_state_machine;
		StaticLevStateMachine<VEHICLE> static_lev_state_machine;
		DynamicLevStateMachine<VEHICLE> dynamic_lev_state_machine;

		StackStateOrder<0> healthcheck_and_load;
		StackStateOrder<0> healthcheck_and_unload;
		StackStateOrder<0> start_static_lev;
		StackStateOrder<0, vector<uint32_t>> start_dynamic_lev;
		StackStateOrder<0, vector<uint32_t>> start_traction;

		StateMachine state_machine;

		static bool healthcheck_and_load_requested;
		static bool healthcheck_and_unload_requested;
		static bool start_static_lev_requested;
		static bool start_dynamic_lev_requested;
		static bool start_traction_requested;

		enum SpecificStates{
			Idle,
			Unload,
			Load,
			DynamicLev,
			StaticLev,
			Traction,

		};

		SpecificStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
					data(data),actuators(actuators),tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),

					health_load_state_machine(data, actuators, tcp, outgoing_orders, encoder),
					health_unload_state_machine(data, actuators, tcp, outgoing_orders, encoder),
					traction_state_machine(data, actuators, tcp, outgoing_orders, encoder),
					static_lev_state_machine(data, actuators, tcp, outgoing_orders, encoder),
					dynamic_lev_state_machine(data, actuators, tcp, outgoing_orders, encoder),

					healthcheck_and_load((uint16_t)IncomingOrdersIDs::heakthcheck_and_load, enter_health_and_load, state_machine, Idle),
					healthcheck_and_unload((uint16_t)IncomingOrdersIDs::healthcheck_and_unload, enter_health_and_unload, state_machine, Idle),
					start_static_lev((uint16_t)IncomingOrdersIDs::start_static_lev_demostration, enter_static_lev, state_machine, Idle),
					start_dynamic_lev((uint16_t)IncomingOrdersIDs::start_dynamic_lev_demostration, enter_dynamic_lev, state_machine, Idle, &data.traction_points),
					start_traction((uint16_t)IncomingOrdersIDs::start_traction_demostration, enter_traction, state_machine, Idle, &data.traction_points)
		{}

		static void enter_health_and_load(){
			healthcheck_and_load_requested = true;
		}

		static void enter_health_and_unload(){
			healthcheck_and_unload_requested = true;
		}

		static void enter_dynamic_lev(){
			start_static_lev_requested = true;
		}

		static void enter_static_lev(){
			start_dynamic_lev_requested = true;
		}

		static void enter_traction(){
			start_traction_requested = true;
		}

		void add_transitions(){
			state_machine.add_transition(Idle, Load, [&](){
				return healthcheck_and_load_requested;
			});

			state_machine.add_transition(Idle, Unload, [&](){
				return healthcheck_and_unload_requested;
			});

			state_machine.add_transition(Idle, DynamicLev, [&](){
				return start_static_lev_requested;
			});

			state_machine.add_transition(Idle, StaticLev, [&](){
				return start_dynamic_lev_requested;
			});

			state_machine.add_transition(Idle, Traction, [&](){
				return start_traction_requested;
			});

			state_machine.add_transition(Load, Idle, [&](){
				return health_load_state_machine.ended;
			});

			state_machine.add_transition(Unload, Idle, [&](){
				return health_unload_state_machine.ended;
			});

			state_machine.add_transition(DynamicLev, Idle, [&](){
				return dynamic_lev_state_machine.ended;
			});

			state_machine.add_transition(StaticLev, Idle, [&](){
				return static_lev_state_machine.ended;
			});

			state_machine.add_transition(Traction, Idle, [&](){
				return traction_state_machine.ended;
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				health_load_state_machine.ended = false;
				health_unload_state_machine.ended = false;
				dynamic_lev_state_machine.ended = false;
				static_lev_state_machine.ended = false;
				traction_state_machine.ended = false;

				healthcheck_and_load_requested = false;
				healthcheck_and_unload_requested = false;
				start_static_lev_requested = false;
				start_dynamic_lev_requested = false;
				start_traction_requested = false;
			}, Idle);
		}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Unload);
			state_machine.add_state(Load);
			state_machine.add_state(DynamicLev);
			state_machine.add_state(StaticLev);
			state_machine.add_state(Traction);

			state_machine.add_state_machine(health_unload_state_machine.state_machine, Unload);
			state_machine.add_state_machine(health_load_state_machine.state_machine, Load);
			state_machine.add_state_machine(traction_state_machine.state_machine, Traction);
			state_machine.add_state_machine(static_lev_state_machine.state_machine, StaticLev);
			state_machine.add_state_machine(dynamic_lev_state_machine.state_machine, DynamicLev);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}
	};

	bool VCU::SpecificStateMachine<VEHICLE>::healthcheck_and_load_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::healthcheck_and_unload_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::start_static_lev_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::start_dynamic_lev_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::start_traction_requested = false;
}
