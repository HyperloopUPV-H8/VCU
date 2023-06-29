#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class FaultSpecificStateMachine;

	template<>
	class FaultSpecificStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		UnloadStateMachine<VEHICLE> health_unload_state_machine;


		StateMachine state_machine;

		static bool healthcheck_and_unload_requested;

		enum SpecificStates{
			Idle,
			Unload,
		};

		FaultSpecificStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
					data(data),actuators(actuators),tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),
					health_unload_state_machine(data, actuators, tcp, outgoing_orders, encoder)

		{
			init();
		}

		static void enter_health_and_unload(){
			healthcheck_and_unload_requested = true;
		}

		void add_transitions(){
			state_machine.add_transition(Idle, Unload, [&](){
				return healthcheck_and_unload_requested;
			});

			state_machine.add_transition(Unload, Idle, [&](){
				return health_unload_state_machine.ended;
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				healthcheck_and_unload_requested = false;
			}, Idle);
		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
				health_unload_state_machine.ended = false;
			}, Unload);
		}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Unload);

			state_machine.add_state_machine(health_unload_state_machine.state_machine, Unload);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();

			data.specific_state = &state_machine.current_state;
		}
	};

	bool VCU::SpecificStateMachine<VEHICLE>::healthcheck_and_unload_requested = false;
}
