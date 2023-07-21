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

		StackStateOrder<0> start_crawling;
		StackStateOrder<0> take_off;
		StackStateOrder<0> landing;

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
					start_crawling((uint16_t)IncomingOrdersIDs::start_crawling, LoadStateMachine<VEHICLE>::enter_crawling, health_load_state_machine.state_machine, LoadStateMachine<VEHICLE>::UnloadStates::Pushing),
					take_off((uint16_t)IncomingOrdersIDs::take_off, StaticLevStateMachine<VEHICLE>::start_levitation, static_lev_state_machine.state_machine, StaticLevStateMachine<VEHICLE>::StaticLevStates::LevOff),
					landing((uint16_t)IncomingOrdersIDs::landing, StaticLevStateMachine<VEHICLE>::stop_levitation, static_lev_state_machine.state_machine, StaticLevStateMachine<VEHICLE>::StaticLevStates::LevOn)

		{
			init();
		}

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
				healthcheck_and_load_requested = false;
				healthcheck_and_unload_requested = false;
				start_static_lev_requested = false;
				start_dynamic_lev_requested = false;
				start_traction_requested = false;
			}, Idle);
		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
				health_unload_state_machine.ended = false;
			}, Unload);

			state_machine.add_exit_action([&](){
				health_load_state_machine.ended = false;
			}, Load);

			state_machine.add_exit_action([&](){
				traction_state_machine.ended = false;
			}, Traction);

			state_machine.add_exit_action([&](){
				static_lev_state_machine.ended = false;
			}, StaticLev);

			state_machine.add_exit_action([&](){
				dynamic_lev_state_machine.ended = false;
			}, DynamicLev);
		}

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

			data.specific_state = &state_machine.current_state;
		}
	};

	bool VCU::SpecificStateMachine<VEHICLE>::healthcheck_and_load_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::healthcheck_and_unload_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::start_static_lev_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::start_dynamic_lev_requested = false;
	bool VCU::SpecificStateMachine<VEHICLE>::start_traction_requested = false;

	template<>
	class SpecificStateMachine<SEC_TEST>{
	public:
		enum VoltageStates : uint8_t{
			NOT_HV,
			HV
		};
		enum SpecificStates : uint8_t{
			IDLE,
			LEVITATING,
			RUNNING,
			END
		};
		bool contactor_state = false;
		bool is_levitating = false;
		bool is_running = false;
		double desired_position = 0.0;
		StateMachine voltage_state_machine;
		StateMachine specific_state_machine;
		double& real_position;
		TCP<VEHICLE>& tcp_handler;
		IncomingOrders<VEHICLE>& incoming_orders;
		Actuators<VEHICLE>& actuators;
		SpecificStateMachine(Data<VEHICLE>& data, double& position, TCP<VEHICLE>& tcp_handler, IncomingOrders<VEHICLE>& incoming_orders, Actuators<VEHICLE>& actuators) : voltage_state_machine(NOT_HV), specific_state_machine(IDLE),real_position(position),tcp_handler(tcp_handler), incoming_orders(incoming_orders),
				actuators(actuators){
			init();
			data.specific_state = &specific_state_machine.current_state;
			data.voltage_state = &voltage_state_machine.current_state;
		}
		void init(){
			voltage_state_machine.add_state(HV);
			voltage_state_machine.add_transition(NOT_HV, HV, [&](){
				return contactor_state;
			});
			voltage_state_machine.add_transition(HV, NOT_HV, [&](){
				return not contactor_state;
			});
			voltage_state_machine.add_exit_action([&](){
				tcp_handler.LCU_MASTER_CONNECTION.send_order(incoming_orders.stop_levitation_order);
				tcp_handler.PCU_CONNECTION.send_order(incoming_orders.stop_traction_order);
				tcp_handler.OBCCU_CONNECTION.send_order(incoming_orders.open_contactors_order);
			},HV);
			specific_state_machine.add_state(LEVITATING);
			specific_state_machine.add_state(RUNNING);
			specific_state_machine.add_state(END);
			specific_state_machine.add_transition(IDLE, LEVITATING, [&](){
				return is_levitating;
			});
			specific_state_machine.add_transition(LEVITATING, RUNNING, [&](){
				return is_running;
			});
			specific_state_machine.add_transition(RUNNING, END, [&](){
				return real_position >= desired_position;
			});
			specific_state_machine.add_enter_action([&](){
				tcp_handler.LCU_MASTER_CONNECTION.send_order(incoming_orders.stop_levitation_order);
				tcp_handler.PCU_CONNECTION.send_order(incoming_orders.stop_traction_order);
				tcp_handler.OBCCU_CONNECTION.send_order(incoming_orders.open_contactors_order);
				actuators.brakes.brake();
			}, END);
			specific_state_machine.add_state_machine(specific_state_machine, HV);
		}

	};
}
