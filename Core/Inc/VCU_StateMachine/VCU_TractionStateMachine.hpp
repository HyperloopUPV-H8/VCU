#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class TractionStateMachine;

	template<>
	class TractionStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		CloseContactorsStateMachine<VEHICLE> close_contactors_state_machine;
		OpenContactorsStateMachine<VEHICLE> open_contactors_state_machine;
		PointTravelStateMachine<VEHICLE> point_travel_state_machine;

		StateMachine state_machine;

		bool ended = false;

		enum DynamicLevStates{
			Idle,
			CloseContactors,
			PointTravel,
			OpenContactors,
		};

		uint32_t aux_last_state;

		TractionStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>&actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
			data(data),actuators(actuators),tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),
			close_contactors_state_machine(data, tcp_handler, outgoing_orders),
			open_contactors_state_machine(data, tcp_handler, outgoing_orders),
			point_travel_state_machine(data, tcp_handler, outgoing_orders)
		{}

		void add_transitions(){
			state_machine.add_transition(Idle, CloseContactors, [&](){
				if(not ended){
					return true;
				}
				return false;
			});

			state_machine.add_transition(CloseContactors, PointTravel, [&](){
				return close_contactors_state_machine.ended;
			});

			state_machine.add_transition(PointTravel, OpenContactors, [&](){
				return point_travel_state_machine.ended;
			});

			state_machine.add_transition(OpenContactors, Idle, [&](){
				return open_contactors_state_machine.ended;
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				actuators.brakes.not_brake();
				actuators.brakes.enable_emergency_brakes();
			}, CloseContactors);

			state_machine.add_enter_action([&](){
				actuators.brakes.brake();
				actuators.brakes.enable_emergency_brakes();
			}, OpenContactors);

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
			state_machine.add_state(PointTravel);
			state_machine.add_state(OpenContactors);

			state_machine.add_state_machine(close_contactors_state_machine.state_machine, CloseContactors);
			state_machine.add_state_machine(open_contactors_state_machine.state_machine, OpenContactors);
			state_machine.add_state_machine(point_travel_state_machine.state_machine, PointTravel);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}
	};
}
