#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class CloseContactorsStateMachine;

	template<>
	class CloseContactorsStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		StateMachine state_machine;

		bool ended = false;

		enum CloseContactorStates{
			RequestClose,
			Closed,
		};

		CloseContactorsStateMachine(Data<VEHICLE>& data, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE> outgoing_orders) :
			data(data), tcp_handler(tcp), outgoing_orders(outgoing_orders)
		{
			init();
		}

		void add_transitions(){
			state_machine.add_transition(RequestClose, Closed, [&](){
				if (data.contactors_state == ContactorState::Close) {
					ended = true;
					return true;
				}

				return false;
			});

		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				tcp_handler.send_to_obccu(outgoing_orders.close_contactors);
			}, RequestClose);

		}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine = {RequestClose};
			state_machine.add_state(Closed);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};

	template<VCU_MODE> class OpenContactorsStateMachine;

	template<>
	class OpenContactorsStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		StateMachine state_machine;

		bool ended = false;

		enum OpenContactorsStates{
			RequestOpen,
			Opened,
		};

		OpenContactorsStateMachine(Data<VEHICLE>& data, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE> outgoing_orders) :
			data(data), tcp_handler(tcp), outgoing_orders(outgoing_orders)
		{
			init();
		}

		void add_transitions(){
			state_machine.add_transition(RequestOpen, Opened, [&](){
				if (data.contactors_state == ContactorState::Open) {
					ended = true;
					return true;
				}

				return false;
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				tcp_handler.send_to_obccu(outgoing_orders.open_contactors);
			}, RequestOpen);
		}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){
			state_machine.add_state(RequestOpen);
			state_machine.add_state(Opened);


			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
