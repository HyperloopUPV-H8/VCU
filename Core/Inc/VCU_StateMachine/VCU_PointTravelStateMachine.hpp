#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class PointTravelStateMachine;

	template<>
	class PointTravelStateMachine<VEHICLE>{
		static constexpr uint32_t initial_aux_state = 0;

	public:
		Data<VEHICLE>& data;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;

		StateMachine state_machine;

		enum TravelStates{
			ReceivingData,
			Calculating,
			MovingForward,
			MovingBackward,
			Braking,
			End,
		};

		bool ended = false;

		uint32_t aux_last_state;

		PointTravelStateMachine(Data<VEHICLE>& data, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders) :
			data(data),tcp_handler(tcp), outgoing_orders(outgoing_orders)
		{}

		void add_transitions(){

			uint32_t next_state;
			for(uint32_t current_state = initial_aux_state; current_state < data.traction_points.size() + initial_aux_state; current_state++){

				if (current_state + 1 > aux_last_state) {
					next_state = (uint32_t)End;
				}else{
					next_state = ++current_state;
				}

				state_machine.add_transition(current_state, next_state, [&](){
					uint32_t speed = std::ceil(std::max(data.target_speed, data.engine_speed));
					uint32_t brake_distance = data.brake_distance_lookup_table[speed];
					uint32_t objective_position = data.traction_points.at(current_state - initial_aux_state);
					if (data.tapes_position > objective_position) {
						return data.tapes_position <= (objective_position + brake_distance);
					}else{
						return data.tapes_position >= (objective_position - brake_distance);
					}
				});
			}
		}

		void add_on_enter_actions(){
			for(uint32_t current_state = initial_aux_state; current_state < data.traction_points.size() + initial_aux_state; current_state++){

				state_machine.add_enter_action([&](){
					uint32_t objective_position = data.traction_points.at(current_state - initial_aux_state);
					if (data.tapes_position > objective_position) {
						//TODO: send order to MOVE BACKWARDS
					}else{
						//TODO: send order to MOVE FORWARDS
					}
				}, current_state);
			}

			state_machine.add_enter_action([&](){
				ended = true;
			}, End);
		}

		void add_on_exit_actions(){}

		void register_timed_actions(){}

		void init(){

			state_machine = initial_aux_state;

			for(uint32_t i = initial_aux_state + 1; i < (data.traction_points.size() + initial_aux_state); i++){
				state_machine.add_state(i);
				aux_last_state = i;
			}

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
