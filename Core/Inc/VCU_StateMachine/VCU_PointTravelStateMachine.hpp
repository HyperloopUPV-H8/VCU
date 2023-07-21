#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class PointTravelStateMachine;

	template<>
	class PointTravelStateMachine<VEHICLE>{

	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE> actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;

		StateMachine state_machine;

		enum TravelStates{
			Idle,
			Calculating,
			MovingForward,
			MovingBackward,
			Braking,
			End
		};

		bool ended = false;

		point_t initial_point;
		point_t* calculated_point = nullptr;
		DIRECTION calculated_direction = DIRECTION::FORWARD;
		bool calculating = true;

		PointTravelStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE> actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders) :
			data(data), actuators(actuators), tcp_handler(tcp), outgoing_orders(outgoing_orders)
		{
			init();
		}

		void add_transitions(){
			state_machine.add_transition(Idle, Calculating, [&](){
				return not ended;
			});

			state_machine.add_transition(Calculating, MovingForward, [&](){
				return not calculating && calculated_direction == DIRECTION::FORWARD;
			});

			state_machine.add_transition(Calculating, MovingBackward, [&](){
				return not calculating && calculated_direction == DIRECTION::BACKWARD;
			});

			state_machine.add_transition(MovingForward, Calculating, [&](){
				if (data.tapes_position >= calculated_point->position - data.change_direction_distance_lookup_table[(uint32_t)ceil(data.engine_speed)]) {
					return true;
				}

				return false;
			});

			state_machine.add_transition(MovingBackward, Calculating, [&](){
				if (data.tapes_position <= calculated_point->position + data.change_direction_distance_lookup_table[(uint32_t)ceil(data.engine_speed)]) {
					return true;
				}

				return false;
			});

			state_machine.add_transition(Braking, End, [&](){
				return data.engine_speed <= Data<VEHICLE>::min_speed or data.tapes_position <= initial_point.position;
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				initial_point = point_t(data.tapes_position, 0.0f);
				data.traction_points.push_back(initial_point);
			}, Idle);

			state_machine.add_enter_action([&](){
				calculating = true;
				if(not data.get_next_point(calculated_point)){

					state_machine.force_change_state(Braking);
					return;
				}

				if (calculated_point->position > data.tapes_position ) {
					calculated_direction = DIRECTION::FORWARD;
				}else{
					calculated_direction = DIRECTION::BACKWARD;
				}

				calculating = false;

			}, Calculating);

			state_machine.add_enter_action([&](){
				calculating = false;

//				outgoing_orders.speed = calculated_point->speed;
//				outgoing_orders.direction = calculated_direction;
//				tcp_handler.send_to_pcu(outgoing_orders.move);

			}, MovingForward);

			state_machine.add_enter_action([&](){
				calculating = false;

//				outgoing_orders.speed = calculated_point->speed;
//				tcp_handler.send_to_pcu(outgoing_orders.move);

			}, MovingBackward);

			state_machine.add_enter_action([&](){
//				tcp_handler.send_to_pcu(outgoing_orders.brake);
			}, Braking);

			state_machine.add_enter_action([&](){
//				tcp_handler.send_to_pcu(outgoing_orders.turn_off);
				ended = true;
				actuators.brakes.brake();
			}, End);
		}

		void add_on_exit_actions(){

		}

		void register_timed_actions(){


		}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Calculating);
			state_machine.add_state(MovingForward);
			state_machine.add_state(MovingBackward);
			state_machine.add_state(Braking);
			state_machine.add_state(End);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};
}
