#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class DynamicLevStateMachine;

	template<>
	class DynamicLevStateMachine<VEHICLE>{
		static constexpr uint32_t levitation_timeout = 6000;//ms
		static constexpr uint32_t landing_timeout = 3000;//ms
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		CloseContactorsStateMachine<VEHICLE> close_contactors_state_machine;
		OpenContactorsStateMachine<VEHICLE> open_contactors_state_machine;
		ReceivingDataStateMachine<VEHICLE> receiving_data_state_machine;
		PointTravelStateMachine<VEHICLE> point_travel_state_machine;

		StackStateOrder<8, uint32_t, float> traction_data;
		StackStateOrder<0> traction_end_data;

		StateMachine state_machine;

		bool ended = false;

		enum DynamicLevStates{
			Idle,
			ReceivingData,
			CloseContactors,
			TakeOff,
			PointTravel,
			Landing,
			OpenContactors,
		};

		DynamicLevStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),
			close_contactors_state_machine(data, tcp_handler, outgoing_orders),
			open_contactors_state_machine(data, tcp_handler, outgoing_orders),
			receiving_data_state_machine(data, tcp_handler),
			point_travel_state_machine(data, actuators, tcp_handler, outgoing_orders),

			traction_data((uint16_t)IncomingOrdersIDs::traction_data, ReceivingDataStateMachine<VEHICLE>::add_traction_point, receiving_data_state_machine.state_machine, ReceivingDataStateMachine<VEHICLE>::ReceivingDataStates::ReceivingData, &ReceivingDataStateMachine<VEHICLE>::received_data_distance, &ReceivingDataStateMachine<VEHICLE>::received_data_speed),
			traction_end_data((uint16_t)IncomingOrdersIDs::traction_end_data, ReceivingDataStateMachine<VEHICLE>::last_data_packet, receiving_data_state_machine.state_machine, ReceivingDataStateMachine<VEHICLE>::ReceivingDataStates::ReceivingData)
		{
			init();
		}

		void add_transitions(){
			state_machine.add_transition(Idle, ReceivingData, [&](){
				if(not ended){
					return true;
				}
				return false;
			});

			state_machine.add_transition(ReceivingData, CloseContactors, [&](){
				return receiving_data_state_machine.ended;
			});

			state_machine.add_transition(CloseContactors, TakeOff, [&](){
				return close_contactors_state_machine.ended;
			});

			state_machine.add_transition(TakeOff, PointTravel, [&](){
				return data.levitation_state == LevitaionState::STABLE;
			});

			state_machine.add_transition(PointTravel, Landing, [&](){
				return point_travel_state_machine.ended;
			});

			state_machine.add_transition(Landing, OpenContactors, [&](){
				return data.levitation_state == LevitaionState::IDLE;
			});

			state_machine.add_transition(OpenContactors, Idle, [&](){
				return open_contactors_state_machine.ended;
			});

		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				data.clean_traction_points();
	        	ReceivingDataStateMachine<VEHICLE>::clean_traction_points();
			}, Idle);

			state_machine.add_enter_action([&](){
				actuators.brakes.not_brake();
				actuators.brakes.enable_emergency_brakes();
			}, CloseContactors);

			state_machine.add_enter_action([&](){
				tcp_handler.send_to_lcu(outgoing_orders.take_off_order);

				Time::set_timeout(levitation_timeout, [&](){
					if(state_machine.current_state == TakeOff && data.levitation_state != STABLE){
						ErrorHandler("Kenos has not been able to stabilize in time, maximum timeout: %dms", levitation_timeout);
					}
				});
			}, TakeOff);

			state_machine.add_enter_action([&](){
				tcp_handler.send_to_lcu(outgoing_orders.landing_order);

				Time::set_timeout(landing_timeout, [&](){
					if(state_machine.current_state == Landing && data.levitation_state != IDLE){
						ErrorHandler("Kenos failed to land on time, maximum timeout: %dms", landing_timeout);
					}
				});
			}, Landing);

			state_machine.add_enter_action([&](){
				actuators.brakes.brake();
				actuators.brakes.enable_emergency_brakes();
			}, OpenContactors);
		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
				close_contactors_state_machine.ended = false;
			}, CloseContactors);

			state_machine.add_exit_action([&](){
				receiving_data_state_machine.ended = false;
			}, ReceivingData);

			state_machine.add_exit_action([&](){
				point_travel_state_machine.ended = false;
			}, PointTravel);

			state_machine.add_exit_action([&](){
				open_contactors_state_machine.ended = false;
				ended = true;
			}, OpenContactors);
		}

		void register_timed_actions(){}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(ReceivingData);
			state_machine.add_state(CloseContactors);
			state_machine.add_state(TakeOff);
			state_machine.add_state(PointTravel);
			state_machine.add_state(Landing);
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
