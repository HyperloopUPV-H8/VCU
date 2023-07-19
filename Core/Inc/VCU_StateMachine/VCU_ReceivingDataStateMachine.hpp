#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class ReceivingDataStateMachine;

	template<>
	class ReceivingDataStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		TCP<VEHICLE>& tcp_handler;
		StateMachine state_machine;

		bool ended = false;

        static vector<point_t> traction_points;
		static bool last_data_packet_requested;
		static uint32_t received_data_distance;
		static float received_data_speed;

		enum ReceivingDataStates{
			Idle,
			ReceivingData,
			End,
		};

        static void add_traction_point(){
        	traction_points.push_back(point_t(received_data_distance, received_data_speed));
        }

        static void clean_traction_points(){
        	traction_points.clear();
        }

		static void last_data_packet(){
			last_data_packet_requested = true;
		}

		ReceivingDataStateMachine(Data<VEHICLE>& data, TCP<VEHICLE>& tcp) :
			data(data), tcp_handler(tcp)
		{
			init();
		}

		void add_transitions(){
			state_machine.add_transition(Idle, ReceivingData, [&](){
				return not ended;
			});

			state_machine.add_transition(ReceivingData, End, [&](){
				return last_data_packet_requested;
			});

		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				data.traction_points = traction_points;
				last_data_packet_requested = false;
			}, End);

		}

		void add_on_exit_actions(){}

		void register_timed_actions(){
		}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(ReceivingData);
			state_machine.add_state(End);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};

	vector<point_t> ReceivingDataStateMachine<VEHICLE>::traction_points = {};
	bool ReceivingDataStateMachine<VEHICLE>::last_data_packet_requested = false;
	uint32_t ReceivingDataStateMachine<VEHICLE>::received_data_distance = 0;
	float ReceivingDataStateMachine<VEHICLE>::received_data_speed = 0.0f;
}


