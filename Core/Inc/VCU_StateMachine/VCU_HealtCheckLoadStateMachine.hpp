#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class LoadStateMachine;

	template<>
	class LoadStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		CloseContactorsStateMachine<VEHICLE> close_contactors_state_machine;
		OpenContactorsStateMachine<VEHICLE> open_contactors_state_machine;

		StateMachine state_machine;

		StackStateOrder<0> start_crawling;

		bool ended = false;

		static bool crawling_requested;

		enum UnloadStates{
			Idle,
			Pushing,
			CloseContactors,
			Crawling,
			Braking,
			OpenContactors,
		};

		LoadStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),
			close_contactors_state_machine(data, tcp_handler, outgoing_orders),
			open_contactors_state_machine(data, tcp_handler, outgoing_orders),
			start_crawling((uint16_t)IncomingOrdersIDs::start_crawling, enter_crawling, state_machine, Pushing)
		{}

		static void enter_crawling(){
			crawling_requested = true;
		}

		void add_transitions(){
			state_machine.add_transition(Idle, Pushing, [&](){
				if(not ended){
					return true;
				}
				return false;
			});

			state_machine.add_transition(Pushing, CloseContactors, [&](){
				//Solo se puede emepezar el crawling si se esta en zona de emergencia
				if(crawling_requested && data.emergency_tape == PinState::ON){
					crawling_requested = false;
					return true;
				}
				return false;
			});

			state_machine.add_transition(CloseContactors, Crawling, [&](){
				return close_contactors_state_machine.ended;
			});

			state_machine.add_transition(Crawling, Braking, [&](){
				return data.emergency_tape == PinState::OFF;
			});

			state_machine.add_transition(Braking,  OpenContactors, [&](){
				return data.tapes_acceleration == 0; //TODO: Muy importante hacer esto con la IMU
			});

			state_machine.add_transition(OpenContactors, Idle, [&](){
				return open_contactors_state_machine.ended;
			});
		}

		void add_on_enter_actions(){

			state_machine.add_enter_action([&](){
				actuators.brakes.disable_emergency_brakes();
				actuators.brakes.not_brake();
			}, Pushing);

			state_machine.add_enter_action([&](){
				//TODO: set engine parameters
				//TODO: send start engine order
			}, Crawling);

			state_machine.add_enter_action([&](){
				//Quizas hace falta darle un offset (avanzar un poco más) para evitar que entre
				//en fault si se mueve ligeremante hacia atras y ha quedado demasido cerca de la emergency tape.
				//TODO: send engine brake order
			}, Braking);

			state_machine.add_enter_action([&](){
				actuators.brakes.enable_emergency_brakes();
				actuators.brakes.brake();
			}, OpenContactors);
		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
				if(data.emergency_tape == PinState::ON){
					ErrorHandler("The vehicle is still in emergency zone after Health&Load procedure");
				}else{
					actuators.brakes.brake();
					 //Siguiendo el FDD kenos debe quedar frenado con pneumatica hasta empezar una demostración
				}
			}, Braking);

			state_machine.add_exit_action([&](){
				ended = true;
			}, OpenContactors);

		}

		void register_timed_actions(){
			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction != FORWARD){
					ErrorHandler("The vehicle is moving backwards when it should be moving forward during load procedure");
				}
			}, (ms)1, Crawling);

		}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(CloseContactors);
			state_machine.add_state(Pushing);
			state_machine.add_state(Crawling);
			state_machine.add_state(Braking);
			state_machine.add_state(OpenContactors);

			state_machine.add_state_machine(close_contactors_state_machine.state_machine, CloseContactors);
			state_machine.add_state_machine(open_contactors_state_machine.state_machine, OpenContactors);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};

	bool LoadStateMachine<VEHICLE>::crawling_requested = false;
}
