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
		EncoderSensor& encoder;
		StateMachine state_machine;

		StackStateOrder<0> start_pushing;
		StackStateOrder<0> start_crawling;

		bool ended = false;

		static bool pushing_requested;
		static bool crawling_requested;

		enum UnloadStates{
			Idle,
			Pushing,
			Accelerating,
			Braking,
		};

		LoadStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), encoder(encoder),
			start_pushing(230, enter_pushing, state_machine, Idle),
			start_crawling(231, enter_crawling, state_machine, Pushing)
		{}

		static void enter_pushing(){
			pushing_requested = true;
		}

		static void enter_crawling(){
			crawling_requested = true;
		}

		void add_transitions(){
			state_machine.add_transition(Idle, Pushing, [&](){
				if(pushing_requested && not ended){
					pushing_requested = false;
					return true;
				}
				return false;
			});

			state_machine.add_transition(Pushing, Accelerating, [&](){
				//Solo se puede emepezar el crawling si se esta en zona de emergencia
				if(crawling_requested && data.emergency_tape == PinState::ON){
					crawling_requested = false;
					return true;
				}
				return false;
			});

			state_machine.add_transition(Accelerating, Braking, [&](){
				return data.emergency_tape == PinState::OFF;
			});

			state_machine.add_transition(Braking,  Idle, [&](){
				return data.tapes_acceleration == 0; //TODO: Muy importante hacer esto con la IMU
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				actuators.brakes.enable_emergency_brakes();
				actuators.brakes.brake();
			}, Idle);

			state_machine.add_enter_action([&](){
				actuators.brakes.disable_emergency_brakes();
				actuators.brakes.not_brake();
			}, Pushing);

			state_machine.add_enter_action([&](){
				//TODO: set engine parameters
				//TODO: send start engine order
			}, Accelerating);

			state_machine.add_enter_action([&](){
				//Quizas hace falta darle un offset (avanzar un poco más) para evitar que entre
				//en fault si se mueve ligeremante hacia atras y ha quedado demasido cerca de la emergency tape.
				//TODO: send engine brake order
			}, Braking);
		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
				if(data.emergency_tape == PinState::ON){
					ErrorHandler("The vehicle is still in emergency zone after Health&Load procedure");
				}else{
					ended = true;
					actuators.brakes.brake();
					 //Siguiendo el FDD kenos debe quedar frenado con pneumatica hasta empezar una demostración
				}
			}, Braking);

		}

		void register_timed_actions(){
			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction != FORWARD){
					ErrorHandler("The vehicle is moving backwards when it should be moving forward during load procedure");
				}
			}, (ms)1, Accelerating);

		}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Pushing);
			state_machine.add_state(Accelerating);
			state_machine.add_state(Braking);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();
		}

	};

	bool LoadStateMachine<VEHICLE>::crawling_requested = false;
	bool LoadStateMachine<VEHICLE>::pushing_requested = false;
}
