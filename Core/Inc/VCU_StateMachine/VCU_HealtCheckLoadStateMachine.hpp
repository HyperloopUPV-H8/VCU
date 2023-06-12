#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU_MODE> class LoadStateMachine;

	template<>
	class LoadStateMachine<VEHICLE>{
	public:
		Data<VEHICLE>& data;
		Brakes<VEHICLE>& brakes;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		StateMachine state_machine;

		enum UnloadStates{
			Idle,
			Pushing,
			Accelerating,
			Braking,
		};

		LoadStateMachine(Data<VEHICLE>& data, Brakes<VEHICLE>&brakes, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data),brakes(brakes),tcp_handler(tcp), encoder(encoder)
		{}

		void add_transitions(){
			//TODO: IDLE -> Pushing con una state orden

			//TODO: Pushing -> Accelerating con una state orden

			state_machine.add_transition(Accelerating, Braking, [&](){
				return data.emergency_tape == PinState::OFF;
			});

			state_machine.add_transition(Braking,  Idle, [&](){
				return data.tapes_acceleration == 0; //TODO: Muy importante hacer esto con la IMU
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				brakes.enable_emergency_brakes();
				brakes.brake();

			}, Idle);

			state_machine.add_enter_action([&](){
				brakes.disable_emergency_brakes();
				brakes.not_brake();
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
					brakes.brake(); //Siguiendo el FDD kenos debe quedar frenado con pneumatica hasta empezar una demostración
				}
			}, Braking);

		}

		void register_timed_actions(){
			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction < 1){ //!TODO: MUY IMPORTANTE comprobar que 1 es hacia adelante!!!!!!!!!
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
}
