#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{

    template<VCU_MODE> class UnloadStateMachine;

	template<>
	class UnloadStateMachine<VEHICLE>{
	static constexpr float distance_offset = 250; //250mm

	public:
		Data<VEHICLE>& data;
		Actuators<VEHICLE>& actuators;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		StateMachine state_machine;

		StackStateOrder<0> return_order;

		bool ended = false;

		static bool return_requested;

		enum LoadStates{
			Idle,
			Returning,
			Crawling,
			Braking,

		};

		UnloadStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), encoder(encoder),
			return_order(232, enter_returning, state_machine, Idle)
		{}

		static void enter_returning(){
			return_requested = true;
		}

		void add_transitions(){
			//Quitar los frenos debe hacerse manualmente con una orden una vez finalizado el procedure
			//porque segun el fdd el pod debe estar frenado hasta el momento que se va a sacar

			state_machine.add_transition(Idle, Returning, [&](){
				if(return_requested && not ended){
					return_requested = false;
					return true;
				}
				return false;
			});

			state_machine.add_transition(Returning, Crawling, [&](){
				return data.emergency_tape == PinState::ON; //Si hemos llegado a la zona de emergencia pasamos a fase crawling
			});

			state_machine.add_transition(Crawling, Braking, [&](){
				return data.emergency_tape == PinState::OFF; //Si hemos llegado al final de la zona de emergencia frenamos
			});

			state_machine.add_transition(Braking,  Idle, [&](){
				return data.tapes_acceleration == 0; //TODO: Muy importante hacer esto con la IMU del motor
			});
		}

		void add_on_enter_actions(){
			state_machine.add_enter_action([&](){
				actuators.brakes.enable_emergency_brakes();
				actuators.brakes.brake();
			}, Idle);

			state_machine.add_enter_action([&](){
				actuators.brakes.not_brake();
				actuators.brakes.disable_emergency_brakes();
				//TODO: set engine parameters
				//TODO: send start engine order
			}, Returning);

			state_machine.add_enter_action([&](){
				//Esto es por si queremos cambiar los parametros (como ir mas lento)
				//TODO: set engine parameters
			}, Crawling);

			state_machine.add_enter_action([&](){
				//TODO: send engine stop order
			}, Braking);

		}

		void add_on_exit_actions(){


			state_machine.add_exit_action([&](){
				if(data.emergency_tape == PinState::ON){
					ErrorHandler("The vehicle is still in emergency zone after Health&Unload procedure");
				}else{
					ended = true;
					actuators.brakes.brake();
				}
			}, Braking);

		}

		void register_timed_actions(){
			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_speed != 0.0){
					ErrorHandler("The vehicle is moving when it shouldn't, in the IDLE state of the Health&Unload procedure");
				}
			}, (ms)1, Crawling);

			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction != BACKWARD){
					ErrorHandler("The vehicle is moving forward when it should be moving backwards during Health&Unload procedure");
				}
			}, (ms)1, Returning);

			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction != BACKWARD){
					ErrorHandler("The vehicle is moving forward when it should be moving backwards during Health&Unload procedure");
				}
			}, (ms)1, Crawling);
		}

		void init(){
			state_machine = {Idle};
			state_machine.add_state(Returning);
			state_machine.add_state(Crawling);
			state_machine.add_state(Braking);

			add_on_enter_actions();
			add_on_exit_actions();
			add_transitions();
			register_timed_actions();
			add_transitions();

		}
	};

	bool UnloadStateMachine<VEHICLE>::return_requested = false;
}
