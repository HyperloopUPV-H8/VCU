#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{

    template<VCU_MODE> class UnloadStateMachine;

	template<>
	class UnloadStateMachine<VEHICLE>{
	static constexpr float distance_offset = 250; //250mm

	public:
		Data<VEHICLE>& data;
		Brakes<VEHICLE>& brakes;
		TCP<VEHICLE>& tcp_handler;
		EncoderSensor& encoder;
		StateMachine state_machine;

		enum LoadStates{
			Idle,
			Returning,
			Crawling,
			Braking,

		};

		UnloadStateMachine(Data<VEHICLE>& data, Brakes<VEHICLE>&brakes, TCP<VEHICLE>& tcp, EncoderSensor& encoder) :
			data(data),brakes(brakes),tcp_handler(tcp), encoder(encoder)
		{}

		void add_transitions(){
			//TODO: IDLE -> Returning con una state orden
			//Quitar los frenos debe hacerse manualmente con una orden una vez finalizado el procedure
			//porque segun el fdd el pod debe estar frenado hasta el momento que se va a sacar

			state_machine.add_transition(Returning, Crawling, [&](){
				return data.emergency_tape == PinState::ON; //Si hemos llegado a la zona de emergencia pasamos a fase crawling
			});

			state_machine.add_transition(Crawling, Braking, [&](){
				return data.emergency_tape == PinState::OFF; //Si hemos llegado al final de la zona de emergencia frenamos
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
				brakes.not_brake();
				brakes.disable_emergency_brakes();
				//TODO: set engine parameters
				//TODO: send start engine order
			}, Returning);

			state_machine.add_enter_action([&](){
				//Esto es por si queremos cambiar los parametros (como ir mas lento)
				//TODO: set engine parameters
			}, Crawling);

			state_machine.add_enter_action([&](){
				//TODO: send engine stop order
				brakes.brake();
			}, Braking);

		}

		void add_on_exit_actions(){}

		void register_timed_actions(){
			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_speed != 0.0){
					ErrorHandler("The vehicle is moving when it shouldn't, in the IDLE state of the Health&Unload procedure");
				}
			}, (ms)1, Crawling);

			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction > 0){ //!TODO: MUY IMPORTANTE comprobar que01 es hacia atras!!!!!!!!!
					ErrorHandler("The vehicle is moving forward when it should be moving backwards during Health&Unload procedure");
				}
			}, (ms)1, Returning);

			state_machine.add_low_precision_cyclic_action([&](){
				if(data.tapes_direction > 0){ //!TODO: MUY IMPORTANTE comprobar que01 es hacia atras!!!!!!!!!
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
}
