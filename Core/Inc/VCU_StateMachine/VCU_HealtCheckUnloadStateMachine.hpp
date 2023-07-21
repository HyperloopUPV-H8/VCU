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
		OutgoingOrders<VEHICLE>& outgoing_orders;
		EncoderSensor& encoder;

		CloseContactorsStateMachine<VEHICLE> close_contactors_state_machine;
		OpenContactorsStateMachine<VEHICLE> open_contactors_state_machine;

		StateMachine state_machine;

		bool ended = false;

		enum LoadStates{
			Idle,
			CloseContactors,
			Returning,
			Crawling,
			Braking,
			OpenContactors,
		};

		UnloadStateMachine(Data<VEHICLE>& data, Actuators<VEHICLE>& actuators, TCP<VEHICLE>& tcp, OutgoingOrders<VEHICLE>& outgoing_orders, EncoderSensor& encoder) :
			data(data), actuators(actuators), tcp_handler(tcp), outgoing_orders(outgoing_orders), encoder(encoder),
			close_contactors_state_machine(data, tcp_handler, outgoing_orders),
			open_contactors_state_machine(data, tcp_handler, outgoing_orders)
		{
			init();
		}

		void add_transitions(){
			//INFO: Quitar los frenos debe hacerse manualmente con una orden una vez finalizado el procedure
			//porque segun el fdd el pod debe estar frenado hasta el momento que se va a sacar

			state_machine.add_transition(Idle, CloseContactors, [&](){
				if(not ended){
					return true;
				}

				return false;
			});

			state_machine.add_transition(CloseContactors, Returning, [&](){
				return close_contactors_state_machine.ended;
			});

			state_machine.add_transition(Returning, Crawling, [&](){
				//INFO: Si hemos llegado a la zona de emergencia pasamos a fase crawling
				return data.emergency_tape == PinState::ON;
			});

			state_machine.add_transition(Crawling, Braking, [&](){
				 //INFO: Si hemos llegado al final de la zona de emergencia frenamos
				return data.emergency_tape == PinState::OFF;
			});

			state_machine.add_transition(Braking, OpenContactors, [&](){
				return data.engine_speed <= Data<VEHICLE>::min_speed;
			});

			state_machine.add_transition(OpenContactors,  Idle, [&](){
				return open_contactors_state_machine.ended;
			});
		}

		void add_on_enter_actions(){

			state_machine.add_enter_action([&](){
				actuators.brakes.not_brake();
				actuators.brakes.disable_emergency_brakes();

				Time::set_timeout(Data<VEHICLE>::brakes_time, [&](){
//					outgoing_orders.speed = Data<VEHICLE>::returning_speed;
//					tcp_handler.send_to_pcu(outgoing_orders.move);
				});
			}, Returning);

			state_machine.add_enter_action([&](){
//				outgoing_orders.speed = Data<VEHICLE>::crawling_speed;
//				outgoing_orders.direction = VCU::DIRECTION::BACKWARD;
//				tcp_handler.send_to_pcu(outgoing_orders.move);
			}, Crawling);

			state_machine.add_enter_action([&](){
//				tcp_handler.send_to_pcu(outgoing_orders.brake);
			}, Braking);

			state_machine.add_enter_action([&](){
				actuators.brakes.enable_emergency_brakes();
				actuators.brakes.brake();
			}, OpenContactors);
		}

		void add_on_exit_actions(){
			state_machine.add_exit_action([&](){
//				tcp_handler.send_to_pcu(outgoing_orders.turn_off);

				if(data.emergency_tape == PinState::ON){
					ErrorHandler("The vehicle is still in emergency zone after Health&Unload procedure");
				}
			}, Braking);

			state_machine.add_exit_action([&](){
				close_contactors_state_machine.ended = false;
			}, CloseContactors);

			state_machine.add_exit_action([&](){
				open_contactors_state_machine.ended = false;
				ended = true;
			}, OpenContactors);

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
			state_machine.add_state(CloseContactors);
			state_machine.add_state(Returning);
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

			data.unload_state = &state_machine.current_state;
		}
	};
}
