#include "main.h"
#include "lwip.h"

#include "ST-LIB.hpp"
#include "Runes/Runes.hpp"
#include "VCU.hpp"
#include "VCU_Time/VCU_Time.hpp"

StateMachine* global;

enum est{
	ini,
	op,
	fa,
};

void sys_res(){
	HAL_NVIC_SystemReset();
}

void a_start(){
	global->force_change_state(op);
}

void a_fa_func(){
	global->force_change_state(fa);
}

void a_op_func(){
	global->force_change_state(op);
}

int main(void){
//{
//	VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION> vcu;
//	VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>::vcu = &vcu;
//	vcu.init();
//	VCU::CyclicActions<VCU::VCU_MODE::BRAKE_VALIDATION>::register_cyclic_actions();

	est current_state = ini;
	STLIB::start();
	ServerSocket BACKEND_CONNECTION(IPV4("192.168.1.3"), 50500);
	DatagramSocket udp(IPV4("192.168.1.3"), 50400, IPV4("192.168.0.9"),50400);

	StateOrder::set_socket(BACKEND_CONNECTION);


	StateMachine machine = {ini};
	machine.add_state(op);
	machine.add_state(fa);

	global = &machine;

	float valor = 1.0f;

	StackPacket<1, est> data_pack (
			99,
			&current_state
	);

	StackOrder<0> start{
			98,
			a_start
	};

	StackOrder<0> reset{
		97,
		sys_res
	};

	StackStateOrder<4, float> a_fa(
			101,
			a_fa_func,
			machine,
			op,
			&valor
	);

	StackStateOrder<4, float> a_op(
			102,
			a_op_func,
			machine,
			fa,
			&valor
	);

	Time::register_low_precision_alarm(100, [&](){
		machine.check_transitions();
	});

	Time::register_low_precision_alarm(10, [&](){
		current_state = (est)machine.current_state;
		//udp.send(data_pack);
	});


	while(1){
		STLIB::update();
	}
}

void Error_Handler(void)
{
	ErrorHandler("HAL error handler triggered");
	while (1){}
}
