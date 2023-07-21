#include "main.h"
#include "lwip.h"

#include "ST-LIB.hpp"
#include "Runes/Runes.hpp"
#include "VCU.hpp"
#include "VCU_Time/VCU_Time.hpp"

int main(void){
	for(uint32_t i = 0; i < 10000000; i++){
		__NOP();
	}

	if (not __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) HAL_NVIC_SystemReset();
		static_assert(HSE_VALUE == 25000000UL);
	#ifndef BOARD
		static_assert(false, "BOARD has to be declared as a symbol");
	#endif

	VCU::VCU_CLASS<VCU::VCU_MODE::VEHICLE> vcu;
	VCU::VCU_CLASS<VCU::VCU_MODE::VEHICLE>::vcu = &vcu;
	vcu.init();
	VCU::CyclicActions<VCU::VCU_MODE::VEHICLE>::register_cyclic_actions();

	GPIO_PinState a = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
	while(1){
//		STLIB::update();
		vcu.read_encoder();
		if(ErrorHandlerModel::error_triggered != 0){
			vcu.actuators.led_can.turn_on();
			vcu.actuators.led_fault.turn_on();
			vcu.actuators.led_flash.turn_on();
			vcu.actuators.led_operational.turn_on();
			vcu.actuators.led_sleep.turn_on();
		}
	}
}

void Error_Handler(void)
{
	ErrorHandler("HAL error handler triggered");
	while (1){}
}
