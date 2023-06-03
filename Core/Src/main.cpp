#include "main.h"
#include "lwip.h"

#include "ST-LIB.hpp"
#include "Runes/Runes.hpp"
#include "VCU.hpp"
#include "VCU_Time/VCU_Time.hpp"

int main(void)
{
	VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION> vcu;
	VCU::VCU_CLASS<VCU::VCU_MODE::BRAKE_VALIDATION>::vcu = &vcu;
	vcu.init();
	VCU::CyclicActions<VCU::VCU_MODE::BRAKE_VALIDATION>::register_cyclic_actions();
	while(1){
		STLIB::update();
	}
}

void Error_Handler(void)
{
	ErrorHandler("HAL error handler triggered");
	while (1){}
}
