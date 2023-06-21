#pragma once
#include "Time/Time.hpp"
#include "VCU_Sensors/VCU_EnviromentalSensors.hpp"
#include "VCU_Sensors/VCU_Reed.hpp"
#include "VCU_Sensors/VCU_RegulatorSensor.hpp"
#include "VCU_Brakes/VCU_Brakes.hpp"
#include "VCU.hpp"

namespace VCU{

	template<VCU_MODE> class CyclicActions;

	template<> class CyclicActions<BRAKE_VALIDATION>{
	public:
		Brakes<VCU_MODE::BRAKE_VALIDATION>& brakes;
		CyclicActions(Brakes<VCU_MODE::BRAKE_VALIDATION>& brakes) : brakes(brakes){}

		static void register_cyclic_actions(){
			Time::register_low_precision_alarm(1, VCU::VCU_CLASS<BRAKE_VALIDATION>::read_brakes_sensors);
			Time::register_low_precision_alarm(16, VCU::VCU_CLASS<BRAKE_VALIDATION>::send_to_backend);
			Time::register_low_precision_alarm(1, VCU::VCU_CLASS<BRAKE_VALIDATION>::update_state_machine);
		}
	};

	template<> class CyclicActions<VEHICLE>{
	public:
		Brakes<VCU_MODE::VEHICLE>& brakes;
		CyclicActions(Brakes<VCU_MODE::VEHICLE>& brakes) : brakes(brakes){}

		static void register_cyclic_actions(){
			Time::register_low_precision_alarm(1, VCU::VCU_CLASS<VEHICLE>::read_brakes_sensors);
			Time::register_low_precision_alarm(100, VCU::VCU_CLASS<VEHICLE>::read_environmental_sensors);
			Time::register_low_precision_alarm(16, VCU::VCU_CLASS<VEHICLE>::send_to_backend);
			Time::register_low_precision_alarm(1, VCU::VCU_CLASS<VEHICLE>::update_state_machine);
		}
	};
}
