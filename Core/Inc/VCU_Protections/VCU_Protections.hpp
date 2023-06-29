#pragma once

#include "VCU_Utilities/VCU_Includes.hpp"

namespace VCU{
	template<VCU::VCU_MODE> class Protections;

    template<>
	class Protections<VCU::VCU_MODE::VEHICLE>{
		Data<VEHICLE>& data;

		Protections(Data<VEHICLE> data): data(data){
			add_protection(&data.high_pressure1, Boundary<float, OUT_OF_RANGE>(100, 300));
			add_protection(&data.low_pressure1, Boundary<float, OUT_OF_RANGE>(0, 10));
			add_protection(&data.low_pressure2, Boundary<float, OUT_OF_RANGE>(0, 10));

			add_protection(&data.regulator_real_pressure, Boundary<float, OUT_OF_RANGE>(0, 10));

			add_protection(&data.bottle_temperature1, Boundary<float, OUT_OF_RANGE>(0, 50));
			add_protection(&data.bottle_temperature2, Boundary<float, OUT_OF_RANGE>(0, 50));

			add_protection(&data.enviremont_pressure, Boundary<float, OUT_OF_RANGE>(0, 1.2f));
			add_protection(&data.enviroment_temperature, Boundary<float, OUT_OF_RANGE>(0, 50));

//			add_protection(&data.reeds_ok, Boundary<bool, NOT_EQUALS>(true));
			add_protection(&data.emergency_braking, Boundary<bool, EQUALS>(true));
		}
	};

}
