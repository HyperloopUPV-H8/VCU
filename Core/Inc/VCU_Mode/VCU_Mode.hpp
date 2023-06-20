#pragma once

namespace VCU{
    enum VCU_MODE{
        BRAKE_VALIDATION,
        POSITION_VALIDATION,
        VEHICLE,
    };

    enum LevitaionState{
		IDLE,
		TAKING_OFF,
		STABLE,
		STICK_UP,
		STICK_DOWN,
		LANDING,
	};
}
