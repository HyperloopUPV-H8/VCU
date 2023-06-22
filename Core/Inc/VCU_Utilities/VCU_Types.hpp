#pragma once

namespace VCU{

    enum VALVE_STATE{
        CLOSED,
        OPEN,
    };

    enum REED_STATE{
    	EXTENDED,
        RETRACTED,
    };

    enum DIRECTION{ //TODO: Comprobar que esto es cierto
    	BACKWARD = 0,
		FORWARD = 1,
    };

    enum ContactorState{
    	Open,
		Close,
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

