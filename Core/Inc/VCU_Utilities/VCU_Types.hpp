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

}

