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

    enum DIRECTION{
    	FORWARD = 0,
		BACKWARD = 1,
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

    enum COIL_ID {
		HEMS_1,
		HEMS_2,
		HEMS_3,
		HEMS_4,
		EMS_1,
		EMS_2,
		EMS_3,
		EMS_4
	};

    struct point_t{
    	uint32_t position;
    	float speed;

    	point_t(){
    		position = 0;
    		speed = 0.0f;
    	}

    	point_t(uint32_t position, float speed){
    		this->position = position;
    		this->speed = speed;
    	}

    	point_t& operator=(const point_t& other){
    		this->position = other.position;
    		this->speed = other.speed;

    		return *this;
    	}
    };


}

