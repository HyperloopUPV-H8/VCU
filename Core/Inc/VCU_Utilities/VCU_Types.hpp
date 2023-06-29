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

