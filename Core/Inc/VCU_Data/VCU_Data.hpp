#pragma once

#include "ST-LIB.hpp"
#include "VCU_Utilities/VCU_Includes.hpp"
#include "VCU_Mode/VCU_Mode.hpp"

namespace VCU{

    template<VCU::VCU_MODE> class Data;

    template<>
    class Data<VCU::VCU_MODE::BRAKE_VALIDATION>{
        public:
    	float regulator_real_pressure = 0.0f;
        float regulator_reference_pressure = 0.0f;

        float high_pressure1 = 0.0f;
        float low_pressure1 = 0.0f;
        float low_pressure2 = 0.0f;

        double bottle_temperature1 = 0.0f;
        double bottle_temperature2 = 0.0f;

        bool emergency_tape = false;

        REED_STATE reed = REED_STATE::RETRACTED;

        VALVE_STATE valve_state;

        void add_protections(){
			add_protection(&emergency_tape, Boundary<bool, EQUALS>(true));
			add_protection((void*)nullptr, Boundary<void, ERROR_HANDLER>());
		}
    };

    template<>
    class Data<VCU::VCU_MODE::VEHICLE>{
        public:
    	//BOARD Data
    	static constexpr float returning_speed = 5.0f;
    	static constexpr float crawling_speed = 3.0f;
		static constexpr float min_speed = 0.0f;
		static constexpr uint16_t brakes_time = 250;

    	float regulator_real_pressure = 0.0f;
        float regulator_reference_pressure = 0.0f;

        PinState emergeny_tape_enable = PinState::OFF;
        PinState emergency_tape = PinState::OFF;
        bool emergency_braking = false;

        float high_pressure1 = 0.0f;
        float low_pressure1 = 0.0f;
        float low_pressure2 = 0.0f;

        double bottle_temperature1 = 0.0f;
        double bottle_temperature2 = 0.0f;

        REED_STATE reed1 = REED_STATE::RETRACTED;
        REED_STATE reed2 = REED_STATE::RETRACTED;
        REED_STATE reed3 = REED_STATE::RETRACTED;
        REED_STATE reed4 = REED_STATE::RETRACTED;
        bool reeds_ok = true;

        VALVE_STATE valve_state = VALVE_STATE::CLOSED;
        float enviroment_temperature = 0.0f;
        float enviremont_pressure = 0.0f;

        double tapes_position = 0.0f;
        double tapes_direction = 0.0f;
        double tapes_speed = 0.0f;
        double tapes_acceleration = 0.0f;

        uint8_t* general_state;
        uint8_t* specific_state;
        uint8_t* voltage_state;

        uint8_t* load_state;
        uint8_t* unload_state;
        uint8_t* traction_state;
        uint8_t* dynamic_lev;
        uint8_t* static_lev;

        uint8_t n_g = 0;
        uint8_t n_s = 0;
        uint8_t n_v = 0;

        //VEHICLE Data
        LevitaionState levitation_state = IDLE;

        ContactorState contactors_state = ContactorState::Open;

        float engine_speed;
        float engine_acceleration;

        //Demostrations
        vector<point_t> traction_points;

        uint32_t brake_distance_lookup_table[35] = {
        		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };

        uint32_t change_direction_distance_lookup_table[35] = {
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
		   };


        bool get_next_point(point_t* p){
         	if (traction_points.empty()) {
 				return false;
 			}

         	*p = *traction_points.begin();
         	traction_points.erase(traction_points.begin());
         	return true;
         }

        void clean_traction_points(){
        	traction_points.clear();
        }

    };
}
