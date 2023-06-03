#pragma once

#include "ST-LIB.hpp"
#include "VCU_Actuators/VCU_ValveActuator.hpp"
#include "VCU_Actuators/VCU_RegulatorActuator.hpp"
#include "VCU_Sensors/VCU_RegulatorSensor.hpp"
#include "VCU_Sensors/VCU_Reed.hpp"
#include "VCU_Data/VCU_Data.hpp"
#include "VCU_Pinout/Pinout.hpp"


namespace VCU{
    template<VCU::VCU_MODE> class Brakes;

    template<>
    class Brakes<VCU::VCU_MODE::BRAKE_VALIDATION>{
        constexpr static uint16_t ntc_lookup_table_size = 256;

        constexpr static float high_pressure_sensor_slope = 113.46153*1.20822977;
        constexpr static float high_pressure_sensor_offset = (-516.25/11.8)*1.20822977;

        constexpr static float low_pressure_sensors_slope = 0.000190905;
        constexpr static float low_pressure_sensors_offset = -1.25;

        constexpr static float operating_pressure = 8.0f;

        ValveActuator valve_actuator;
        RegulatorActuator regulator_actuator;
        RegulatorSensor regulator_sensor;

        DigitalOutput emergency_tape_enable;

        Reed reed;

        double ntc_lookup_table[ntc_lookup_table_size];

        LookupSensor temperature_sensor1;
        LookupSensor temperature_sensor2;

        LinearSensor<float> high_pressure_sensor;
        LinearSensor<float> low_pressure_sensor1;
        LinearSensor<float> low_pressure_sensor2;

    public:
        Brakes(Data<VCU_MODE::BRAKE_VALIDATION>& data) : valve_actuator(Pinout::VALVE, data.valve_state),
        regulator_actuator(Pinout::REGULATOR_OUT, data.regulator_reference_pressure),
        regulator_sensor(Pinout::REGULATOR_IN, data.regulator_real_pressure),

        emergency_tape_enable(Pinout::EMERGENCY_TAPE_ENABLE),

        reed(Pinout::REED1, &data.reed),

        temperature_sensor1(Pinout::BOTTLE_TEMP1, ntc_lookup_table, ntc_lookup_table_size, &data.bottle_temperature1),
        temperature_sensor2(Pinout::BOTTLE_TEMP2, ntc_lookup_table, ntc_lookup_table_size, &data.bottle_temperature2),

        high_pressure_sensor(Pinout::HIGH_PRESSURE, high_pressure_sensor_slope, high_pressure_sensor_offset, &data.high_pressure1),
        low_pressure_sensor1(Pinout::LOW_PRESSURE1, low_pressure_sensors_slope, low_pressure_sensors_offset, &data.low_pressure1),
        low_pressure_sensor2(Pinout::LOW_PRESSURE2, low_pressure_sensors_slope, low_pressure_sensors_offset, &data.low_pressure2)
        {

        }


        void read(){
            regulator_sensor.read();

            temperature_sensor1.read();
            temperature_sensor2.read();

            high_pressure_sensor.read();
            low_pressure_sensor1.read();
            low_pressure_sensor2.read();

            reed.read();
        }

        void brake(){
            valve_actuator.close();
        }

        void not_brake(){
            valve_actuator.open();
        }

        void disable_emergency_brakes(){
            emergency_tape_enable.turn_on();
        }

        void enable_emergency_brakes(){
            emergency_tape_enable.turn_off();
        }



        void check_reeds(){
        	reed.read();
        }

        void init(){
        	regulator_actuator.init();
            read();
            brake();
            regulator_actuator.set_pressure(operating_pressure);
            read();
        }

        void set_regulator_pressure(float new_pressure){
            regulator_actuator.set_pressure(new_pressure);
        }


    };


    template<>
    class Brakes<VCU::VCU_MODE::VEHICLE>{
        constexpr static uint16_t ntc_lookup_table_size = 256;

        constexpr static float high_pressure_sensor_slope = 0.006681691;
        constexpr static float high_pressure_sensor_offset = -43.75;

        constexpr static float low_pressure_sensors_slope = 0.000190905;
        constexpr static float low_pressure_sensors_offset = -1.25;
      

        constexpr static float operating_pressure = 8.0f;

        private:
            Data<VCU::VCU_MODE::VEHICLE>& data;

            ValveActuator valve_actuator;
            RegulatorActuator regulator_actuator;
            RegulatorSensor regulator_sensor;

            SensorInterrupt emergency_tape; 
            DigitalOutput emergency_tape_enable;
            
            Reed reed1;
            Reed reed2;
            Reed reed3;
            Reed reed4;

            double ntc_lookup_table[ntc_lookup_table_size];

            LookupSensor temperature_sensor1;
            LookupSensor temperature_sensor2;

            LinearSensor<float> high_pressure_sensor;
            LinearSensor<float> low_pressure_sensor1;
            LinearSensor<float> low_pressure_sensor2;

        public:
            Brakes(Data<VCU::VCU_MODE::VEHICLE>& data):
                data(data),

                valve_actuator(Pinout::VALVE, data.valve_state),
                regulator_actuator(Pinout::REGULATOR_OUT, data.regulator_reference_pressure),
                regulator_sensor(Pinout::REGULATOR_IN, data.regulator_real_pressure),

                emergency_tape(Pinout::EMERGENCY_TAPE, [&](){emergency_tape.read();}, data.emergency_tape),
                emergency_tape_enable(Pinout::EMERGENCY_TAPE_ENABLE),

                reed1(Pinout::REED1, &data.reed1),
                reed2(Pinout::REED2, &data.reed2),
                reed3(Pinout::REED3, &data.reed3),
                reed4(Pinout::REED4, &data.reed4),

                temperature_sensor1(Pinout::BOTTLE_TEMP1, ntc_lookup_table, ntc_lookup_table_size, &data.bottle_temperature1),
                temperature_sensor2(Pinout::BOTTLE_TEMP2, ntc_lookup_table, ntc_lookup_table_size, &data.bottle_temperature2),
               
                high_pressure_sensor(Pinout::HIGH_PRESSURE, high_pressure_sensor_slope, high_pressure_sensor_offset, &data.high_pressure1),
                low_pressure_sensor1(Pinout::LOW_PRESSURE1, low_pressure_sensors_slope, low_pressure_sensors_offset, &data.low_pressure1),
                low_pressure_sensor2(Pinout::LOW_PRESSURE2, low_pressure_sensors_slope, low_pressure_sensors_offset, &data.low_pressure2)
            {}

            void read(){
                regulator_sensor.read();
                
                emergency_tape.read();

                temperature_sensor1.read();
                temperature_sensor2.read(); 

                high_pressure_sensor.read();
                low_pressure_sensor1.read();
                low_pressure_sensor2.read();
            }

            void brake(){
                valve_actuator.close();
                

                Time::set_timeout(1, [&](){
                    check_reeds();
                });
            }

            void not_brake(){
                valve_actuator.open();

                Time::set_timeout(1, [&](){
                    check_reeds();
                });
            }

            void disable_emergency_brakes(){
                emergency_tape_enable.turn_on();
                data.emergeny_tape_enable = PinState::ON;
            }

            void enable_emergency_brakes(){
                emergency_tape_enable.turn_off();
                data.emergeny_tape_enable = PinState::OFF;
            }

            void check_reeds(){
                reed1.read();
                reed2.read();
                reed3.read();
                reed4.read();
                data.reeds_ok = ((data.reed1 == data.reed2) == data.reed3) == data.reed4;
            }

            void init(){
                read();
                enable_emergency_brakes();
                brake();
                regulator_actuator.set_pressure(operating_pressure);
                read();
            }

    };
}
