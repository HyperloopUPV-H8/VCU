/*
 * VCU_UDP.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: stefancostea & Pablo
 */

#pragma once

#include "DatagramSocket.hpp"
#include "VCU.hpp"
#include "VCU_Utilities/VCU_Types.hpp"
#include "Packets.hpp"

namespace VCU{
	template<VCU_MODE> class UDP;

	template<>
    class UDP<BRAKE_VALIDATION> {
    public:
        DatagramSocket BACKEND_CONNECTION;
        UDP() {}

        void init(){
        	BACKEND_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, BACKEND_IP, UDP_PORT);
        	BACKEND_CONNECTION.reconnect();
        }

        void send_to_backend(Packet& packet){
        	BACKEND_CONNECTION.send(packet);
        }
    };

	template<>
    class UDP<VEHICLE> {
    public:
        DatagramSocket BACKEND_CONNECTION;
        DatagramSocket OBCCU_CONNECTION;
        DatagramSocket BMSL_CONNECTION;
        DatagramSocket PCU_CONNECTION;
        DatagramSocket LCU_MASTER_CONNECTION;

        UDP() {}

        void init(){
        	BACKEND_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, BACKEND_IP, UDP_PORT);
        	BACKEND_CONNECTION.reconnect();
//
//        	OBCCU_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, OBCCU_IP, UDP_PORT);
//        	OBCCU_CONNECTION.reconnect();
//
//        	BMSL_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, BMSL_IP, UDP_PORT);
//        	BMSL_CONNECTION.reconnect();
//
//        	LCU_MASTER_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, LCU_MASTER_IP, UDP_PORT);
//			LCU_MASTER_CONNECTION.reconnect();
//
//        	PCU_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, PCU_IP, UDP_PORT);
//        	PCU_CONNECTION.reconnect();


        }

        void send_to_backend(Packet& packet){
        	BACKEND_CONNECTION.send(packet);
        }

        void send_to_obccu(Packet& packet){
        	OBCCU_CONNECTION.send(packet);
        }

        void send_to_bmsl(Packet& packet){
        	BMSL_CONNECTION.send(packet);
        }

        void send_to_pcu(Packet& packet){
        	PCU_CONNECTION.send(packet);
        }

        void send_to_lcu(Packet& packet){
        	LCU_MASTER_CONNECTION.send(packet);
        }

    };
}
