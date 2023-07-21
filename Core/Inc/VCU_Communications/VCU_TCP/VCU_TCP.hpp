/*
 * VCU_TCP.hpp
 *
 *  Created on: Jun 1, 2023
 *      Author: stefancostea
 */

#pragma once

#include "VCU_Pinout/Pinout.hpp"
#include "VCU_Utilities/VCU_Types.hpp"
#include "VCU_Mode/VCU_Mode.hpp"
#include "Socket.hpp"
#include "ServerSocket.hpp"

namespace VCU{
	template<VCU::VCU_MODE> class TCP;

	template<>
    class TCP<VCU::VCU_MODE::BRAKE_VALIDATION>{
    public:
        ServerSocket BACKEND_CONNECTION;

        TCP() {}

        void init(){
        	BACKEND_CONNECTION = ServerSocket(VCU_IP, SERVER_PORT, "Backend");
        }

        void send_to_backend(Order& order){
        	BACKEND_CONNECTION.send_order(order);
        }
    };
    
	template<>
	class TCP<VCU_MODE::VEHICLE>{
	public:
		ServerSocket BACKEND_CONNECTION;
		Socket OBCCU_CONNECTION;
		Socket BMSL_CONNECTION;
		Socket PCU_CONNECTION;
		Socket LCU_MASTER_CONNECTION;

		static TCP<VEHICLE>* tcp_handler;


		enum OrderIDPrefix{
		        	VCU = 200,
					LCU = 300,
					PCU = 600,
					BMSL = 800,
					OBCCU = 900
		        };

		TCP(){}

		void init(){
			BACKEND_CONNECTION = ServerSocket(VCU_IP, SERVER_PORT, "Backend");

			OBCCU_CONNECTION = Socket(VCU_IP, CLIENT_PORT, OBCCU_IP, SERVER_PORT, "OBCCU");
//			OBCCU_CONNECTION.reconnect();

			BMSL_CONNECTION = Socket(VCU_IP, CLIENT_PORT, BMSL_IP, SERVER_PORT, "BMSL");
//			BMSL_CONNECTION.reconnect();

			PCU_CONNECTION = Socket(VCU_IP, CLIENT_PORT, PCU_IP, SERVER_PORT, "PCU");
////		PCU_CONNECTION.reconnect();
//
			LCU_MASTER_CONNECTION = Socket(VCU_IP, CLIENT_PORT, LCU_MASTER_IP, SERVER_PORT, "LCU_MASTER");
			ServerSocket::set_default_parser(order_parser);
		}

        void send_to_backend(Order& order){
        	BACKEND_CONNECTION.send_order(order);
        }

        void send_to_obccu(Order& order){
        	OBCCU_CONNECTION.send_order(order);
        }

        void send_to_bsml(Order& order){
        	BMSL_CONNECTION.send_order(order);
        }

        void send_to_lcu(Order& order){
        	LCU_MASTER_CONNECTION.send_order(order);
        }

        void send_to_pcu(Order& order){
        	PCU_CONNECTION.send_order(order);
        }

        bool check_connections(){
        	return
        	BACKEND_CONNECTION.is_connected()
			&&
			OBCCU_CONNECTION.is_connected()
			&&
			BMSL_CONNECTION.is_connected()
			&&
			LCU_MASTER_CONNECTION.is_connected()
			&&
			PCU_CONNECTION.is_connected()
			;
        }

        void reconnect_all(){
			if(not OBCCU_CONNECTION.is_connected()) OBCCU_CONNECTION.reconnect();
			if(not BMSL_CONNECTION.is_connected()) BMSL_CONNECTION.reconnect();
			if(not LCU_MASTER_CONNECTION.is_connected()) LCU_MASTER_CONNECTION.reconnect();
			if(not PCU_CONNECTION.is_connected()) PCU_CONNECTION.reconnect();
		}

        static bool auxiliar_socket_send(void* data, size_t size, Socket& socket){
    		if(size > tcp_sndbuf(socket.socket_control_block)){
    			return false;
    		}
    		struct pbuf* packet = pbuf_alloc(PBUF_TRANSPORT, size, PBUF_POOL);
    		pbuf_take(packet, data, size);
    		socket.tx_packet_buffer.push(packet);
    		socket.send();
    		pbuf_free(packet);
    		return true;
        }

        static void order_parser(void* data, size_t size){
                	uint16_t order_id = Order::get_id(data);
                	if(order_id >= OrderIDPrefix::VCU && order_id <= OrderIDPrefix::LCU){
                		ErrorHandler("Cannot default parse an order for the VCU itself");
                	}else if(order_id >= OrderIDPrefix::LCU && order_id <= OrderIDPrefix::PCU){
                		auxiliar_socket_send(data, size, tcp_handler->LCU_MASTER_CONNECTION);
                	}else if(order_id >= OrderIDPrefix::PCU && order_id <= OrderIDPrefix::BMSL){
                		auxiliar_socket_send(data, size, tcp_handler->PCU_CONNECTION);
                	}else if (order_id >= OrderIDPrefix::BMSL && order_id <= OrderIDPrefix::OBCCU) {
                		auxiliar_socket_send(data, size, tcp_handler->BMSL_CONNECTION);
        			}else if (order_id >= OrderIDPrefix::OBCCU && order_id <= 1000) {
                		auxiliar_socket_send(data, size, tcp_handler->OBCCU_CONNECTION);
        			}else{
        				ErrorHandler("Cannot parse order reroute");
        			}
                }

	};

	TCP<VEHICLE>* TCP<VEHICLE>::tcp_handler = nullptr;
}
