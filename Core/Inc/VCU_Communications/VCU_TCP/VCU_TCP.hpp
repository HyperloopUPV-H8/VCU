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
        	BACKEND_CONNECTION = ServerSocket(VCU_IP, SERVER_PORT);
        }
        void send_to_backend(Order& order){
        	BACKEND_CONNECTION.send_order(order);
        }
    };
    
	template<>
	class TCP<VCU_MODE::VEHICLE>{
		ServerSocket BACKEND_CONNECTION;
		Socket OBCCU_CONNECTION;
		Socket BMSL_CONNECTION;
		Socket PCU_CONNECTION;
		Socket LCU_MASTER_CONNECTION;
		TCP(){}
		void init(){
			BACKEND_CONNECTION = ServerSocket(VCU_IP, SERVER_PORT);
			OBCCU_CONNECTION = Socket(VCU_IP, CLIENT_PORT, OBCCU_IP, SERVER_PORT);
			OBCCU_CONNECTION.reconnect();
			BMSL_CONNECTION = Socket(VCU_IP, CLIENT_PORT, BMSL_IP, SERVER_PORT);
			BMSL_CONNECTION.reconnect();
			PCU_CONNECTION = Socket(VCU_IP, CLIENT_PORT, PCU_IP, SERVER_PORT);
			PCU_CONNECTION.reconnect();
			LCU_MASTER_CONNECTION = Socket(VCU_IP, CLIENT_PORT, LCU_MASTER_IP, SERVER_PORT);
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
	};
}
