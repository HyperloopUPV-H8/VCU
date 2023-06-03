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
        void send_to_master(Order& order){
        	BACKEND_CONNECTION.send_order(order);
        }
    };
    
}
