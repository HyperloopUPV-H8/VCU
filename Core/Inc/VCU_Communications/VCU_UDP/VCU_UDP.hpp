#pragma once
#include "DatagramSocket.hpp"
#include "VCU.hpp"
#include "VCU_Utilities/VCU_Types.hpp"

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
		UDP() {}
		void init(){
			//TODO: Conectarse a todas las placas
			BACKEND_CONNECTION = DatagramSocket(VCU_IP, UDP_PORT, BACKEND_IP, UDP_PORT);
			BACKEND_CONNECTION.reconnect();
		}
		void send_to_backend(Packet& packet){
			BACKEND_CONNECTION.send(packet);
		}
	};
}
