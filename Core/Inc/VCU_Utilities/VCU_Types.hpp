#pragma once

namespace VCU{

	const IPV4 VCU_IP = {"192.168.1.3"};
	const IPV4 LCU_SLAVE_IP = {"192.168.1.5"};
	const IPV4 LCU_MASTER_IP = {"192.168.1.4"};
	const IPV4 BACKEND_IP = {"192.168.0.9"};
	constexpr uint16_t SERVER_PORT = 50500;
	constexpr uint16_t CLIENT_PORT = 50501;
	constexpr uint16_t UDP_PORT = 50400;

    enum VALVE_STATE{
        CLOSED,
        OPEN,
    };

    enum REED_STATE{
    	EXTENDED,
        RETRACTED,
    };

}

