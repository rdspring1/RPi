#include "udp_sender.h"
#include "udp_receiver.h"
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
	bool send = false;
	if(argc == 2)
	{
		send = true;
	}

	try
	{
		const short port = 13;
		boost::asio::io_service ioservice;
		udp_receiver* receiver = NULL;
		udp_sender* sender = NULL;
		if(!send)
		{
			receiver = new udp_receiver(ioservice, port);
		}
		else
		{
			// Default Broadcast: 255.255.255.255 / ff::ff::ff::ff
			// Subnet Broadcast: private ip address space | (~subnet mask)
			// i.e. 10.42.0.0/24 | 255.255.255.0 -> 10.42.0.255
			sender = new udp_sender(ioservice, boost::asio::ip::address::from_string(argv[1]), port);
		}
		ioservice.run();
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
