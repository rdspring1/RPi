#ifndef FUSION_BASE_H
#define FUSION_BASE_H

#include "udp_sender.h"
#include "udp_receiver.h"
#include "interpreter_base.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

const short port = 13;

template<class T>
class FusionBase : public InterpreterBase
{
	public:
		typedef boost::function<void (std::vector<boost::asio::mutable_buffer>&)> create_buffer_fptr;

		FusionBase(ObjectDetector& d, string ip_address, create_buffer_fptr create_buffer) 
			: InterpreterBase(d), work_send(ios_send), work_recv(ios_send)
	{
		// Setup Bi-Direction Communication
		receiver = new udp_receiver<T>(ios_receive, port, create_buffer); 
		sender = new udp_sender<T>(ios_send, boost::asio::ip::address::from_string(ip_address), port);
		asio_send = new boost::thread(boost::bind(&boost::asio::io_service::run, &ios_send));
		asio_receiver = new boost::thread(boost::bind(&boost::asio::io_service::run, &ios_receive));
	}

	protected:
		udp_receiver<T>* receiver;
		udp_sender<T>* sender;

	private:
		boost::asio::io_service ios_send;
		boost::asio::io_service ios_receive;
		boost::asio::io_service::work work_send;
		boost::asio::io_service::work work_recv;
		boost::thread* asio_send;
		boost::thread* asio_receiver;
};
#endif /* FUSION_BASE_H */
