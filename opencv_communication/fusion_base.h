#include "udp_sender.h"
#include "udp_receiver.h"
#include "interpreter_base.h"

#include <boost/thread.hpp>

const short port = 13;

template<class T>
class FusionBase : public InterpreterBase
{
	public:
		FusionBase(ObjectDetector& d, string ip_address) : InterpreterBase(d)
	{
		// Setup Bi-Direction Communication
		receiver = new udp_receiver<T>(ios_receive, port);
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
		boost::thread* asio_send;
		boost::thread* asio_receiver;
};