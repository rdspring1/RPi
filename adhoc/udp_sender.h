#include <iostream>
#include <sstream>
#include <string>
#include <boost/asio.hpp>
#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"

const int max_message_count = 10;

class udp_sender
{
	public:
		udp_sender(boost::asio::io_service& io_service, const boost::asio::ip::address& address, unsigned short port)
			: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
			endpoint_(address, port),
			timer_(io_service),
			message_count_(0)
	{
		socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
		socket_.set_option(boost::asio::socket_base::broadcast(true));

		//boost::shared_ptr<std::string> message_(new std::string("Hello World"));
		std::ostringstream os;
		os << "Message " << message_count_++;
		message_ = os.str();

		socket_.async_send_to(boost::asio::buffer(message_), endpoint_, 
				boost::bind(&udp_sender::handle_send_to, this, boost::asio::placeholders::error));
	}

	private:
		void handle_send_to(const boost::system::error_code& error)
		{
			if (!error)
			{
				timer_.expires_from_now(boost::posix_time::seconds(1));
				timer_.async_wait(boost::bind(&udp_sender::handle_timeout, this, boost::asio::placeholders::error));
			}
			std::cout << error << std::endl;
		}

		void handle_timeout(const boost::system::error_code& error)
		{
			if (!error)
			{
				std::ostringstream os;
                os << "Message " << message_count_++;
                message_ = os.str();
				socket_.async_send_to(boost::asio::buffer(message_), endpoint_, 
						boost::bind(&udp_sender::handle_send_to, this, boost::asio::placeholders::error));
			}
		}

		~udp_sender()
		{
			boost::system::error_code error;
			socket_.close(error);
		}

	private:
		boost::asio::ip::udp::socket socket_;
		boost::asio::ip::udp::endpoint endpoint_;
		boost::asio::deadline_timer timer_;
		int message_count_;
		std::string message_;
};

