#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "boost/bind.hpp"

class udp_receiver
{
	public:
		udp_receiver(boost::asio::io_service& io_service, unsigned short port) 
			: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port))
		{
			// Create the socket so that multiple may be bound to the same address.
			socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));

			socket_.async_receive_from(boost::asio::buffer(data_, max_length), sender_endpoint_,
					boost::bind(&udp_receiver::handle_receive_from, this, 
						boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		}

		void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd)
		{
			if (!error)
			{
				std::cout.write(data_, bytes_recvd);
				std::cout << std::endl;

				socket_.async_receive_from(boost::asio::buffer(data_, max_length), sender_endpoint_,
						boost::bind(&udp_receiver::handle_receive_from, this,
							boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			}
			std::cout << error << std::endl;
		}

	private:
		boost::asio::ip::udp::socket socket_;
		boost::asio::ip::udp::endpoint sender_endpoint_;
		enum { max_length = 1024 };
		char data_[max_length];
};

