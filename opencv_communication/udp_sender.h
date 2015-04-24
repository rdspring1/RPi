#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <iostream>
#include <sstream>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

template<class T>
class udp_sender
{
	public:
		udp_sender(boost::asio::io_service& io_service, const boost::asio::ip::address& address, unsigned short port)
			: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port+1)),
			endpoint_(address, port),
			message_count_(0)
	{
		socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
		socket_.set_option(boost::asio::socket_base::broadcast(true));
	}

		const unsigned message_id()
		{
			return message_count_;
		}

		void async_send_msg(T& msg)
		{
			socket_.async_send_to(boost::asio::buffer(&msg, sizeof(msg)), endpoint_, 
					boost::bind(&udp_sender::handle_send_to, this, boost::asio::placeholders::error));
			++message_count_;
		}

		void async_send_msg(T&& header, std::vector<unsigned char>& body)
		{
			std::vector<boost::asio::const_buffer> buffer;
			buffer.push_back(boost::asio::buffer(&header, sizeof(T)));
			buffer.push_back(boost::asio::buffer(body));
			socket_.async_send_to(buffer, endpoint_, 
					boost::bind(&udp_sender::handle_send_to, this, 
					boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			++message_count_;
		}

	private:

		void handle_send_to(const boost::system::error_code& error, size_t bytes_transferred)
		{
			if (error)
			{
				std::cout << "Failed to send message: " << error << std::endl;
			}
			else
			{
				std::cout << "Send Message: " << bytes_transferred << std::endl;
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
		unsigned message_count_;
};
#endif /* UDP_SENDER_H */
