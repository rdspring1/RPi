#include <utility>
#include <algorithm>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

template<class T>
class udp_receiver
{
	public:
		udp_receiver(boost::asio::io_service& io_service, unsigned short port) 
			: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port))
		{
			// Create the socket so that multiple may be bound to the same address.
			socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));

			socket_.async_receive_from(boost::asio::buffer(data_, sizeof(T)), sender_endpoint_,
					boost::bind(&udp_receiver::handle_receive_from, this, 
						boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		}

		void async_receive_msgs(std::vector<T>& data)
		{
			// lock
			mtx_.lock();

			// clear data
			data.clear();

			// swap data with stored data
			std::swap(data, stored_data);	

			// release
			mtx_.unlock();
		}

	private:
		void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd)
		{
			if (!error)
			{
				if(bytes_recvd == sizeof(T))
				{
					// lock
					mtx_.lock();

					// cast buffer to template type T
					T* data = reinterpret_cast<T*>(data_);

					// add new msg to stored data
					stored_data.push_back(*data);

					// release
					mtx_.unlock();
				}

				socket_.async_receive_from(boost::asio::buffer(data_, sizeof(T)), sender_endpoint_,
						boost::bind(&udp_receiver::handle_receive_from, this,
							boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			}
			else
			{
				std::cout << error << std::endl;
			}
		}

		boost::asio::ip::udp::socket socket_;
		boost::asio::ip::udp::endpoint sender_endpoint_;
		std::vector<T> stored_data;
		boost::mutex mtx_;
		char data_[sizeof(T)];
};

