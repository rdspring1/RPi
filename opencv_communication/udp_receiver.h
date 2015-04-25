#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

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
			data_.front() = boost::asio::buffer(header_);
			data_.back() = boost::asio::buffer(body_);

			// Create the socket so that multiple may be bound to the same address.
			socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));

			socket_.async_receive_from(data_, sender_endpoint_,
					boost::bind(&udp_receiver::handle_receive_from, this, 
						boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		}

		void async_receive_msgs(std::vector< std::pair<T, unsigned char*> >& data)
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
				std::cout << "Initial: " << bytes_recvd << " Header: " << sizeof(T) << std::endl;
				if(bytes_recvd >= sizeof(T))
				{
					// lock
					mtx_.lock();

					++last_message_id;

					// cast buffer to template type T
					T* data = reinterpret_cast<T*>(header_);
					std::cout << "Complete: " << bytes_recvd << " Header: " << sizeof(T) << " Body: " << data->size << std::endl;
					if(bytes_recvd > data->size)
					{
						// add new msg to stored data
						stored_data.push_back(std::make_pair(*data, (unsigned char*) NULL));

						if(data->size > 0)
						{
							stored_data.back().second = (unsigned char*) malloc(data->size);
							memcpy(stored_data.back().second, body_, data->size);
						}
						//std::cout << "Message Received - Successfully" << std::endl;

						if(data->message_id != last_message_id)
						{
							last_message_id = data->message_id;
							++lost_message_count;
						}
					}

					// release
					mtx_.unlock();
				}

				socket_.async_receive_from(data_, sender_endpoint_,
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
		unsigned last_message_id = 0;
		unsigned lost_message_count = 0;
		std::vector< std::pair<T, unsigned char*> > stored_data;
		boost::mutex mtx_;

		const static unsigned BODY_SIZE = 66560; // 65 KB buffer
		const static unsigned BUFFER_SIZE = 2;
		char header_[sizeof(T)];
		char body_[BODY_SIZE];
		std::array<boost::asio::mutable_buffer, BUFFER_SIZE> data_;
};
#endif /* UDP_RECEIVER_H */
