#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <list>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

template<typename T>
class udp_receiver
{
	public:
		typedef boost::function<void (std::vector<boost::asio::mutable_buffer>&)> create_buffer_fptr;

		udp_receiver(boost::asio::io_service& io_service, unsigned short port, create_buffer_fptr create_buffer) 
			: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)), create_buffer_(create_buffer)
		{
			create_buffer_(data_);
			socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
			socket_.async_receive_from(data_, sender_endpoint_,
					boost::bind(&udp_receiver::handle_receive_from, this, 
						boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		}

		void async_receive_msgs(std::list< std::vector<boost::asio::mutable_buffer> >& data)
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
				//std::cout << "Initial: " << bytes_recvd << " Header: " << sizeof(T) << std::endl;
				if(bytes_recvd >= sizeof(T))
				{
					// lock
					mtx_.lock();

					++last_message_id;

					// cast buffer to template type T
					T* header = boost::asio::buffer_cast<T*>(data_.front());
					//std::cout << "Complete: " << bytes_recvd << " Header: " << sizeof(T) << " Body: " << header->size << std::endl;
					if(bytes_recvd == header->size)
					{
						// add new msg to stored data
						std::vector<boost::asio::mutable_buffer> new_data;
						create_buffer_(new_data);
						std::swap(data_, new_data);
						stored_data.push_back(std::move(new_data));

						if(header->message_id != last_message_id)
						{
							last_message_id = header->message_id;
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
		std::list< std::vector<boost::asio::mutable_buffer> > stored_data;
		std::vector<boost::asio::mutable_buffer> data_;
		boost::mutex mtx_;
		create_buffer_fptr create_buffer_;
};
#endif /* UDP_RECEIVER_H */
