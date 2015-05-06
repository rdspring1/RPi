#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <list>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <stdexcept>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

template<typename T>
class UdpReceiver
{
	public:
		typedef std::unordered_map<unsigned, std::vector<boost::asio::mutable_buffer> > MessageList;
		typedef boost::function<void (std::vector<boost::asio::mutable_buffer>&)> create_buffer_fptr;

		UdpReceiver(boost::asio::io_service& io_service, unsigned short port, create_buffer_fptr create_buffer) 
			: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)), create_buffer_(create_buffer)
		{
			create_buffer_(data_);
			socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
			socket_.async_receive_from(data_, sender_endpoint_,
					boost::bind(&UdpReceiver::handle_receive_from, this, 
						boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		}

		void async_receive_msgs(MessageList& data)
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

		~UdpReceiver()
		{
			boost::system::error_code error;
			socket_.close(error);
		}

	private:
		void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd)
		{
			if (!error)
			{
				//std::cout << "Total: " << bytes_recvd << " Header: " << sizeof(T) << std::endl;
				if(bytes_recvd >= sizeof(T))
				{
					// lock
					mtx_.lock();

					// cast buffer to template type T
					T* new_header = boost::asio::buffer_cast<T*>(data_.front());
					//std::cout << "Complete: " << bytes_recvd << " Header: " << sizeof(T) << " Body: " << header->size << std::endl;
					if(bytes_recvd == new_header->size)
					{
						// Check if this message is from a new neighbor
						bool update = (stored_data.count(new_header->robot_id) == 0);

						// If not, replace with newer message (check timestamp)
						if(!update)
						{
							T* current_header = boost::asio::buffer_cast<T*>(stored_data[new_header->robot_id].front());
							update = (new_header->timestamp > current_header->timestamp);
						}

						if(update)
						{
							// add new msg to stored data
							std::vector<boost::asio::mutable_buffer> new_data;
							create_buffer_(new_data);
							std::swap(data_, new_data);
							stored_data[new_header->robot_id] = std::move(new_data);
						}
					}
					else
					{
						throw std::runtime_error("Failed to receive message: " + bytes_recvd);
					}

					// release
					mtx_.unlock();
				}

				socket_.async_receive_from(data_, sender_endpoint_,
						boost::bind(&UdpReceiver::handle_receive_from, this,
							boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			}
			else
			{
				throw std::runtime_error(error.message());
			}
		}

		boost::asio::ip::udp::socket socket_;
		boost::asio::ip::udp::endpoint sender_endpoint_;
		MessageList stored_data;
		std::vector<boost::asio::mutable_buffer> data_;
		boost::mutex mtx_;
		create_buffer_fptr create_buffer_;
};
#endif /* UDP_RECEIVER_H */
