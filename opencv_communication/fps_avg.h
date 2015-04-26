#ifndef FPS_AVG_H
#define FPS_AVG_H

#include <iostream>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class FpsAvg
{
	public:
		FpsAvg(unsigned rate) 
			: rate_(rate), timer(ios_timer, boost::posix_time::seconds(rate_)), work_timer(ios_timer)
	{
		timer.async_wait(boost::bind(&FpsAvg::printFPS, this, boost::asio::placeholders::error));
		asio_timer = new boost::thread(boost::bind(&boost::asio::io_service::run, &ios_timer));
	}

		void update()
		{
			++frame_count_;
		}

		void printFPS(const boost::system::error_code& error)
		{
			long elapsed_time = (initial) ? rate_ : (timer.expires_at() - previous_time_).total_seconds();
			previous_time_ = timer.expires_at();
			std::cout << "Elapsed Time: " << elapsed_time << " FPS: " << (frame_count_ / elapsed_time) << std::endl; 
			frame_count_ = 0;
			initial = false;
			timer.expires_at(timer.expires_at() + boost::posix_time::seconds(rate_));
			timer.async_wait(boost::bind(&FpsAvg::printFPS, this, boost::asio::placeholders::error));
		}

	private:
		boost::asio::io_service ios_timer;
		const long rate_;
		boost::asio::deadline_timer timer;
		boost::asio::io_service::work work_timer;
		boost::posix_time::ptime previous_time_;
		boost::thread* asio_timer;
		long frame_count_ = 0;
		bool initial = true;
};

#endif /* FPS_H */
