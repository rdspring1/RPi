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
		FpsAvg(unsigned rate, bool update = false) 
			: rate_(rate), update_(update), timer(ios_timer, boost::posix_time::seconds(rate_)), work_timer(ios_timer)
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
			long elapsed_time = (initial_) ? rate_ : (timer.expires_at() - previous_time_).total_seconds();
			fps_ = frame_count_ / elapsed_time;
			previous_time_ = timer.expires_at();
			initial_ = false;

			if(update_)
			{
				frame_count_ = 0;
				timer.expires_at(timer.expires_at() + boost::posix_time::seconds(rate_));
				timer.async_wait(boost::bind(&FpsAvg::printFPS, this, boost::asio::placeholders::error));
				std::cout << "Elapsed Time: " << elapsed_time << " FPS: " << fps_ << std::endl; 
			}
		}

		long frame_count() const
		{
			return frame_count_;
		}

		long value() const
		{
			return fps_;
		}

	private:
		boost::asio::io_service ios_timer;
		const long rate_;
		const bool update_;
		boost::asio::deadline_timer timer;
		boost::asio::io_service::work work_timer;
		boost::posix_time::ptime previous_time_;
		boost::thread* asio_timer;
		long frame_count_ = 0;
		long fps_ = 0;
		bool initial_ = true;
};

#endif /* FPS_H */
