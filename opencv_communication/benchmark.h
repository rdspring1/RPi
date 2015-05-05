#ifndef BENCHMARK_H
#define BENCHMARK_H

#include "interpreter_base.h"
#include "fps_avg.h"

#include <stdexcept>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// Run a test for X seconds using a VideoCapture Device, ObjectLibrary, FusionBase type
// FPS
// detection rate per object
// detection rate per image
// average confidence measure
// variance of confidence
class Benchmark
{
	public:
		Benchmark(cv::VideoCapture& cap, InterpreterBase& ib, long duration) :
			cap_(cap), ib_(ib), duration_(duration), timer_(ios_timer_, boost::posix_time::seconds(duration)),
			object_detected(ib.num_objects()), obj_confidence(ib.num_objects()), 
			image_detected(ib.num_images()), img_confidence(ib.num_images()) {}

		void run();

	private:
		void update(IReport&& ir);

		void report(long frame_count);

		void start_timer();

		void PostRun(const boost::system::error_code& error);

		cv::VideoCapture& cap_;
		InterpreterBase& ib_;
		const long duration_;
		boost::asio::io_service ios_timer_;
		boost::asio::deadline_timer timer_;
		boost::thread* asio_timer_;
		bool run_status_ = true;

		std::vector<double> object_detected;
		std::vector<double> obj_confidence;
		std::vector<double> image_detected;
		std::vector<double> img_confidence;
};
#endif /* BENCHMARK_H */
