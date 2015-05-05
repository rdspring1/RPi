#include "benchmark.h"

void Benchmark::run()
{
	FpsAvg fps(duration_);
	start_timer();
	while(run_status_)
	{
		Mat img_scene;
		cap_ >> img_scene;
		cvtColor(img_scene, img_scene, CV_RGB2GRAY, 1);

		if( !img_scene.data )
		{ 
			throw std::runtime_error("Error Reading Image"); 
		}

		try
		{
			update( ib_.detect(img_scene) );
			fps.update();
		}
		catch (std::exception ex)
		{
			std::cout << ex.what() << std::endl;
		}

		if(waitKey(50) >= 0)
		{
			break;
		}
	}
	report(fps.frame_count());
}

void Benchmark::update(IReport&& ir)
{
	for(unsigned idx = 0; idx < ir.objects.size(); ++idx)
	{
		object_detected[idx] += (double) ir.objects[idx];	
		obj_confidence[idx] += ir.object_confidence[idx];
	}

	for(unsigned idx = 0; idx < ir.images.size(); ++idx)
	{
		image_detected[idx] += (double) ir.images[idx];	
		img_confidence[idx] += ir.image_confidence[idx];
	}
}

void Benchmark::report(long frame_count)
{
	std::cout << std::endl;
	for(unsigned idx = 0; idx < ib_.num_objects(); ++idx)
	{
		double obj_detect_rate = object_detected[idx] / (double) frame_count;
		double obj_avg_confidence = obj_confidence[idx] / (double) frame_count;
		std::cout << ib_.obj_name(idx) << "- ODR: " << obj_detect_rate << " OAC: " << obj_avg_confidence << std::endl; 
	}

	std::cout << std::endl;

	for(unsigned idx = 0; idx < ib_.num_images(); ++idx)
	{
		double img_detect_rate = image_detected[idx] / (double) frame_count;
		double img_avg_confidence = img_confidence[idx] / (double) frame_count;
		std::cout << ib_.img_name(idx) << "- IDR: " << img_detect_rate << " IAC: " << img_avg_confidence << std::endl; 
	}
}

void Benchmark::start_timer()
{
	timer_.async_wait(boost::bind(&Benchmark::PostRun, this, boost::asio::placeholders::error));
	asio_timer_ = new boost::thread(boost::bind(&boost::asio::io_service::run, &ios_timer_));
}

void Benchmark::PostRun(const boost::system::error_code& error)
{
	run_status_ = false;
}
