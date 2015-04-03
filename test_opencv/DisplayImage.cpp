#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    VideoCapture cap(0);
    if(!cap.isOpened())
        return -1;
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    namedWindow("Display Image", WINDOW_AUTOSIZE );
    for( ; ; )
    {
        Mat image;
        cap >> image;
	cvtColor(image, image, CV_RGB2HSV);
        imshow("Display Image", image);
        if(waitKey(30) >= 0)
            break;
    }

    return 0;
}
