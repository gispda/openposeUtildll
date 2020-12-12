#include "KarmanFilter.hpp"



KarmanFilter::KarmanFilter()
{

	cv::Mat frame;

	// >>>> Kalman Filter
	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;

	unsigned int type = CV_32F;
	kf = new cv::KalmanFilter(stateSize, measSize, contrSize, type);
}


KarmanFilter::~KarmanFilter()
{
	delete kf;
}
