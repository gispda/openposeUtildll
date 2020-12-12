#ifndef __KARMANFILTER_HPP__
#define __KARMANFILTER_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/video/video.hpp>
///#pragma once
class KarmanFilter
{
public:
	KarmanFilter();
	~KarmanFilter();
protected:
	// Camera frame
	cv::KalmanFilter* kf;
};
#endif

