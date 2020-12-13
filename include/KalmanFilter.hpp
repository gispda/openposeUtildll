#ifndef __KARMANFILTER_HPP__
#define __KARMANFILTER_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/video/video.hpp>

#include <openpose/headers.hpp>

///#pragma once
class Kalman_Filter
{
public:
	Kalman_Filter();
	~Kalman_Filter();
protected:
	int stateSize;
	int measSize;
	int contrSize;

	float wbl;

	float omega;

	bool m_binit;

	double ticks;

	double precTick;


	double dT;
	// Camera frame
	cv::KalmanFilter* kf;

	cv::Mat state;// (stateSize, 1, type);  // [x,y,v_x,v_y]
	cv::Mat meas;// (measSize, 1, type);    // [z_x,z_y]



	double render_threshold;

	///节点前一时刻的所有的关节坐标数据
	//op::Array<float> pre_poseKeypoints;

	///节点当前的所有的关节坐标数据
	//op::Array<float> cur_poseKeypoints;

public:

	void init(int x, int y);

	void update(int x, int y);


	void predict();

	bool Isinit()
	{
		return m_binit;
	}
	void getState(int& x, int& y);
	//void init(op::Array<float>& poseKeypoints);
};
#endif

