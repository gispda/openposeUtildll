#include "KalmanFilter.hpp"
#include <iostream>



Kalman_Filter::Kalman_Filter()
{

	//cv::Mat frame;

	// >>>> Kalman Filter
	stateSize = 4;
	measSize = 2;
	contrSize = 0;

	ticks = 0;

	dT = 0;

	wbl = 30.0f;

	omega = 100.0f;

	m_binit = false;

	unsigned int type = CV_32F;
	kf = new cv::KalmanFilter(stateSize, measSize, contrSize, type);

	state.create(stateSize, 1, type);  // [x,y,v_x,v_y]
	meas.create(measSize, 1, type);    // [z_x,z_y]


	//cv::Mat procNoise(stateSize, 1, type)
	// [E_x,E_y,E_v_x,E_v_y]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0]
	// [ 0 1 0 dT]
	// [ 0 0 1 0]
	// [ 0 0 0 1]
	cv::setIdentity(kf->transitionMatrix);


	// Measure Matrix H
   // [ 1 0 0 0]
   // [ 0 1 0 0]
  
	kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf->measurementMatrix.at<float>(0) = 1.0f;
	kf->measurementMatrix.at<float>(5) = 1.0f;


	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0   ]
	// [ 0    Ey  0     0   ]
	// [ 0    0   Ev_x  0   ]
	// [ 0    0   0     Ev_y]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf->processNoiseCov.at<float>(0) = 1e-4;
	kf->processNoiseCov.at<float>(5) = 1e-4;
	kf->processNoiseCov.at<float>(10) = omega;
	kf->processNoiseCov.at<float>(15) = omega;



	// Measures Noise Covariance Matrix R
   // [ w 16w]  

	kf->measurementNoiseCov = cv::Mat::zeros(measSize, 1, type);
	kf->measurementNoiseCov.at<float>(0) = wbl;
	kf->measurementNoiseCov.at<float>(1) = 16* wbl;
	



}


Kalman_Filter::~Kalman_Filter()
{
	
	delete kf;
}

void Kalman_Filter::init(int x, int y)
{

	meas.at<float>(0) = (float)x;
	meas.at<float>(1) = (float)y;



	if (!m_binit) // First detection!
	{
		// >>>> Initialization
		kf->errorCovPre.at<float>(0) = 1; // px
		kf->errorCovPre.at<float>(5) = 1; // px
		kf->errorCovPre.at<float>(10) = 1;
		kf->errorCovPre.at<float>(15) = 1;

		state.at<float>(0) = meas.at<float>(0);
		state.at<float>(1) = meas.at<float>(1);
		state.at<float>(2) = 0;
		state.at<float>(3) = 0;
		// <<<< Initialization

		kf->statePost = state;

		m_binit = true;
	}
}

void Kalman_Filter::update(int x, int y)
{
	meas.at<float>(0) = (float)x;
	meas.at<float>(1) = (float)y;

	int prex, prey;

	getState(prex, prey);

	if (fabs(x - prex) >= 150 || fabs(y - prey) >= 350)
	{
	////简单条件判断是否换人了,重新初始化
		m_binit = false;
		init(x, y);
	}
	else
	{
		kf->correct(meas);
		predict();
	}
}

void Kalman_Filter::predict()
{
	precTick = ticks;
	ticks = (double)cv::getTickCount();

	dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

	if (m_binit)
	{
		// >>>> Matrix A
		kf->transitionMatrix.at<float>(2) = dT;
		kf->transitionMatrix.at<float>(9) = dT;
		// <<<< Matrix A

		std::cout << "dT:" << std::endl << dT << std::endl;

		state = kf->predict();
		std::cout << "State post:" << std::endl << state << std::endl;		
	}

}

void Kalman_Filter::getState(int& x, int& y)
{
	x = state.at<float>(0);
	y = state.at<float>(1);
}



//void KarmanFilter::init(op::Array<float>& poseKeypoints)
//{
//
//	cur_poseKeypoints = poseKeypoints;
//}
