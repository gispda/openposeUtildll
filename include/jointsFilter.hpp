#ifndef JOINTS_FILTER_HPP_
#define JOINTS_FILTER_HPP_


#include <iostream>
#include <string>
#include <list>

#include <openpose/headers.hpp>
#include <kalmanFilter.hpp>

using namespace std;

class jointsFilter {

public:
	jointsFilter();

	~jointsFilter();
    
	
	void init();

	void clear();

	void stopFilter();

	/**
	* Initialize the filter with a guess for initial states.
	*/
	void startFilter(op::Array<float>& poseKeypoints);

	/**
	* Update the prediction based on control input.
	*/
	//void predict(const Eigen::VectorXd& u);

	

protected:

	bool m_binit;

	double render_threshold;

	///节点前一时刻的所有的关节坐标数据
	op::Array<float> pre_poseKeypoints;

	///节点当前的所有的关节坐标数据
	op::Array<float> cur_poseKeypoints;

	std::map<int, Kalman_Filter*> manKalmantracker;

	void logInfo(int info);
	void logInfo(float info);
	void logInfo(double info);
	void logInfo(bool info);

	//void logInfo(std::string info);


	void logInfo(char* info);


};
#endif