/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>


#include "jointsfilter.hpp"
#include "simlog.h"


jointsFilter::jointsFilter()
{

	m_binit = false;

	render_threshold = 0.05; ///等于flags.hpp中

}

jointsFilter::~jointsFilter()
{
	clear();
}

void jointsFilter::init()
{


}

void jointsFilter::clear()
{

	for (auto& kv : manKalmantracker) {
		delete kv.second;
		kv.second = NULL;
	}
	manKalmantracker.clear();
	m_binit = false;
}

void jointsFilter::stopFilter()
{
}


void jointsFilter::logInfo(int info)
{

	LDebug("number is :{}", info);
}

void jointsFilter::logInfo(float info)
{
	LDebug("number is :{}", info);
}

void jointsFilter::logInfo(double info)
{
	LDebug("number is :{}", info);
}

void jointsFilter::logInfo(bool info)
{
	LDebug("number is :{}", info);
}

void jointsFilter::logInfo(char* info)
{

	LDebug(info);
}
void jointsFilter::startFilter(op::Array<float>& poseKeypoints)
{
	int ux = -1, uy = -1;
	int x = -1, y = -1;
	Kalman_Filter* kf = NULL;
	double score = 0;
	if (m_binit == false){
	
		for (auto part = 0; part < poseKeypoints.getSize(1); part++)
		{
			logInfo("body part index");
			logInfo(part);
			kf = new Kalman_Filter();

			score = poseKeypoints[{0, part, 2}];
			if (score >= render_threshold)  // skip low score
			{
				x = round(poseKeypoints[{0, part, 0}]);
				y = round(poseKeypoints[{0, part, 1}]);

				logInfo("init source x,y");
				logInfo(x);
				//logInfo("update source x,y---------------");
				logInfo(y);
			

				kf->init(x, y);

				logInfo("init source x,y end");
			}

			manKalmantracker.insert(std::make_pair(part, kf));
		}

		m_binit = true;
	}  ///初始化建立25个卡尔曼滤波对象,有x,y数据的就init
	else
	{
		for (auto part = 0; part < poseKeypoints.getSize(1); part++)
		{
			logInfo("body part index");
			logInfo(part);

			kf = manKalmantracker[part];

			score = poseKeypoints[{0, part, 2}];
			if (score >= render_threshold)  // skip low score
			{
				x = round(poseKeypoints[{0, part, 0}]);
				y = round(poseKeypoints[{0, part, 1}]);
				logInfo("update source x,y");
				logInfo(x);
				
				logInfo(y);
				logInfo("update source x,y---------------");
				kf->update(x, y);
				logInfo("update x,y  end");
				kf->getState(ux, uy);

				
				logInfo("track getstate");
				poseKeypoints[{0, part, 0}] = ux;
				poseKeypoints[{0, part, 1}] = uy;

				logInfo(ux);

				logInfo(uy);
				logInfo("track getstate over");
			}

		//	manKalmantracker.insert(std::make_pair(part, kf));
		}

	}

	//cur_poseKeypoints = poseKeypoints;



}
