#ifndef __UTILS_HPP__
#define __UTILS_HPP__

//#define NDEBUG

#pragma once;

///#define GLOG_NO_ABBREVIATED_SEVERITIES

//#include "GL/glut.h"    /* OpenGL Utility Toolkit header */
//
//#include <GL/freeglut.h>

#include <math.h>
#include <stdint.h>
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

#include <sl/Camera.hpp>
#include <openpose/headers.hpp>

#ifndef M_PI
#define M_PI 3.1416f
#endif


using namespace sl;




cv::Mat slMat2cvMat(sl::Mat& input);
/**
 * Conversion function between sl::Mat and cv::Mat
 **/


cv::Mat slMat2cvMat(sl::Mat& input) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	//qDebug() << "convert sl mat to cv mat: ";
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}




#endif /*__UTILS_HPP__*/
