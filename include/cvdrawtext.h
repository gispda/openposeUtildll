#ifndef PUTTEXT_H_
#define PUTTEXT_H_

#include <windows.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace cv;

 class cvdrawText
{
public:
	cvdrawText();
	~cvdrawText();
	static void GetStringSize(HDC hDC, const char* str, int* w, int* h);
	static void putTextZH(cv::Mat& dst, const char* str, Point org, Scalar color, int fontSize,
		const char* fn = "Arial", bool italic = false, bool underline = false, int fontweight = 5);
};
#endif // PUTTEXT_H_