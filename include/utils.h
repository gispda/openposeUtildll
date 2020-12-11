


#pragma once


#include <opencv2/opencv.hpp>

#include <openpose/headers.hpp>
//#include <QString>
#include<string>

using namespace std;

class Utils
{
	Utils();

	~Utils();
public:
	//QString static fromStringToQString(std::string ops);

	//op::DataFormat static stringToDataFormat(std::string dataFormat);

	void static testconststring(const std::string ff);

	std::vector<std::string> static splitData(const std::string& input_string, const char& reg);

};