//#include "zed_3dreconstruction.h"

//#include <QString>


//#include <QObject>
//#include <QImage>
#include <opencv2/opencv.hpp>
// Command-line user interface
//#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

#include "utils.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>




using namespace std;

Utils::Utils()
{
}


Utils::~Utils()
{
}
//QString  Utils::fromStringToQString(std::string ops)
//{
//	QString ss = "";
//	std::string stds;
//
//	if (ops.empty())
//		return ss;
//
//
//
//	ss = QString::fromStdString(ops);
//	return ss;
//
//}


std::vector<std::string> Utils::splitData(const std::string& input_string, const char& reg)
{
	std::string item;
	std::vector<std::string> elems;

	stringstream input(input_string);

	while (getline(input, item, reg))
	{
		elems.push_back(item);
	}
	return elems;
}

//op::DataFormat Utils::stringToDataFormat(std::string dataFormat)
//{
//	try
//	{
//		if (dataFormat == "json")
//			return op::DataFormat::Json;
//		else if (dataFormat == "xml")
//			return op::DataFormat::Xml;
//		else if (dataFormat == "yaml")
//			return op::DataFormat::Yaml;
//		else if (dataFormat == "yml")
//			return op::DataFormat::Yml;
//		else
//		{
//			op::error("String does not correspond to any known format (json, xml, yaml, yml)",
//				__LINE__, __FUNCTION__, __FILE__);
//			return op::DataFormat::Json;
//		}
//	}
//	catch (const std::exception & e)
//	{
//		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//		return op::DataFormat::Json;
//	}
//}

void Utils::testconststring(const std::string ff)
{

	//	std::cout << ff;
}
