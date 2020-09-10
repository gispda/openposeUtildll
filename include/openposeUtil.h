#ifndef OPENPOSEUTIL_H
#define OPENPOSEUTIL_H


#pragma once;

//#define GLOG_NO_ABBREVIATED_SEVERITIES


//#include <QtGui>
//#include <chrono>


#include <thread>
#include <vector>
//#include <utility>
//#include <QMetaType>
using namespace std;
// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
//#include <gflags/gflags.h>
//// Allow Google Flags in Ubuntu 14
//#ifndef GFLAGS_GFLAGS_H_
//namespace gflags = google;
//#endif
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <opencv2/highgui.hpp>


//

#include <cstdint>
#include "utf8.h"


#include <sl/Camera.hpp>
#include "manpose.h"

#include "cvdrawtext.h"
#include <json/json.h>

#define ENABLE_FLOOR_PLANE_DETECTION 1 // Might be disable to use older ZED SDK

// Debug options
#define DISPLAY_BODY_BARYCENTER 1
#define PATCH_AROUND_KEYPOINT 1 



#define random(a,b) (rand()%(b-a)+a)

using namespace cv;
using namespace sl;

extern "C" void clean();

class openposeUtil
{
  

public:
	openposeUtil();
    ~openposeUtil();
	void stopposeservice();

private:
    //  volatile bool stopped;
    int inputSourceWidth;
    int inputSourceHeight;

    sl::Mat currentFrame;
  //  QImage frame;
    //   QMutex stoppedMutex;
//    QMutex updateMembersMutex;
    Size frameSize;
    Point framePoint;
   // QString name;
    // Image processing flags
private:


	VideoWriter outposeVideo;
	VideoWriter outzedVideo;
	VideoWriter outmergeVideo;



    float thres_score;
	// Create ZED objects
	sl::Camera zed;
	sl::Pose camera_pose;
	std::thread zed_callback, openpose_callback,savesvo_callback,saveposeavi_callback,savezedavi_callback;
	//std::thread savesvo_callback, saveposeavi_callback, savezedavi_callback;
	std::mutex data_in_mtx, data_out_mtx;
	std::vector<op::Array<float>> netInputArray;
	op::Array<float> poseKeypoints;
	op::Point<int> imageSize, outputSize, netInputSize, netOutputSize;
	op::PoseModel poseModel;
	std::vector<double> scaleInputToNetInputs;


	sl::Plane plane;
	sl::Transform resetTrackingFloorFrame;
	int image_width;
	int image_height;

	int simage_width;
	int simage_height;

	bool need_new_image;
	bool ready_to_start;
	int model_kp_number;


	bool isfirst;


	bool quit;
	manpose* _manpose;
	// OpenGL window to display camera motion
	//GLViewer viewer;

	cplane plane1;
	cplane plane2;

	cline3d line;

	const int MAX_CHAR = 128;
	const sl::UNIT unit = sl::UNIT::METER;
	const float MAX_DISTANCE_LIMB = 1; //0.8;
	const float MAX_DISTANCE_CENTER = 1.8; //1.5;



	Json::Reader json_reader;// 解析json用Json::Reader   
	Json::Value json_root; // Json::Value是一种很重要的类型，可以代表任意类型。如int, string, object, array         
	Json::Value json_bodyls;   // 创建数组
	Json::StreamWriterBuilder  builder;

	bool isrootwrite;
	std::ifstream json_is;
	cv::Mat image_ocv;

	cv::Mat hkimg;
	cv::Mat zedsvoimg;
	cv::Mat zedposeimg;
	cv::Mat zedposedataimg;

	cv::Mat dst_img;


	int poseimgidx;

	int poseimgnum;
	int zedimgnum;
	int hkimgnum;
	int jsonnum;


	//op::PoseExtractorCaffe* poseExtractorCaffe;
	//{ poseModel, FLAGS_openpose_root_dir + FLAGS_model_folder, FLAGS_num_gpu_start, {}, op::ScaleMode::ZeroToOne, 1 };


	std::string printangleinfo(AngleInfo& angle)
	{
	  std::string info;

	  info.append(angle.desc).append(":").append(floatTostring(angle.angle.angle, 5));
	  info.append(" 平面角度:");
	  info.append(floatTostring(angle.angle.anglexoy,5));
	  info.append(" 矢角度:");
	  info.append(floatTostring(angle.angle.angleyoz,5));
	  return info;
	}

	std::list<AngleInfo> openposeUtil::getAnglesFromonebodypose(Json::Value _json_bodypose);

	void recalcposedatimg(Json::Value _json_bodypose);

	Json::Value getJsonByIndex(std::list<Json::Value>& _rs,int idx)
	{
		int i = 0;
		for (list <Json::Value> ::iterator it = _rs.begin(); it != _rs.end(); ++it)
		{			
			if (i == idx)
				return *it;
			i++;
		}
	}

	std::list<Json::Value> getbodyposesfromjsonfile(std::string _json_file);
	std::list<AngleInfo> fromjsonfile(std::string _json_file);

	// Sample functions
	void startZED();
	void startOpenpose();
	void runZed();

	void findpose();


	void drawText(cv::Mat* image,AngleInfo angle, Body body=BODY_RIGHT);

	void drawText(cv::Mat* image, std::string _text, cv::Point origin);
	void initDevice();
	void fill_people_ogl(op::Array<float>& poseKeypoints, sl::Mat& xyz);
	//// ����õ�ϵͳ��Ҫ�ĸ��ֽǶ�
	void calcmanpose(op::Array<float>& poseKeypoints, sl::Mat& xyz);

	void InitAngle(Angle& _angle);


	void calcplaneangle(std::map<int, sl::float4> keypoints_position);


	void mergeimg(cv::Mat& img1, cv::Mat& img2, cv::Mat& img3, cv::Mat& img4);

	std::string doubleTostring(const double value, unsigned int precision)
	{
		std::ostringstream out;
		if (precision > 0)
			out.precision(precision);

		out << value;
		return out.str();
	}

	std::string floatTostring(const float value, unsigned int precision)
	{
		std::ostringstream out;
		if (precision > 0)
			out.precision(precision);

		out << value;
		return out.str();
	}


	sl::float4 getvector_unit(sl::float4 v1)
	{
		return v1/sqrtf(v1.x*v1.x+v1.y*v1.y+v1.z*v1.z);	
	}
	sl::double3 fromfloat3(sl::float3 v)
	{
		sl::double3 dv = sl::double3(v.x, v.y, v.z);
		return dv;
	}
	sl::double3 fromfloat4(sl::float4 v)
	{
		sl::double3 dv = sl::double3(v.x, v.y, v.z);
		return dv;
	}


	double getdotproduct(sl::float4 v1, sl::float4 v2)
	{
		double ret = v1.x*v2.x + v1.y*v2.y + v1.z *v2.z;
		return ret;
	
	}

	double getdotproductyoz(sl::float4 v1, sl::float4 v2)
	{
		double ret = v1.y * v2.y + v1.z * v2.z;
		return ret;

	}
	double getdotproductxoy(sl::float4 v1, sl::float4 v2)
	{
		double ret = v1.x * v2.x + v1.y * v2.y;
		return ret;

	}


	std::string  getNowTime()
	{
		time_t nowtime = time(NULL);
		struct tm* l = localtime(&nowtime);
		char buf[128];

		int randnum = random(0, 99);
		snprintf(buf, sizeof(buf), "%04d%02d%02d%02d%02d%02d%02d", l->tm_year + 1900, l->tm_mon + 1, l->tm_mday, l->tm_hour, l->tm_min, l->tm_sec, randnum);
		std::string s(buf);
		return s;
	}


	////计算pv0,pv1 和 pv2,pv3之间的夹角，如果是平面法线则pv3不考虑
	Angle calctwovectorang(int  vidx[4], sl::float4& pv0, sl::float4& pv1, sl::float4& pv2, sl::float4& pv3, std::map<int, sl::float4>& keypoints_position);

	bool AssertVectorIsNAN(sl::float4& pv0);

	bool IsUseful(sl::float4& v1, sl::float4& v2, sl::float4& center_gravity);

	sl::float4 getPatchIdx(const int& center_i, const int& center_j, sl::Mat& xyzrgba);
	void fill_ptcloud(sl::Mat& xyzrgba);





	cv::Mat fromjson(std::string json_file);
	
public:



	


	bool initFloorZED(sl::Camera& zed);


	//void configureptmainwindow(PTMainWindow* ppm);
	void initLogParameter(int argc, char* arcgv[]);
	int getlogging_level();
	bool getdisable_multi_thread();
	std::string getoutput_resolution();
	std::string getmodel_folder();
	std::string getwrite_json();
	std::string getnet_resolution();
	std::string getvideo();
	std::string getimage_dir();
	std::string getwrite_images();
	std::string getwrite_video();


	void setlogging_level(int logging_level);
	void setdisable_multi_thread(bool disable_multi_thread);
	void setoutput_resolution(std::string output_resolution);
	void setmodel_folder(std::string model_folder);
	void setwrite_json(std::string write_json);
	void setnet_resolution(std::string net_resolution);
	void setvideo(std::string video);
	void setimage_dir(std::string image_dir);
	void setwrite_images(std::string write_images);
	void setwrite_video(std::string write_video);
	void logInfo(std::string info);
	

	void logInfo(int info);
	void logInfo(float info);
	void logInfo(double info);
	void logInfo(bool info);

	//void logInfo(std::string info);
	
	void logInfo(AngleInfo angle);
	void logInfo(char* info);

	void logInfo(sl::float3 info);
	void logInfo(sl::float4 info);

	void logInfo(sl::Transform info);
	void logInfo(sl::Translation info);


	void logInfo(int vidx[4]);

	///////启动zed相机


	void startzeddevice();

	///////关闭zed相机


	void closezeddevice();


	//////////
	///启动姿态估计算法服务
	////

	void startposeservice(std::string svo_files, bool isshow = false);

	

	////只能跟随程序一起退出姿态估计算法服务
	void stoppposewithexitsys();



	void startposeserviceonlinezed();

	//调用同步zedopenpose处理
	void startzedopenpose();


	//同步运行zed和openpose
	void runzedopenpose();

	///一次采集
	void startgetposeavidata();

	///一次采集
	void endgetposeavidata();



	std::string UTF8ToGB(const std::string& str);

	std::wstring s2ws(const std::string& str);
	
	std::string ws2s(const std::wstring& wstr);

	std::string replace_utf8_escape_sequences(const std::string& str);


	std::string getzedcurimg();

	void startRecordPose();

	std::string endRecordPose();

	bool startSavezedSvo();

	std::string endSavezedSvo();

	bool savesvo();

	bool saveposeavi();

	void addmanpose();

	//std::string startSavezedSvo();


	bool saveposedata();

	void startSavezedavi();

	std::string endSavezedavi();


	bool savezedavi();

	
	void GetStringSize(HDC hDC, const char* str, int* w, int* h);
	int gettextlen(cv::Mat& dst, const char* str, Point org);



	std::string startmergereportavi(std::string hmavi_file, std::string zedsvo_file, std::string zedposeavi_file, std::string posedata_file, bool isshow = false);


	void setmerge_avi_File(std::string _stdmerge_avi_File)
	{
		//stdsvo_File = _svofile;
		stdmerge_avi_File = _stdmerge_avi_File;
	}

	void setsvo_File(std::string _svofile)
	{
		stdsvo_File = _svofile;
	
	}
	void setzed_avi_File(std::string _zed_avi_File)
	{
		stdzed_avi_File = _zed_avi_File;

	}
	void setpose_avi_File(std::string _poseavifile)
	{
		stdpose_avi_File = _poseavifile;

	}
	void setpose_data_Dir(std::string _posedatadir)
	{
		stdpose_data_Dir = _posedatadir;

	}

	std::string getmerge_avi_File()
	{
		if (stdmerge_avi_File.compare("") == 0)
		{
			stdmerge_avi_File = getNowTime() + ".avi";
		}

		return stdmerge_avi_File;
	}


	std::string getpose_avi_File()
	{
		if (stdpose_avi_File.compare("") == 0)
		{
			stdpose_avi_File = getNowTime() + ".avi";
		}

		return stdpose_avi_File;

	}

	std::string getzed_avi_File()
	{
		if (stdzed_avi_File.compare("") == 0)
		{
			stdzed_avi_File = getNowTime() + ".avi";
		}

		return stdzed_avi_File;

	}



	
	std::string getpose_data_Dir()
	{
		if (stdpose_data_Dir.compare("") == 0)
		{
			stdpose_data_Dir = getNowTime();
		}

		return stdpose_data_Dir;
	}
	std::string getsvo_File()
	{
		if (stdsvo_File.compare("") == 0)
		{
			stdsvo_File = getNowTime() + ".svo";
		}

		return stdsvo_File;
	}


	void reinit();


	bool isCreatePoseAvi();
protected:

	bool bzero;

	bool bcreateposeavi;

	bool bshow;



	bool bsaveSvo;


	bool bsavePose;
	bool bsaveZed;

	std::string stdsvo_File;
	std::string stdpose_avi_File;
	std::string stdzed_avi_File;
	std::string stdpose_data_Dir;
	std::string stdmerge_avi_File;

	std::ofstream fout;

	long  maxframecount;

	long jsonidx;
//	PTMainWindow* pptmainwindow;
   
//	QImage fromCvMat(cv::Mat cvimgmat);
//signals:
  //  void newFrame(const QImage &frame);
    //void newName(const QString &name);
  //  void newPoint(const vector<pair<float,float> > &);
};

#endif // OPENPOSEUTIL_H
