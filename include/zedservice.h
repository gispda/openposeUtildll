#ifndef ZEDSERVICE_H
#define ZEDSERVICE_H


#pragma once;

//#include <qobject.h>

// Qt header files
//#include <QtGui>
// OpenCV header files
//#include <opencv/highgui.h>
#include <xstring>

#include <time.h>

class openposeUtil;
///class PTMainWindow;
class zedservice
{
  //  Q_OBJECT

public:
    zedservice();
    ~zedservice();
    //ImageBuffer *imageBuffer;
   



	///////����zed���


	//static void startzeddevice();

	///////�ر�zed���


	//static void closezeddevice();


	//////////
	///������̬�����㷨����
	////

	static std::string startposeservice(std::string svo_files,bool isshow=false);
	static bool isCreatePoseAvi();
	static void stopposeservice();

	////ֻ�ܸ������һ���˳���̬�����㷨����
	//static void stoppposewithexitsys();

	/////�ɼ�svo��Ƶ�ļ�
	//static void startSavesvo();
	//static std::string endSavesvo();


	/////�ɼ�����̬���ƽ������Ƶ�ļ���������Ƶ�ļ�������̬�����ı������jsonĿ¼��
	//static void startSaveposeavi();
	//static std::string endSaveposeavi();


	/////�ɼ�zed�����ԭʼ��Ƶavi��ʽ��������Ƶ�ļ���
	//static void startSavezedavi();
	//static std::string endSavezedavi();



	////�ɼ�zed���ͼƬ,����ͼƬ��
	static std::string getzedcurimg();

	////�ϳ��Ĺ�����Ƶ�ļ����ֱ����뺣����Ƶ��zed��avi��ʽԭʼ��Ƶ������̬���ƽ������Ƶ����̬���ƽ����json�ļ���Ŀ¼,�����Ĺ������Ƶ�ļ���
	static std::string startmergereportavi(std::string hmavi_file, std::string zedsvo_file, std::string zedposeavi_file, std::string posedata_file, bool isshow = false);

    /////��������zed���������̬�����㷨
   // static void startposeserviceonlinezed();


	/*static void startSavezedSvo();
	static std::string endSavezedSvo();
    */


	///һ�βɼ�
	//static void startgetposeavidata();

	//static void endgetposeavidata();

    void deleteProcessingThread();
	static openposeUtil* getCurZedservice();

	//void configureptmainwindow(PTMainWindow* ppm);
	void initLogParameter(int argc, char* arcgv[]);
	//int getlogging_level();
	//bool getdisable_multi_thread();
	//std::string getoutput_resolution();
	//std::string getmodel_folder();
	//std::string getwrite_json();
	//std::string getnet_resolution();
	//std::string getvideo();
	//std::string getimage_dir();
	//std::string getwrite_images();
	//std::string getwrite_video();


	/*void setlogging_level(int logging_level);
	void setdisable_multi_thread(bool disable_multi_thread);
	void setoutput_resolution(std::string output_resolution);
	void setmodel_folder(std::string model_folder);
	void setwrite_json(std::string write_json);
	void setnet_resolution(std::string net_resolution);
	void setvideo(std::string video);
	void setimage_dir(std::string image_dir);
	void setwrite_images(std::string write_images);
	void setwrite_video(std::string write_video);*/



	

	//static void setsvoFIle(std::string _svofile);
	//void logInfo(std::string info);
public:
	//static std::string getsvo_file();

	//static std::string getzedavi_file();
	static std::string getpose_avi_file();
	static std::string getpose_data_dir();

	static std::string  calcInt()
	{
		time_t nowtime = time(NULL);
		struct tm* l = localtime(&nowtime);
		char buf[128];

		//int randnum = random(0, 99);
		snprintf(buf, sizeof(buf), "%04d%02d%02d%02d%02d%02d", l->tm_year + 1900, l->tm_mon + 1, l->tm_mday, l->tm_hour, l->tm_min, l->tm_sec);
		std::string s(buf);
		return s;
	}

private:
    int imageSize;

	int mergeint =1;
	int poseint = 1;
};


#endif // ZEDSERVICE_H
