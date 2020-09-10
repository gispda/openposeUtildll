

#include "zedservice.h"
#include "openposeUtil.h"

#include "global.h"
#include "dirutil.h"


// Qt header files
//#include <QtGui>

//extern ProcessingThread* processingThread 
openposeUtil* mzedservice=NULL;

zedservice::zedservice(){

	if(mzedservice==NULL)
	mzedservice = new openposeUtil();
}

zedservice::~zedservice(){
	deleteProcessingThread();
}

//void zedservice::startzeddevice()
//{
//}
//
//void zedservice::closezeddevice()
//{
//}
//
//
//void  zedservice::startSavezedSvo()
//{
//	cout << "start save zed svo" << endl;
//	zedservice::getCurZedservice()->startSavezedSvo();
//}
//std::string zedservice::endSavezedSvo()
//{
//
//	return zedservice::getCurZedservice()->endSavezedSvo();
//}


////capture video
//void zedservice::startposeserviceonlinezed()
//{
//    //创建处理进程
//    
//
//    //启动处理进程
//	zedservice::getCurZedservice()->startposeserviceonlinezed();
//
//    return;
//}

openposeUtil* zedservice::getCurZedservice()
{

	if (mzedservice == NULL)
		mzedservice = new openposeUtil();
	return mzedservice;
}


void zedservice::initLogParameter(int argc, char* arcgv[])
{
	zedservice::getCurZedservice()->initLogParameter(argc, arcgv);
}
//
//int zedservice::getlogging_level()
//{
//	return zedservice::getCurZedservice()->getlogging_level();
//}
//
//bool zedservice::getdisable_multi_thread()
//{
//	return zedservice::getCurZedservice()->getdisable_multi_thread();
//}
//
//std::string zedservice::getoutput_resolution()
//{
//	return zedservice::getCurZedservice()->getoutput_resolution();
//}
//
//std::string zedservice::getmodel_folder()
//{
//	return zedservice::getCurZedservice()->getmodel_folder();
//}
//
//std::string zedservice::getwrite_json()
//{
//	return zedservice::getCurZedservice()->getwrite_json();
//}
//
//std::string zedservice::getnet_resolution()
//{
//	return zedservice::getCurZedservice()->getnet_resolution();
//}
//
//std::string zedservice::getvideo()
//{
//	return zedservice::getCurZedservice()->getvideo();
//}
//
//std::string zedservice::getimage_dir()
//{
//	return zedservice::getCurZedservice()->getimage_dir();
//}
//
//std::string zedservice::getwrite_images()
//{
//	return zedservice::getCurZedservice()->getwrite_images();
//}
//
//std::string zedservice::getwrite_video()
//{
//	return zedservice::getCurZedservice()->getwrite_video();
//}
//
//void zedservice::setlogging_level(int logging_level)
//{
//	zedservice::getCurZedservice()->setlogging_level(logging_level);
//}
//
//void zedservice::setdisable_multi_thread(bool disable_multi_thread)
//{
//	zedservice::getCurZedservice()->setdisable_multi_thread(disable_multi_thread);
//}
//
//void zedservice::setoutput_resolution(std::string output_resolution)
//{
//	zedservice::getCurZedservice()->setoutput_resolution(output_resolution);
//}
//
//void zedservice::setmodel_folder(std::string model_folder)
//{
//	zedservice::getCurZedservice()->setmodel_folder(model_folder);
//}
//
//void zedservice::setwrite_json(std::string write_json)
//{
//	zedservice::getCurZedservice()->setwrite_json(write_json);
//}
//
//void zedservice::setnet_resolution(std::string net_resolution)
//{
//	zedservice::getCurZedservice()->setnet_resolution(net_resolution);
//}
//
//void zedservice::setvideo(std::string video)
//{
//	zedservice::getCurZedservice()->setvideo(video);
//}
//
//void zedservice::setimage_dir(std::string image_dir)
//{
//	zedservice::getCurZedservice()->setimage_dir(image_dir);
//}
//
//void zedservice::setwrite_images(std::string write_images)
//{
//	zedservice::getCurZedservice()->setwrite_images(write_images);
//}
//
//void zedservice::setwrite_video(std::string write_video)
//{
//	zedservice::getCurZedservice()->setwrite_video(write_video);
//}

std::string zedservice::startposeservice(std::string svo_files, bool isshow)
{


	string curdir = dirutil::GetCurrentWorkingDir();

	zedservice::getCurZedservice()->startposeservice(svo_files,isshow);


	
	///string mergeavi = zedservice::getCurZedservice()->startmergereportavi(hmavi_file, zedsvo_file, zedposeavi_file, posedata_file, isshow);
	string resbuffer;

	string strint = calcInt();

	resbuffer.append(strint);
	resbuffer.append("#");
	resbuffer.append("startposeservice");
	resbuffer.append("#");
	resbuffer.append(curdir).append("\\").append(zedservice::getCurZedservice()->getpose_avi_File());
	resbuffer.append("#");
	resbuffer.append(zedservice::getCurZedservice()->getpose_data_Dir());
	return resbuffer;


}

bool zedservice::isCreatePoseAvi()
{
	return zedservice::getCurZedservice()->isCreatePoseAvi();
}

void zedservice::stopposeservice()
{

	cout << "in here" << endl;
	zedservice::getCurZedservice()->stopposeservice();
}


//void zedservice::startSavesvo()
//{
//	//setsvoFIle(svofile);
//	zedservice::getCurZedservice()->startRecordSvo();
//}
//
//std::string zedservice::endSavesvo()
//{
//
//	return zedservice::getCurZedservice()->startSavezedsvo();
//}

//void zedservice::startSaveposeavi()
//{
//	zedservice::getCurZedservice()->startRecordPose();
//}

//std::string  zedservice::endSaveposeavi()
//{
//	return zedservice::getCurZedservice()->endRecordPose();
//}

//void zedservice::startSavezedavi()
//{
//	zedservice::getCurZedservice()->startSavezedavi();
//}

std::string zedservice::getzedcurimg()
{


	return zedservice::getCurZedservice()->getzedcurimg();
}

//std::string zedservice::endSavezedavi()
//{
//
//	return zedservice::getCurZedservice()->endSavezedavi();
//}

std::string zedservice::startmergereportavi(std::string hmavi_file, std::string zedsvo_file, std::string zedposeavi_file, std::string posedata_file, bool isshow)
{

	string curdir = dirutil::GetCurrentWorkingDir();
	string mergeavi = zedservice::getCurZedservice()->startmergereportavi(hmavi_file, zedsvo_file, zedposeavi_file, posedata_file, isshow);
	string resbuffer;

	string strint = calcInt();

	resbuffer.append(strint);
	resbuffer.append("#");
	resbuffer.append("startmergereportavi");
	resbuffer.append("#");
	resbuffer.append(curdir).append("\\").append(mergeavi);
	return resbuffer;
}

//void zedservice::setsvoFIle(std::string _svofile)
//{
//	zedservice::getCurZedservice()->setsvo_File(_svofile);
//
//}
//
//std::string zedservice::getsvo_file()
//{
//	return zedservice::getCurZedservice()->getsvo_File();
//}

//std::string zedservice::getzedavi_file()
//{
//	return zedservice::getCurZedservice()->getzed_avi_File();
//}

std::string zedservice::getpose_avi_file()
{

	cout << "pose avi in here" << endl;
	string curdir = dirutil::GetCurrentWorkingDir();

	return curdir + "\\" + zedservice::getCurZedservice()->getpose_avi_File();
}

std::string zedservice::getpose_data_dir()
{

	cout << "pose data in here" << endl;
	
	return zedservice::getCurZedservice()->getpose_data_Dir();
}

//void zedservice::stoppposewithexitsys()
//{
//	//  qDebug() << "About to stop processing thread...";
//	if (mzedservice != NULL)
//	{
//
//	mzedservice->stoppposewithexitsys();
//	}
//    //processingThread->wait();
////    qDebug() << "Processing thread successfully stopped.";
//} // stopProcessingThread()

//void zedservice::startgetposeavidata()
//{
//
//
//	zedservice::getCurZedservice()->startgetposeavidata();
//}
//
//void zedservice::endgetposeavidata()
//{
//	zedservice::getCurZedservice()->endgetposeavidata();
//}


void zedservice::deleteProcessingThread()
{
    // Delete thread
    delete mzedservice;
} // deleteProcessingThread()


