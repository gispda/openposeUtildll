#include "openposeUtil.h"
#include <windows.h>
#include <string>
//#include <QDebug>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "flags.hpp"

//#include "control.h"
#include "utils.h"


#include "simlog.h"
#include "utils.hpp"

#include "dirutil.h"
//#include "vld.h"

//#include "PTMainWindow.h"

//DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
//" 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
//" low priority messages and 4 for important ones.");
//DEFINE_bool(disable_multi_thread,       false,          "It would slightly reduce the frame rate in order to highly reduce the lag. Mainly useful"
//" for 1) Cases where it is needed a low latency (e.g. webcam in real-time scenarios with"
//" low-range GPU devices); and 2) Debugging OpenPose when it is crashing to locate the"
//" error.");
//DEFINE_int32(profile_speed,             1000,           "If PROFILER_ENABLED was set in CMake or Makefile.config files, OpenPose will show some"
//" runtime statistics at this frame number.");
//// Producer
//DEFINE_string(image_dir,                "examples/media/",      "Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
//DEFINE_double(camera_fps,               30.0,           "Frame rate for the webcam (also used when saving video). Set this value to the minimum"
//" value between the OpenPose displayed speed and the webcam real frame rate.");
//// OpenPose
//DEFINE_string(model_folder,             "/media/zjg/workspace/Qt-demo/video/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
//DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
//" input image resolution.");
//DEFINE_int32(num_gpu,                   -1,             "The number of GPU devices to use. If negative, it will use all the available GPUs in your"
//" machine.");
//DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
//DEFINE_int32(keypoint_scale,            0,              "Scaling of the (x,y) coordinates of the final pose data array, i.e. the scale of the (x,y)"
//" coordinates that will be saved with the `write_keypoint` & `write_keypoint_json` flags."
//" Select `0` to scale it to the original source resolution, `1`to scale it to the net output"
//" size (set with `net_resolution`), `2` to scale it to the final output size (set with"
//" `resolution`), `3` to scale it in the range [0,1], and 4 for range [-1,1]. Non related"
//" with `scale_number` and `scale_gap`.");
//DEFINE_int32(number_people_max,         -1,             "This parameter will limit the maximum number of people detected, by keeping the people with"
//" top scores. The score is based in person area over the image, body part score, as well as"
//" joint score (between each pair of connected body parts). Useful if you know the exact"
//" number of people in the scene, so it can remove false positives (if all the people have"
//" been detected. However, it might also include false negatives by removing very small or"
//" highly occluded people. -1 will keep them all.");
//// OpenPose Body Pose
//DEFINE_bool(body_disable,               false,          "Disable body keypoint detection. Option only possible for faster (but less accurate) face"
//" keypoint detection.");
//DEFINE_string(model_pose,               "COCO",         "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
//"`MPI_4_layers` (15 keypoints, even faster but less accurate).");
//DEFINE_string(net_resolution,           "-1x368",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
//" decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
//" closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
//" any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
//" input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
//" e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
//DEFINE_int32(scale_number,              1,              "Number of scales to average.");
//DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
//" If you want to change the initial scale, you actually want to multiply the"
//" `net_resolution` by your desired initial scale.");
//// OpenPose Body Pose Heatmaps and Part Candidates
//DEFINE_bool(heatmaps_add_parts,         false,          "If true, it will fill op::Datum::poseHeatMaps array with the body part heatmaps, and"
//" analogously face & hand heatmaps to op::Datum::faceHeatMaps & op::Datum::handHeatMaps."
//" If more than one `add_heatmaps_X` flag is enabled, it will place then in sequential"
//" memory order: body parts + bkg + PAFs. It will follow the order on"
//" POSE_BODY_PART_MAPPING in `src/openpose/pose/poseParameters.cpp`. Program speed will"
//" considerably decrease. Not required for OpenPose, enable it only if you intend to"
//" explicitly use this information later.");
//DEFINE_bool(heatmaps_add_bkg,           false,          "Same functionality as `add_heatmaps_parts`, but adding the heatmap corresponding to"
//" background.");
//DEFINE_bool(heatmaps_add_PAFs,          false,          "Same functionality as `add_heatmaps_parts`, but adding the PAFs.");
//DEFINE_int32(heatmaps_scale,            2,              "Set 0 to scale op::Datum::poseHeatMaps in the range [-1,1], 1 for [0,1]; 2 for integer"
//" rounded [0,255]; and 3 for no scaling.");
//DEFINE_bool(part_candidates,            false,          "Also enable `write_json` in order to save this information. If true, it will fill the"
//" op::Datum::poseCandidates array with the body part candidates. Candidates refer to all"
//" the detected body parts, before being assembled into people. Note that the number of"
//" candidates is equal or higher than the number of final body parts (i.e. after being"
//" assembled into people). The empty body parts are filled with 0s. Program speed will"
//" slightly decrease. Not required for OpenPose, enable it only if you intend to explicitly"
//" use this information.");
//// OpenPose Face
//DEFINE_bool(face,                       false,          "Enables face keypoint detection. It will share some parameters from the body pose, e.g."
//" `model_folder`. Note that this will considerable slow down the performance and increse"
//" the required GPU memory. In addition, the greater number of people on the image, the"
//" slower OpenPose will be.");
//DEFINE_string(face_net_resolution,      "368x368",      "Multiples of 16 and squared. Analogous to `net_resolution` but applied to the face keypoint"
//" detector. 320x320 usually works fine while giving a substantial speed up when multiple"
//" faces on the image.");
//// OpenPose Hand
//DEFINE_bool(hand,                       false,          "Enables hand keypoint detection. It will share some parameters from the body pose, e.g."
//" `model_folder`. Analogously to `--face`, it will also slow down the performance, increase"
//" the required GPU memory and its speed depends on the number of people.");
//DEFINE_string(hand_net_resolution,      "368x368",      "Multiples of 16 and squared. Analogous to `net_resolution` but applied to the hand keypoint"
//" detector.");
//DEFINE_int32(hand_scale_number,         1,              "Analogous to `scale_number` but applied to the hand keypoint detector. Our best results"
//" were found with `hand_scale_number` = 6 and `hand_scale_range` = 0.4.");
//DEFINE_double(hand_scale_range,         0.4,            "Analogous purpose than `scale_gap` but applied to the hand keypoint detector. Total range"
//" between smallest and biggest scale. The scales will be centered in ratio 1. E.g. if"
//" scaleRange = 0.4 and scalesNumber = 2, then there will be 2 scales, 0.8 and 1.2.");
//DEFINE_bool(hand_tracking,              false,          "Adding hand tracking might improve hand keypoints detection for webcam (if the frame rate"
//" is high enough, i.e. >7 FPS per GPU) and video. This is not person ID tracking, it"
//" simply looks for hands in positions at which hands were located in previous frames, but"
//" it does not guarantee the same person ID among frames.");
//// OpenPose 3-D Reconstruction
//DEFINE_bool(3d,                         false,          "Running OpenPose 3-D reconstruction demo: 1) Reading from a stereo camera system."
//" 2) Performing 3-D reconstruction from the multiple views. 3) Displaying 3-D reconstruction"
//" results. Note that it will only display 1 person. If multiple people is present, it will"
//" fail.");
//DEFINE_int32(3d_min_views,              -1,             "Minimum number of views required to reconstruct each keypoint. By default (-1), it will"
//" require all the cameras to see the keypoint in order to reconstruct it.");
//DEFINE_int32(3d_views,                  1,              "Complementary option to `--image_dir` or `--video`. OpenPose will read as many images per"
//" iteration, allowing tasks such as stereo camera processing (`--3d`). Note that"
//" `--camera_parameters_folder` must be set. OpenPose must find as many `xml` files in the"
//" parameter folder as this number indicates.");
//// OpenPose Rendering
//DEFINE_int32(part_to_show,              0,              "Prediction channel to visualize (default: 0). 0 for all the body parts, 1-18 for each body"
//" part heat map, 19 for the background heat map, 20 for all the body part heat maps"
//" together, 21 for all the PAFs, 22-40 for each body part pair PAF.");
//DEFINE_bool(disable_blending,           false,          "If enabled, it will render the results (keypoint skeletons or heatmaps) on a black"
//" background, instead of being rendered into the original image. Related: `part_to_show`,"
//" `alpha_pose`, and `alpha_pose`.");
//// OpenPose Rendering Pose
//DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
//" rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
//" while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
//" more false positives (i.e. wrong detections).");
//DEFINE_int32(render_pose,               -1,             "Set to 0 for no rendering, 1 for CPU rendering (slightly faster), and 2 for GPU rendering"
//" (slower but greater functionality, e.g. `alpha_X` flags). If -1, it will pick CPU if"
//" CPU_ONLY is enabled, or GPU if CUDA is enabled. If rendering is enabled, it will render"
//" both `outputData` and `cvOutputData` with the original image and desired body part to be"
//" shown (i.e. keypoints, heat maps or PAFs).");
//DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
//" hide it. Only valid for GPU rendering.");
//DEFINE_double(alpha_heatmap,            0.7,            "Blending factor (range 0-1) between heatmap and original frame. 1 will only show the"
//" heatmap, 0 will only show the frame. Only valid for GPU rendering.");
//// OpenPose Rendering Face
//DEFINE_double(face_render_threshold,    0.4,            "Analogous to `render_threshold`, but applied to the face keypoints.");
//DEFINE_int32(face_render,               -1,             "Analogous to `render_pose` but applied to the face. Extra option: -1 to use the same"
//" configuration that `render_pose` is using.");
//DEFINE_double(face_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to face.");
//DEFINE_double(face_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to face.");
//// OpenPose Rendering Hand
//DEFINE_double(hand_render_threshold,    0.2,            "Analogous to `render_threshold`, but applied to the hand keypoints.");
//DEFINE_int32(hand_render,               -1,             "Analogous to `render_pose` but applied to the hand. Extra option: -1 to use the same"
//" configuration that `render_pose` is using.");
//DEFINE_double(hand_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to hand.");
//DEFINE_double(hand_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to hand.");
//// Result Saving
//DEFINE_string(write_images,             "",             "Directory to write rendered frames in `write_images_format` image format.");
//DEFINE_string(write_images_format,      "png",          "File extension and format for `write_images`, e.g. png, jpg or bmp. Check the OpenCV"
//" function cv::imwrite for all compatible extensions.");
//DEFINE_string(write_video,              "",             "Full file path to write rendered frames in motion JPEG video format. It might fail if the"
//" final path does not finish in `.avi`. It internally uses cv::VideoWriter.");
//DEFINE_string(write_json,               "",             "Directory to write OpenPose output in JSON format. It includes body, hand, and face pose"
//" keypoints (2-D and 3-D), as well as pose candidates (if `--part_candidates` enabled).");
//DEFINE_string(write_coco_json,          "",             "Full file path to write people pose data with JSON COCO validation format.");
//DEFINE_string(write_heatmaps,           "",             "Directory to write body pose heatmaps in PNG format. At least 1 `add_heatmaps_X` flag"
//" must be enabled.");
//DEFINE_string(write_heatmaps_format,    "png",          "File extension and format for `write_heatmaps`, analogous to `write_images_format`."
//" For lossless compression, recommended `png` for integer `heatmaps_scale` and `float` for"
//" floating values.");
//DEFINE_string(write_keypoint,           "",             "(Deprecated, use `write_json`) Directory to write the people pose keypoint data. Set format"
//" with `write_keypoint_format`.");
//DEFINE_string(write_keypoint_format,    "yml",          "(Deprecated, use `write_json`) File extension and format for `write_keypoint`: json, xml,"
//" yaml & yml. Json not available for OpenCV < 3.0, use `write_keypoint_json` instead.");
//DEFINE_string(write_keypoint_json,      "",             "(Deprecated, use `write_json`) Directory to write people pose data in JSON format,"
//" compatible with any OpenCV version.");

// Configuration header file
//构造函数，指定缓冲区，和图片宽度、高度
openposeUtil::openposeUtil()
{
	// Initialize variables



	bsaveSvo = false;
	bsavePose = false;


	bsaveZed = false;


	m_bfilter = true;

	//stdsvoFile = getNowTime()+".svo";
	stdsvo_File = "";

	stdpose_data_Dir = "";

	stdzed_avi_File = "";

	stdpose_avi_File = "";


	stdmerge_avi_File = "";
	// qRegisterMetaType<vector<pair<float,float>>>("vector<pair<float,float>>");

	filterRect.x = -1;
	filterRect.y = -1;
	filterRect.width = -1;
	filterRect.height = -1;


	 //outputVideo = NULL;

	bshow = true;


	initDevice();

	maxframecount = 300;

	m_svodescIdx = 0;
	m_svoImgIdx = 0;

	poseimgidx = 0;
	isrootwrite = false;


//	poseimgnum=0;
	zedimgnum=0;
	hkimgnum=0;
	jsonnum=0;



	m_inputDataType = InputDataType::SVO_TXT;

	

	//zedposedataimg = new cv::Mat[10];
	builder.settings_["emitUTF8"] = true;

	SimLog::Instance().InitSimLog("openpose", "openposeutilinfo.txt",true);



	isopen = false;

	svoTxtPos = 0;

	m_svotxtDir = "D:\\project\\ParatroopersTraining\\data\\images3\\";

	m_blastsvo = false;


	personIdx = -1;

	if (m_inputDataType == InputDataType::SVO_TXT)
	{
		infile.open(m_svotxtDir+"filelist.txt", std::ifstream::in);

		if (infile.is_open())
		{
			isopen = true;




			createSvodescmap();
			for (auto& kv : m_svoimgmap) {
				cout << kv.first << " has value " << kv.second << endl;

			}
			cout << "----------------" << endl;
			for (auto& kv : m_svopersonmap) {
				cout << kv.first << " has value " << kv.second << endl;

			}
			cout << "----------------" << endl;

			cout << "open filelist.txt okay" << endl;
		}
		else
		{
			isopen = false;
			cout << "open filelist.txt error" << endl;
		}
	}

	//ImgIdx = -1;

	//testData(340);
	//for(int i=0;i<897;i++)
	//getFilterRect(i);
	//testData(341);
} // openposeUtil constructor

void openposeUtil::testData(int imgidx)
{
	readImgRectFromText(imgidx);
	op::Rectangle<int> _rect;
	int width = 1920, height = 1080;

	int ydiv = 1080 / 20;

	int temp = 0, suby = 0;
	int maxidx = -1;

	int eqnum = 0;

	std::map<int, op::Rectangle<int>> posrect2;
	posrect2.empty();
	for (int i = 0; i < bodyposrect.size(); i++)
	{
		_rect = bodyposrect[i];

		temp = bodyposrect[i].y;

		temp = temp / ydiv;

		if (suby <= temp)
		{
			suby = temp;
			//maxidx = i;
			posrect2.insert(make_pair(i, _rect));
		}
		cout << "_rect " <<i<<" "<< _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;
	}
	int subx = 0;

	for (auto& kv : posrect2) {
		//count << kv.first << " has value " << kv.second << endl;
		
		_rect = kv.second;
		cout << "_rect " << " " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;
		temp = _rect.x;
		if (subx < temp)
		{
			subx = temp;
			maxidx = kv.first;
		}
	}

	if (maxidx != -1)
	{
		_rect = bodyposrect[maxidx];
		cout << "select _rect " << " " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;

		
		//std::string strfile;
		//std::string temp;
		//std::stringstream ss,ss1;
		//ss << ImgIdx;

		////temp = std::string::
		//ss1 << maxidx;
		//strfile.append("D:\\project\\ParatroopersTraining\\data\\images2\\").append(ss.str()).append("_").append(ss1.str()).append(".jpg");
  //      
		//cout << "file name is " << strfile << endl;
		//cv::Mat image = imread(strfile);

		//cv::Rect rect(_rect.x, _rect.y, _rect.width, _rect.height);

		//cv::Rect rect1(5, 5, 90, 300);
		//cv::rectangle(image, rect, Scalar(255, 0, 0), 1, LINE_8, 0);
		//cv::rectangle(image, rect1, Scalar(255, 0, 0), 1, LINE_8, 0);
		//imshow("12345", image);
		//waitKey(0);

	}


	
}

void openposeUtil::getFilterRect(int imgidx)
{
	readImgRectFromText(imgidx);
	op::Rectangle<int> _rect;
	int width = 1920, height = 1080;

	int ydiv = height / 10;

	int temp = 0, suby = 0;
	int maxidx = -1;

	int eqnum = 0;

	std::map<int, op::Rectangle<int>> posrect2;
	//posrect2.empty();
	for (int i = 0; i < bodyposrect.size(); i++)
	{
		_rect = bodyposrect[i];

		temp = bodyposrect[i].y+ bodyposrect[i].height;

		temp = temp / ydiv;


		cout << "rect temp " << temp << endl;
		if (suby < temp)
		{
			suby = temp;
			//maxidx = i;
			posrect2.clear();
			posrect2.insert(make_pair(i, _rect));

			cout << "rect suby " << suby << endl;
		}
		else if (suby == temp)
		{
			posrect2.insert(make_pair(i, _rect));
			cout << "same suby " << suby << endl;
		}
		//cout << "_rect " << i << " " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;
	}
	int subx = 0;
	int ii = 0,jj = 0;
	for (auto& kv : posrect2) {
		//count << kv.first << " has value " << kv.second << endl;

		_rect = kv.second;
		cout << "_rect " << " " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;
		temp = _rect.x+ _rect.width;
		if (subx < temp)
		{
			subx = temp;
			maxidx = kv.first;
			cout << "same _rect has  " << " " << jj << endl;
			jj++;

		}
		ii++;
	}

	//for (auto& kv : bodyposrectfile) {
	//	cout << kv.first << " has value " << kv.second << endl;

	//	/*_rect = kv.second;
	//	cout << "_rect " << " " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;
	//	temp = _rect.x + _rect.width;
	//	if (subx < temp)
	//	{
	//		subx = temp;
	//		maxidx = kv.first;
	//		cout << "same _rect has  " << " " << jj << endl;
	//		jj++;

	//	}
	//	ii++;*/
	//}

	if (maxidx != -1)
	{
		filterRect = bodyposrect[maxidx];
		cout << "select _rect " << " " << filterRect.x << "," << filterRect.y << "," << filterRect.width << "," << filterRect.height << endl;

		//std::string filefullname = "D:\\project\\ParatroopersTraining\\data\\images2\\"+ImgIdx+"_"+
		//std::string strfile;
		//std::string temp;
		//std::stringstream ss, ss1;
		//ss << ImgIdx;

		//temp = std::string::
		//ss1 << maxidx;
		//strfile.append("D:\\project\\ParatroopersTraining\\data\\images2\\").append(ss.str()).append("_").append(ss1.str()).append(".jpg");

	//	cout << "file name is " << strfile << endl;

	//	cout << "file size is " << bodyposrectfile.size() << endl;

		//std::string strfilenew;
		//strfilenew = bodyposrectfile[maxidx];

		///strfilenew.append("D:\\project\\ParatroopersTraining\\data\\images2\\").append(bodyposrectfile[maxidx]);
		//cv::Mat image = imread(strfilenew);

	//	cv::Rect rect(_rect.x, _rect.y, _rect.width, _rect.height);

		//cv::Rect rect1(5, 5, 90, 300);
		//cv::rectangle(image, rect, Scalar(255, 0, 0), 1, LINE_8, 0);
		//cv::rectangle(image, rect1, Scalar(255, 0, 0), 1, LINE_8, 0);
		//imshow("12345", image);
		//waitKey(0);

	}
}

openposeUtil::~openposeUtil()
{
	if (_manpose != NULL)
	{
		delete _manpose;

		logInfo("clean manpose");
	}
	if (outposeVideo.isOpened())
	{
	outposeVideo.release();
	///delete outputVideo;
	logInfo("clean outposevideo");
   }

	m_svoimgmap.clear();  //
	m_svopersonmap.clear();  //
	stoppposewithexitsys();

	SimLog::Instance().EndLog();
} // openposeUtil destructor

//处理线程的主功能函数


//停止处理线程
void openposeUtil::stopposeservice()
{
	//    stoppedMutex.lock();
	if (quit == true)
	{
		cout << "re stoped!!" << endl;
		return ;
	}
	


	if (bsavePose)
	{

		bsavePose = false;

		outposeVideo.release();

		sl::sleep_ms(20);
	}

	quit = true;
	bposeavifinished = true;
	cout << "start clean resource" << endl;
	//viewer.exit();

	
	cout << "over savepose avi res" << endl;



	if (zed.isOpened())
	{

		cout << "close zed " << endl;
		zed.close();



		//openpose_callback.detach();
		///zed_callback.detach();


	}
	else
		cout << "zed not need close" << endl;

	/*if (openpose_callback.joinable())
		openpose_callback.join();

	if (zed_callback.joinable())
		zed_callback.join();*/




	//zed_callback.join();


	//cout << "over zed callback avi res" << endl;
	///openpose_callback.join();

	//cout << "over openpose callback avi res" << endl;
	//saveposeavi_callback.join();


	//openpose_callback.join();
	cout << "stop openpose callback" << endl;


	

	//cout << "22233333333333" << endl;
	

	


	//cout << "11155555555555555555555555" << endl;


	//savemanpose();



	

	//bsaveZed = false;
	//outposeVideo.release();
	//outzedVideo.release();
	//delete outputVideo;
	//outputVideo = NULL;


	logInfo("over all pose avi");
	//reinit();

	//logInfo("over recording pose avi");
	///return getpose_avi_File();


	//cout << "fffffffffffffffffff" << endl;

	//savesvo_callback.join();
	//saveposeavi_callback.join();

	//return this->getsvo_File();


	//logInfo("exit Processing Thread");

  //  stoppedMutex.unlock();
} // stopopenposeUtil()



void openposeUtil::logInfo(std::string info)
{
	//QString ss = Utils::fromStringToQString(info);
	//logInfo(ss);

	LDebug(info);
}

void openposeUtil::logInfo(int info)
{
	///QString ss = QString::number(info);
	LDebug("number is :{}", info);
}

void openposeUtil::logInfo(float info)
{
	//QString ss = QString::number(info);
	LDebug("number is :{}", info);
}

void openposeUtil::logInfo(double info)
{
	//QString ss = QString::number(info);
	LDebug("number is :{}", info);
}

void openposeUtil::logInfo(bool info)
{
	//QString ss = QString::number(info);
	LDebug("number is :{}", info);
}






void openposeUtil::logInfo(AngleInfo angle)
{
	//std::string winfo;
	//winfo.append(angle.desc);


	std::string info;

	info.append(angle.desc).append(":").append(floatTostring(angle.angle.angle, 5)).append(" x,y:").append(floatTostring(angle.x, 5)).append(",").append(floatTostring(angle.y, 5)).append(" over");

	LDebug(info);
}

void openposeUtil::logInfo(char* info)
{
	//QString ss(info);
	LDebug(info);
}

void openposeUtil::logInfo(sl::float3 info)
{
	///loginfo("sl")
	string qinfo;

	qinfo.append("vector or point is ").append("x:").append(floatTostring(info.x, 5)).append(" y:").append(floatTostring(info.y, 5)).append(" z:").append(floatTostring(info.z, 5)).append(" over");

	logInfo(qinfo);
}

void openposeUtil::logInfo(sl::float4 info)
{

	string qinfo;

	qinfo.append("vector or point is ").append("x:").append(floatTostring(info.x, 5)).append(" y:").append(floatTostring(info.y, 5)).append(" z:").append(floatTostring(info.z, 5)).append(" over");

	logInfo(qinfo);
}

void openposeUtil::logInfo(sl::Transform info)
{
	string qinfo;
	qinfo.append("Matrix  is ").append("{{").append(floatTostring(info.m[0], 5)).append(",").
		append(floatTostring(info.m[1], 5)).append(",").
		append(floatTostring(info.m[2], 5)).append(",").
		append(floatTostring(info.m[3], 5)).append("}").
		append("{").append(floatTostring(info.m[4], 5)).append(",").
		append(floatTostring(info.m[5], 5)).append(",").
		append(floatTostring(info.m[6], 5)).append(",").
		append(floatTostring(info.m[7], 5)).append("}").
		append("{").append(floatTostring(info.m[8], 5)).append(",").
		append(floatTostring(info.m[9], 5)).append(",").
		append(floatTostring(info.m[10], 5)).append(",").
		append(floatTostring(info.m[11], 5)).append("}").
		append("{").append(floatTostring(info.m[12], 5)).append(",").
		append(floatTostring(info.m[13], 5)).append(",").
		append(floatTostring(info.m[14], 5)).append(",").
		append(floatTostring(info.m[15], 5)).append("}}");
	logInfo(qinfo);
}

void openposeUtil::logInfo(sl::Translation info)
{
	string qinfo;
	qinfo.append("Translation Matrix  is ").append(" tx:").append(floatTostring(info.tx, 5)).append(" ty:").append(floatTostring(info.ty, 5)).append(" tz:").append(floatTostring(info.tz, 5)).append(" over");;
	logInfo(qinfo);
}

void openposeUtil::logInfo(int vidx[4])
{
	string qinfo;
	qinfo.append("body pose index each part is ").append(floatTostring(vidx[0], 5)).append("-").append(floatTostring(vidx[1], 5)).
		append(":").append(floatTostring(vidx[2], 5)).append("-").append(floatTostring(vidx[3], 5)).append(" over");

	logInfo(qinfo);
}

void openposeUtil::startzeddevice()
{
}

//void openposeUtil::configureptmainwindow(PTMainWindow* ppm)
//{
//
//	pptmainwindow = ppm;
//}

void openposeUtil::initLogParameter(int argc, char* arcgv[])
{
	gflags::ParseCommandLineFlags(&argc, &arcgv, true);

	//google::InitGoogleLogging(arcgv[0]);

	FLAGS_log_dir = ".";

	*arcgv = arcgv[0];
	//LOG(INFO) << arcgv[0];
	//LOG(INFO) << "一次初始化google logger";

	//logInfo("一次初始化google logger");
}

void openposeUtil::drawText(cv::Mat * image, AngleInfo angle, Body body)
{
	//image->setTo(cv::Scalar(100, 0, 0));

	//设置绘制文本的相关参数
	std::string text = "";

	//获取文本框的长宽


	//将文本框居中绘制
	if (isnan(angle.angle.angle))
	{
		return;
	}

	text = doubleTostring(angle.angle.angle, 8);





	cv::Point origin;
	origin.x = angle.x;
	origin.y = angle.y;
	int tlen = gettextlen(*image, text.data(), origin);
	if (body == BODY_LEFT)
	{
		origin.x = origin.x - tlen;
	}

	try {
		cvdrawText::putTextZH(*image, text.data(), origin, cv::Scalar(0, 255, 255), 12, "Arial", true, true);
	}
	catch (Exception ex)
	{
		logInfo(ex.what());
		cout << ex.what() << endl;
	}


}

void openposeUtil::drawText(cv::Mat * image, std::string _text, cv::Point origin, int fonth, int cR, int cG, int cB)
{

	cvdrawText::putTextZH(*image, _text.data(), origin, cv::Scalar(cR, cG, cB), fonth, "Arial", true, true);
}

void openposeUtil::initDevice()
{


	thres_score = 0.6;


	//ReInitResolution();


	need_new_image = true;
	ready_to_start = false;
	model_kp_number = 25;

	bool quit = false;

	bposeavifinished = false;

	bcomposeavifinished = false;
	//	pptmainwindow = NULL;
	_manpose = new manpose();

	//logInfo("start configkeydata");
	_manpose->configKeyData();
	//logInfo("configkeydata");


	//int vidx = _manpose->getbody_part_index("right_hipknee_plane", SECOND_SEC_START);

	//logInfo("right_hipknee_plane");
	//logInfo(vidx);

}

void openposeUtil::ReInitResolution(Resolution _image_size)
{
	poseImage_width = _image_size.width;
	poseImage_height = _image_size.height;
	image_width = _image_size.width;;
	image_height = _image_size.height;;

	simage_width = _image_size.width;;
	simage_height = _image_size.height;;
}




void openposeUtil::startposeserviceonlinezed()
{
	// Set configuration parameters for the ZED
	InitParameters initParameters;
	initParameters.camera_resolution = RESOLUTION::HD720;
	initParameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Might be GPU memory intensive combine with openpose
	initParameters.coordinate_units = unit;
	initParameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	initParameters.sdk_verbose = 0;
	initParameters.depth_stabilization = true;
	initParameters.svo_real_time_mode = 1;

	if (std::string(FLAGS_svo_path).find(".svo") && !std::string(FLAGS_svo_path).empty()) {
		logInfo("Opening ");
		logInfo(FLAGS_svo_path);;
		initParameters.input.setFromSVOFile(std::string(FLAGS_svo_path).c_str());
	}

	//logInfo("1111111111111111111111111111111");
	// Open the camera
	ERROR_CODE err = zed.open(initParameters);
	if (err != sl::ERROR_CODE::SUCCESS) {
		//logInfo(err);
		logInfo("open camera error");
		zed.close();
		return; // Quit if an error occurred
	}
	logInfo("succeed open zed.");


	Resolution image_size;
	


	image_size = zed.getCameraInformation().camera_configuration.resolution;
	ReInitResolution(image_size);

	if (FLAGS_estimate_floor_plane)
		initFloorZED(zed);




	//logInfo("2222222222222222222222222222222");
	// Initialize OpenGL viewer
//	viewer.init();

	// init OpenPose
	//cout << "OpenPose : loading models..." << endl;

	logInfo("OpenPose : loading models...");
	// ------------------------- INITIALIZATION -------------------------
	// Read Google flags (user defined configuration)
	outputSize = op::flagsToPoint(std::string(FLAGS_output_resolution).c_str(), "-1x-1");
	netInputSize = op::flagsToPoint(std::string(FLAGS_net_resolution).c_str(), "-1x368");

	cout << netInputSize.x << "x" << netInputSize.y << endl;
	netOutputSize = netInputSize;
	poseModel = op::flagsToPoseModel(std::string(FLAGS_model_pose).c_str());

	//logInfo("fffffffffffffffffffffffffffffffffffffffffffffff");

	if (FLAGS_model_pose == "COCO") model_kp_number = 18;
	else if (FLAGS_model_pose.find("MPI") != std::string::npos) model_kp_number = 15;
	else if (FLAGS_model_pose == "BODY_25") model_kp_number = 25;


	//logInfo("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG");
	// Check no contradictory flags enabled
	if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.) op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
	if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1) op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.", __LINE__, __FUNCTION__, __FILE__);



	//logInfo("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
	// Start ZED callback
	startZED();
	//logInfo("end startZed");
	startOpenpose();
	///logInfo("IIIIIIIIIIII");
	// Set the display callback
	//glutCloseFunc(clean);
	//glutMainLoop();
}
void openposeUtil::startzedopenpose()
{

	quit = false;
//	zed_callback = std::thread(&openposeUtil::runzedopenpose, this);
}

void openposeUtil::startgetposeavidata()
{
	if (bsavePose)
		return;



	savezedavi_callback = std::thread(&openposeUtil::saveposeavi, this);

	//if (bsaveZed)
		//return;



	//saveposeavi_callback = std::thread(&openposeUtil::savezedavi, this);





	quit = false;
	savesvo_callback = std::thread(&openposeUtil::savesvo, this);

	//bool retval = saveSvo(svofile);




}
void openposeUtil::endgetposeavidata()
{


	if (bsavePose == false)
		return;

	//savemanpose();
	bsavePose = false;
	outposeVideo.release();

	//bsaveZed = false;
	//outposeVideo.release();
	//outzedVideo.release();
	//delete outputVideo;
	//outputVideo = NULL;
	reinit();

	logInfo("over all pose avi");

	if (bsaveSvo == false)
		return;

	/*if (zed.isOpened())
	{
		cout << "fsfdsfsdf" << endl;
		zed.close();
	}
	else
		cout << "sdffsdfsdf" << endl;*/

	bsaveSvo = false;


	cout << "fffffffffffffffffff" << endl;

	//savesvo_callback.join();
	//saveposeavi_callback.join();
	logInfo("set bool Save svo false");
	//return this->getsvo_File();

	savesvo_callback.join();
	saveposeavi_callback.join();


}
std::string openposeUtil::UTF8ToGB(const std::string & str)
{

	string result;
	WCHAR* strSrc;
	LPSTR szRes;

	int i = MultiByteToWideChar(CP_UTF8, 0, str.data(), -1, NULL, 0);
	strSrc = new WCHAR[i + 1];
	MultiByteToWideChar(CP_UTF8, 0, str.data(), -1, strSrc, i);

	i = WideCharToMultiByte(CP_ACP, 0, strSrc, -1, NULL, 0, NULL, NULL);
	szRes = new CHAR[i + 1];
	WideCharToMultiByte(CP_ACP, 0, strSrc, -1, szRes, i, NULL, NULL);

	result = szRes;
	delete[]strSrc;
	delete[]szRes;
	return result;

}
std::wstring openposeUtil::s2ws(const std::string & s)
{
	int len;
	int slength = (int)s.length() + 1;
	len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
	wchar_t* buf = new wchar_t[len];
	MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	std::wstring r(buf);
	delete[] buf;
	return r;
}
std::string openposeUtil::ws2s(const std::wstring & s)
{
	int len;
	int slength = (int)s.length() + 1;
	len = WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, 0, 0, 0, 0);
	char* buf = new char[len];
	WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, buf, len, 0, 0);
	std::string r(buf);
	delete[] buf;
	return r;
}
std::string openposeUtil::replace_utf8_escape_sequences(const std::string & str)
{

	std::string result;
	std::string::size_type first = 0;
	std::string::size_type last = 0;
	while (true) {
		// Find an escape position
		last = str.find("\\u", last);
		if (last == std::string::npos) {
			result.append(str.begin() + first, str.end());
			break;
		}

		// Extract a 4 digit hexadecimal
		const char* hex = str.data() + last + 2;
		char* hex_end;
		std::uint_fast32_t code = std::strtoul(hex, &hex_end, 16);
		std::string::size_type hex_size = hex_end - hex;

		// Append the leading and converted string
		if (hex_size != 4) last = last + 2 + hex_size;
		else {
			result.append(str.begin() + first, str.begin() + last);
			try {
				utf8::utf16to8(&code, &code + 1, std::back_inserter(result));
			}
			catch (const utf8::exception&) {
				// Error Handling
				result.clear();
				break;
			}
			first = last = last + 2 + 4;
		}
	}
	return result;
}
std::string openposeUtil::getzedcurimg()
{

	//cout << "-----------------111111111111----" << endl;
	std::string tmpimg = dirutil::GetCurrentWorkingDir() + "\\" + getNowTime() + ".bmp";
	logInfo(tmpimg);
	//cout << tmpimg << endl;
	//data_out_mtx.lock();


	cv::imwrite(tmpimg, image_ocv);



	return tmpimg;
	//return "";
}
void openposeUtil::startRecordPose()
{
	if (bsavePose)
		return;



	savezedavi_callback = std::thread(&openposeUtil::saveposeavi, this);
}
std::string openposeUtil::endRecordPose()
{
	if (bsavePose == false)
		return "";

	//savemanpose();
	bsavePose = false;
	outposeVideo.release();
	outzedVideo.release();
	//delete outputVideo;
	//outputVideo = NULL;
	reinit();

	logInfo("over recording pose avi");
	return getpose_avi_File();
}
bool openposeUtil::startSavezedSvo()
{


	//bool retflag;

	quit = false;
	savesvo_callback = std::thread(&openposeUtil::savesvo, this);

	//bool retval = saveSvo(svofile);

	return true;
}
bool openposeUtil::savesvo()
{
	//retflag = true;
	// Enable recording with the filename specified in argument
	sl::String svoFile(getsvo_File().data());



	InitParameters init_parameters;
	init_parameters.camera_resolution = RESOLUTION::HD2K;
	init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
	init_parameters.depth_stabilization = true;
	//InitParameters initParameters;
	//initParameters.camera_resolution = RESOLUTION::VGA;
	//initParameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Might be GPU memory intensive combine with openpose
	//initParameters.coordinate_units = unit;
	//initParameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	//initParameters.sdk_verbose = 0;
	//initParameters.depth_stabilization = true;
	////initParameters.svo_real_time_mode = 1;
	cout << "open zed start11111111111111111111111111" << endl;

	ERROR_CODE zed_open_state = zed.open(init_parameters);
	if (zed_open_state != ERROR_CODE::SUCCESS) {
		cout << "open zed error" << endl;
		return false;
	}


	ERROR_CODE returned_state = zed.enableRecording(RecordingParameters(svoFile, SVO_COMPRESSION_MODE::H264));

	if (returned_state != ERROR_CODE::SUCCESS) {

		cout << "111111111111111111111111" << endl;
		logInfo("Recording ZED : ERROR");
		zed.close();
		return false;
	}

	int  frames_recorded = 0;

	bsaveSvo = true;
	cout << "22222222222222222222222222222" << endl;
	while (bsaveSvo) {
		if (zed.grab() == ERROR_CODE::SUCCESS) {
			// Each new frame is added to the SVO file
			sl::RecordingStatus state = zed.getRecordingStatus();
			if (state.status)
				frames_recorded++;
			//cout << "33333333333333333333333333" << endl;
			logInfo("Frame count: " + to_string(frames_recorded));
		}
	}

	// Stop recording
	zed.disableRecording();

	cout << "2444444444444444444444444444444" << endl;
	logInfo("over recording Svo");
	//retflag = false;
	return true;
}



bool openposeUtil::saveposeavi()
{


	bsavePose = true;
	//logInfo("------------------------------------+++++");


	int ii = VideoWriter::fourcc('M', 'J', 'P', 'G');
	logInfo("++++++++++++++++++++++++++++++++++++");
	//logInfo(ii);
	//outputVideo = new VideoWriter(getpose_avi_File(), VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(672, 376),true);

	logInfo(image_height);
	logInfo(image_width);


	outposeVideo.release();
	outposeVideo.open(getpose_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 10, cv::Size(image_width, image_height), true);
	//outzedVideo.open(getzed_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 10, cv::Size(image_width, image_height), true);

	if (!outposeVideo.isOpened())
	{
		logInfo("codec failed\n");

		cout<<"codec failed\n"<<endl;
		return false;
	}
	else
	{
		logInfo("create pose avi ");
		cout << "create pose avi" << endl;
	}
	//fout.open(getpose_data_File().c_str(), std::ios::app);

	///json_root["bodyposes"] = 111111;


	//logInfo("-----------------------------sfdddddddddddddddddddd-------+++++");
	// Stop recording
	//zed.disableRecording();

	//retflag = false;
	return true;

}
bool openposeUtil::appendonejson()
{
	

	Json::Value oneposes, bodypos, permpos, frame,framposidx;        // 根节点

	//oneposes["id"] = getNowTime(); // 根节点下"name"对应的值"HaKing"
	//root["age"] = 24;        // 根节点下"age"对应的值24

	Json::Value array, arraypos, arraypos1;

	Json::Value item, ItemPos, permitem;

	AngleInfo angle;

	//Json::Valu

	angle = _manpose->getang_plane1_left_right();




	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "NONE";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}

	//item["px"] = angle.pos.x;
	//item["py"] = angle.pos.y;
	//item["pz"] = angle.pos.z;



	angle = _manpose->getang_plane2_before_after();

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "NONE";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}


	angle = _manpose->getang_midhip_plane_normal();

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "NONE";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_hipknee_midhip(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_hipknee_midhip(BODY_RIGHT);



	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//----------------------------------------


	angle = _manpose->getang_hipknee_plane_normal(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_hipknee_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------


	angle = _manpose->getang_kneeankle_plane_normal(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_kneeankle_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------

	angle = _manpose->getang_kneeankle_hipknee(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_kneeankle_hipknee(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------

	angle = _manpose->getang_anklebigtoe_kneeankle(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_anklebigtoe_kneeankle(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------

	angle = _manpose->getang_anklebigtoe_plane_normal(BODY_LEFT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_anklebigtoe_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------
	//---------------------------------

	angle = _manpose->getang_heel_plane_normal(BODY_LEFT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;
		//item["perm"] = angle.angle.angle/90;


		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_heel_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;
		//item["perm"] = angle.angle.angle / 90;

		array.append(item); // append()以数组的形式添加
	}


	//---------------------------------






	oneposes["bodypose"] = array;


	///-----------------------------------------
	angle = _manpose->getang_midhip_plane_normal();

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		permitem["headmidang"] = angle.angle.angle;
		permitem["headmidangxoy"] = angle.angle.anglexoy;
		permitem["headmidangyoz"] = angle.angle.angleyoz;

	}


	///---------
	angle = _manpose->getang_kneeankle_hipknee(BODY_LEFT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		permitem["lefthipknee"] = angle.angle.angle;
		permitem["lefthipkneexoy"] = angle.angle.anglexoy;
		permitem["lefthipkneeyoz"] = angle.angle.angleyoz;
		//item["p
	}

	angle = _manpose->getang_kneeankle_hipknee(BODY_RIGHT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		permitem["righthipknee"] = angle.angle.angle;

		permitem["righthipkneexoy"] = angle.angle.anglexoy;
		permitem["righthipkneeyoz"] = angle.angle.angleyoz;
		//item["p
	}
	permitem["twoknee"] = _manpose->twokneev;
	permitem["twoheelv"] = _manpose->twoheelv;
	arraypos1.append(permitem);

	permpos["perm"] = arraypos1;
	////-------------------------------------------





	/*std::vector<int> partsLink;
	partsLink = {
		0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 8, 12, 12,
		13, 13, 14, 14, 19, 19, 20, 14, 21, 8, 9, 9, 10, 10, 11, 11, 24,
		11, 22, 22, 23, 0, 16, 0, 15, 15, 17, 16, 18
	};*/

	sl::double3 jointpos;
	for (int part = 0; part < 25; part++) {


		jointpos = _manpose->bodyjointposmap[part];

		ItemPos["id"] = part;
		ItemPos["x"] = jointpos.x;
		ItemPos["y"] = jointpos.y;
		ItemPos["z"] = jointpos.z;


		arraypos.append(ItemPos); // append()以数组的形式添加
		//if(isnan(jointpos.x) || isnan(jointpos.y) || isnan(jointpos.z))

		//	v1 = keypoints_position[partsLink[part]];
		//	v2 = keypoints_position[partsLink[part + 1]];

		//	IsUseful(v1, v2, center_gravity);

	}
	bodypos["xyzpos"] = arraypos;
	

	framposidx["frameidx"] = _manpose->frameidx;

	frame.append(framposidx);
	frame.append(oneposes);
	frame.append(bodypos);
	frame.append(permpos);
	
	//totaljsons["frame"] = _manpose->frameidx;
	totaljsons["people"].append(frame);

	//Json::StreamWriterBuilder  builder;
	//builder.settings_["emitUTF8"]= true;

	

	//writer->write(json_bodyls, &fout);

	logInfo("append over one frame json data");
}
void openposeUtil::writeJson()
{
	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

	writer->write(totaljsons, &ffout);
	ffout.close();
}
void openposeUtil::addmanpose()
{


	fout.open(getpose_data_Dir().c_str() + std::to_string(m_svoImgIdx) + ".json", std::ios::app);

	Json::Value oneposes,bodypos,permpos,root;        // 根节点
	
	//oneposes["id"] = getNowTime(); // 根节点下"name"对应的值"HaKing"
	//root["age"] = 24;        // 根节点下"age"对应的值24

	Json::Value array,arraypos,arraypos1;

	Json::Value item,ItemPos,permitem;

	AngleInfo angle;

	//Json::Valu

	angle = _manpose->getang_plane1_left_right();



	
	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{	
	item["LEFTRIGHT"] = "NONE";
	item["angle"] = angle.angle.angle;
	item["anglexoy"] = angle.angle.anglexoy;
	item["angleyoz"] = angle.angle.angleyoz;
	item["x"] = angle.x;
	item["y"] = angle.y;
	item["desc"] = angle.desc;
	item["descxoy"] = angle.descxoy;
	item["descyoz"] = angle.descyoz;

	array.append(item); // append()以数组的形式添加
	}

	//item["px"] = angle.pos.x;
	//item["py"] = angle.pos.y;
	//item["pz"] = angle.pos.z;

	

	angle = _manpose->getang_plane2_before_after();

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "NONE";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	

	angle = _manpose->getang_midhip_plane_normal();

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "NONE";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_hipknee_midhip(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_hipknee_midhip(BODY_RIGHT);



	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//----------------------------------------


	angle = _manpose->getang_hipknee_plane_normal(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_hipknee_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------


	angle = _manpose->getang_kneeankle_plane_normal(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_kneeankle_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------

	angle = _manpose->getang_kneeankle_hipknee(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_kneeankle_hipknee(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------

	angle = _manpose->getang_anklebigtoe_kneeankle(BODY_LEFT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_anklebigtoe_kneeankle(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------

	angle = _manpose->getang_anklebigtoe_plane_normal(BODY_LEFT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_anklebigtoe_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;

		array.append(item); // append()以数组的形式添加
	}
	//---------------------------------
	//---------------------------------

	angle = _manpose->getang_heel_plane_normal(BODY_LEFT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "LEFT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;
		//item["perm"] = angle.angle.angle/90;
		

		array.append(item); // append()以数组的形式添加
	}
	angle = _manpose->getang_heel_plane_normal(BODY_RIGHT);

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		item["LEFTRIGHT"] = "RIGHT";
		item["angle"] = angle.angle.angle;
		item["anglexoy"] = angle.angle.anglexoy;
		item["angleyoz"] = angle.angle.angleyoz;
		item["x"] = angle.x;
		item["y"] = angle.y;
		item["desc"] = angle.desc;
		item["descxoy"] = angle.descxoy;
		item["descyoz"] = angle.descyoz;
		//item["perm"] = angle.angle.angle / 90;

		array.append(item); // append()以数组的形式添加
	}


	//---------------------------------






	oneposes["bodypose"] = array;


	///-----------------------------------------
	angle = _manpose->getang_midhip_plane_normal();

	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		permitem["headmidang"] = angle.angle.angle;
		permitem["headmidangxoy"] = angle.angle.anglexoy;
		permitem["headmidangyoz"] = angle.angle.angleyoz;

	}
	

	///---------
	angle = _manpose->getang_kneeankle_hipknee(BODY_LEFT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		permitem["lefthipknee"] = angle.angle.angle;
		permitem["lefthipkneexoy"] = angle.angle.anglexoy;
		permitem["lefthipkneeyoz"] = angle.angle.angleyoz;
		//item["p
	}

	angle = _manpose->getang_kneeankle_hipknee(BODY_RIGHT);


	if (angle.angle.angle != NAN && angle.angle.angle != -NAN)
	{
		permitem["righthipknee"] = angle.angle.angle;

		permitem["righthipkneexoy"] = angle.angle.anglexoy;
		permitem["righthipkneeyoz"] = angle.angle.angleyoz;
		//item["p
	}
	permitem["twoknee"] = _manpose->twokneev;
	permitem["twoheelv"] = _manpose->twoheelv;
	arraypos1.append(permitem);

	permpos["perm"] = arraypos1;
	////-------------------------------------------





	/*std::vector<int> partsLink;
	partsLink = {
		0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 8, 12, 12,
		13, 13, 14, 14, 19, 19, 20, 14, 21, 8, 9, 9, 10, 10, 11, 11, 24,
		11, 22, 22, 23, 0, 16, 0, 15, 15, 17, 16, 18
	};*/

	sl::double3 jointpos;
	for (int part = 0; part < 25; part++) {


		jointpos = _manpose->bodyjointposmap[part];

		ItemPos["id"] = part;
		ItemPos["x"] = jointpos.x;
		ItemPos["y"] = jointpos.y;
		ItemPos["z"] = jointpos.z;


		arraypos.append(ItemPos); // append()以数组的形式添加
		//if(isnan(jointpos.x) || isnan(jointpos.y) || isnan(jointpos.z))

		//	v1 = keypoints_position[partsLink[part]];
		//	v2 = keypoints_position[partsLink[part + 1]];

		//	IsUseful(v1, v2, center_gravity);

	}
	bodypos["xyzpos"] = arraypos;

	root.append(oneposes);
	root.append(bodypos);
	root.append(permpos);
	//Json::StreamWriterBuilder  builder;
	//builder.settings_["emitUTF8"]= true;

	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());



	//json_root["bodyposes"].append(oneposes);



	///logInfo("start save json data");
	writer->write(root, &fout);
	fout.close();

	//writer->write(json_bodyls, &fout);

	logInfo("end save json data");
	//std::string out = root.toStyledString();
	//std::cout << out << std::endl;
//	fout << out << std::endl;
}
std::string openposeUtil::endSavezedSvo()
{
	if (bsaveSvo == false)
		return "";


	bsaveSvo = false;

	logInfo("set bool Save svo false");
	return this->getsvo_File();
}
bool openposeUtil::saveposedata()
{
	if (bsavePose == false)
	{

		logInfo("un start pose avi");
		return false;
	}


	return true;
}
void openposeUtil::startSavezedavi()
{

	if (bsaveZed)
		return;



	savezedavi_callback = std::thread(&openposeUtil::savezedavi, this);

}
std::string openposeUtil::endSavezedavi()
{
	if (bsaveZed == false)
		return "";

	//savemanpose();
	bsaveZed = false;
	//outposeVideo.release();
	outzedVideo.release();
	//delete outputVideo;
	//outputVideo = NULL;

	logInfo("over recording zed avi");
	return this->getzed_avi_File();
}
bool openposeUtil::savezedavi()
{
	bool br = true;

	bsaveZed = true;
	logInfo("------------------------------------+++++");



	//logInfo(ii);
	//outputVideo = new VideoWriter(getpose_avi_File(), VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(672, 376),true);

	logInfo(image_height);
	logInfo(image_width);
	//outposeVideo.open(getpose_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 10, cv::Size(image_width, image_height), true);
	outzedVideo.open(getzed_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 25, cv::Size(image_width, image_height), true);

	if (!outzedVideo.isOpened())
	{
		logInfo("zed codec failed\n");
		return false;
	}
	else
		logInfo("zed avi create\n");



	return br;
}

void openposeUtil::GetStringSize(HDC hDC, const char* str, int* w, int* h)
{

	SIZE size;
	GetTextExtentPoint32A(hDC, str, strlen(str), &size);
	if (w != 0) * w = size.cx;
	if (h != 0) * h = size.cy;

}
int openposeUtil::gettextlen(cv::Mat & dst, const char* str, Point org)
{

	CV_Assert(dst.data != 0 && (dst.channels() == 1 || dst.channels() == 3));

	int x, y, r, b;
	if (org.x > dst.cols || org.y > dst.rows) return 0;
	x = org.x < 0 ? -org.x : 0;
	y = org.y < 0 ? -org.y : 0;




	HDC hDC = CreateCompatibleDC(0);

	if (hDC == NULL)
		return std::string(str).length();

	int strBaseW = 0, strBaseH = 0;
	int singleRow = 0;
	char buf[1 << 12];
	strcpy_s(buf, str);
	char* bufT[1 << 12];  // 这个用于分隔字符串后剩余的字符，可能会超出。
	//处理多行
	{
		int nnh = 0;
		int cw, ch;

		const char* ln = strtok_s(buf, "\n", bufT);
		while (ln != 0)
		{
			GetStringSize(hDC, ln, &cw, &ch);
			strBaseW = max(strBaseW, cw);
			strBaseH = max(strBaseH, ch);

			ln = strtok_s(0, "\n", bufT);
			nnh++;
		}
		singleRow = strBaseH;
		strBaseH *= nnh;
	}

	if (org.x + strBaseW < 0 || org.y + strBaseH < 0)
	{

		DeleteObject(hDC);
		return 0;
	}

	r = org.x + strBaseW > dst.cols ? dst.cols - org.x - 1 : strBaseW - 1;

	DeleteObject(hDC);
	return r;
}
void openposeUtil::mergeimg(cv::Mat & img1, cv::Mat & img2, cv::Mat & img3, cv::Mat & img4)
{
	/*cv::Mat img1 = cv::imread("test1.jpg");
	cv::Mat img2 = cv::imread("test2.jpg");
	cv::Mat img3 = cv::imread("test3.jpg");
	cv::Mat img4 = cv::imread("test4.jpg");*/

	// Make sure they have been loaded successfully
	if (img1.empty() || img2.empty() || img3.empty() || img4.empty())
	{
		logInfo("!!! Failed to load one of the images\n");
		return;
		//return "";
	}

	/* Make sure they are compatible: same type, depth and # channels */

	if ((img1.type() != img2.type()) ||
		(img3.type() != img4.type()) ||
		(img1.type() != img4.type()))
	{
		logInfo("!!! The depth doesn't match!\n");
		return;
		//return "";
	}

	if ((img1.depth() != img2.depth()) ||
		(img3.depth() != img4.depth()) ||
		(img1.depth() != img4.depth()))
	{
		logInfo("!!! The depth doesn't match!\n");
		return;
		//return "";
	}

	if ((img1.channels() != img2.channels()) ||
		(img3.channels() != img4.channels()) ||
		(img1.channels() != img4.channels()))
	{
		logInfo("!!! Number of channels doesn't match!\n");
		return;
		//return "";
	}

	// Create the destination image based on the size of the loaded images
	//  _________
	// |    |    |
	// |_1__|_2__|
	// |    |    |
	// |_3__|_4__|

	/* As the input images might have different sizes, we need to make sure
	 * to create an output image that is big enough to store all of them.
	 */

	 // Compute the width of the output image
	int row1_size = img1.size().width + img2.size().width;
	int row2_size = img3.size().width + img4.size().width;
	int new_width = std::max(row1_size, row2_size);

	// Compute the height of the output image
	int col1_size = img1.size().height + img3.size().height;
	int col2_size = img2.size().height + img4.size().height;
	int new_height = std::max(col1_size, col2_size);

	// Create the destination image
	dst_img.create(cv::Size(new_width, new_height), img1.type());

	//logInfo(dst_img.size().width);
	//logInfo(dst_img.size().height);
	//std::cout << "dst: size " << dst_img.size().width << "x" << dst_img.size().height << std::endl;

	/* Copy the pixels of the input images to the destination */

	//  _________
	// |    |    |
	// |_1__|    |
	// |         |
	// |_________|
	img1.copyTo(dst_img(cv::Rect(0, 0, img1.cols, img1.rows)));


	//cv::imshow("Test", dst_img);
	//cv::waitKey(0);
	//  _________
	// |    |    |
	// |_1__|_2__|
	// |         |
	// |_________|    
	img2.copyTo(dst_img(cv::Rect(img1.size().width, 0, img2.cols, img2.rows)));


	//cv::imshow("Test", dst_img);
	//cv::waitKey(0);
	//  _________
	// |    |    |
	// |_1__|_2__|
	// |    |    |
	// |_3__|____|    
	img3.copyTo(dst_img(cv::Rect(0,
		std::max(img1.size().height, img2.size().height),
		img3.cols,
		img3.rows)));


	//cv::imshow("Test", dst_img);
	//cv::waitKey(0);
	//  _________
	// |    |    |
	// |_1__|_2__|
	// |    |    |
	// |_3__|_4__|    
	img4.copyTo(dst_img(cv::Rect(img3.size().width,
		std::max(img1.size().height, img2.size().height),
		img4.cols,
		img4.rows)));

	// For testing purposes, display it on the screen
	//cv::imshow("Test", dst_img);
	//cv::waitKey(0);

}
std::string openposeUtil::startmergereportavi(std::string hmavi_file, std::string zedsvo_file, std::string zedposeavi_file, std::string posedata_dir, bool isshow)
{


	bshow = isshow;
	logInfo(hmavi_file);

	logInfo(zedsvo_file);
	logInfo(zedposeavi_file);
	logInfo(posedata_dir);
	reinit();

	//fromjson(posedata_file);
	///std::list<Json::Value> jsonbodyposels = getbodyposesfromjsonfile(posedata_dir);

	VideoCapture hkcap;
	//VideoCapture zedavicap;
	VideoCapture poseavicap;


	hkcap.open(hmavi_file);
	//zedavicap.open(zedsvo_file);
	poseavicap.open(zedposeavi_file);



	InitParameters init_parameters;
	init_parameters.input.setFromSVOFile(zedsvo_file.data());
	init_parameters.camera_disable_self_calib = true;
	init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

	// Open the camera
	ERROR_CODE zed_open_state = zed.open(init_parameters);
	if (zed_open_state != ERROR_CODE::SUCCESS) {
		logInfo("Camera Open error.");
		return "";
	}

	sl::Resolution new_image_size;


	if (!hkcap.isOpened() ||
		//!zedavicap.isOpened() ||
		!poseavicap.isOpened())
	{
		logInfo("无法打开视频文件");
		return "";
	}
	cv::Mat simg1, simg2,simg4;

	cv::Mat img1, img2, img3, img4;


	
	Json::Value jsonbodypose;

	Resolution image_size;
	int new_width, new_height;



	image_size = zed.getCameraInformation().camera_configuration.resolution;
	new_width = image_size.width / 1;
	new_height = image_size.height / 1;

	image_height = new_height;
	image_width = new_width;

	sl::Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
	image_ocv = slMat2cvMat(image_zed);


	RuntimeParameters rt_param;
	int svo_position = 0;
	int svo_frame_rate = zed.getInitParameters().camera_fps;
	int nb_frames = zed.getSVONumberOfFrames();

	//logInfo(nb_frames);
	zed.setSVOPosition(0);
	poseavicap.set(CAP_PROP_POS_FRAMES, 0);
	int videoFramesNum = poseavicap.get(CAP_PROP_FRAME_COUNT);//获取视频帧数
	hkcap.set(CAP_PROP_POS_FRAMES, 0);
	int hkvideoframenum = hkcap.get(CAP_PROP_FRAME_COUNT);//获取视频帧数

	//int framestep = nb_frames / videoFramesNum;


	logInfo("frame data is ");

	logInfo(zedimgnum);
//	logInfo(poseimgnum);
	logInfo(videoFramesNum);
	
	//logInfo(nb_frames);
	//logInfo(framestep);
	zedimgnum = zed.getSVONumberOfFrames();;


	if (videoFramesNum == 0)
	{
		cout << "pose video frames count is 0" << endl;
		logInfo("pose video frames count is 0");
		return "";
	}
	else
	jsonnum = videoFramesNum;

	if (hkvideoframenum == 0)
	{
		cout << "hai kang video frames count is 0" << endl;
		logInfo("hai kang video frames count is 0");
		return "";
	}
	else
		hkimgnum = hkvideoframenum;
	cout << "pose frmaes is " << jsonnum << endl;
	cout << "haikang frmaes is " << hkimgnum << endl;

	//hkimgnum = 276;
	double framestep =(double)zedimgnum / hkimgnum;

	int hkidx = 0;
	int poseidx = 0;

	int hkvidx = 0;
	cout << "framestep is " << framestep << endl;
	while (poseavicap.grab())
	{

		//hkidx = (int)i * framestep;

		//if ((i % framestep)==0)

		
		//cout << "hai kang vidx is " << hkidx << endl;
		//cout << "pose videoidx is " << poseidx << endl;
		hkvidx = (int)hkidx * framestep;

		if(hkvidx ==poseidx)
		{

			hkcap.grab();
			hkcap.retrieve(simg1);

			hkidx++;


		}
		cout << "hai kang idx is " << hkidx << endl;
		cout << "pose videoidx is " << poseidx << endl;
		cout << "hk vv videoidx is " << hkvidx << endl;


		//svo_position = zed.getSVOPosition();
		//zed.setSVOPosition(i * framestep);
		poseavicap.retrieve(zedposeimg);
		cout << "-------------------------1111111111" << endl;
		if (zed.grab(rt_param) == ERROR_CODE::SUCCESS) {

			zed.retrieveImage(image_zed, VIEW::LEFT);
			cv::cvtColor(image_ocv, simg2, cv::COLOR_RGBA2RGB);
		}
		cout << "-------------------------222222222222222" << endl;

		cv::resize(simg1, img1, zedposeimg.size());

		if (bshow)
		{
			cv::imshow("test", simg2);
			waitKey(10);
		}
		//zedavicap.retrieve(simg2);



		cv::resize(simg2, img2, zedposeimg.size());

		//jsonbodypose = getJsonByIndex(jsonbodyposels,i);
		//cout << posedata_dir + std::to_string(i) + ".json" << endl;
		simg4 = fromjson(posedata_dir + std::to_string(poseidx) + ".json");
		//cout << "over from json" << endl;
		//recalcposedatimg(jsonbodypose);
		//cv::imshow("Test2", img2);
		//waitKey(10);
		cv::resize(simg4, img4, zedposeimg.size());
		cout << "----------------------" << endl;
		mergeimg(img1, img2, zedposeimg, img4);
		cout << "--over merge image--------------------" << endl;
		if (!outmergeVideo.isOpened())
		{
			outmergeVideo.open(getmerge_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 25, dst_img.size(), true);
			cout << "open videowarer---------" << endl;
		}
		else
			cout << "not need open videowarer---------" << endl;


		if (bshow)
		{
			cv::imshow("Test", dst_img);
			waitKey(10);
		}
		//logInfo("start one merge frame");
		outmergeVideo.write(dst_img);

		sl::sleep_ms(20);

		cout << "sleep 20ms" << endl;
		//zedposedataimg.release();

		poseidx++;
	}
	//logInfo("avi frame is ");
	//logInfo(i);
	hkcap.release();
	zed.close();

	poseavicap.release();
	outmergeVideo.release();

	destroyAllWindows();
//	poseimgnum = 0;
	zedimgnum = 0;
	hkimgnum = 0;
	jsonnum = 0;
	//mergeimg(cv::Mat & img1, cv::Mat & img2, cv::Mat & img3, cv::Mat & img4)
	// Load 4 images from the disk

	bcomposeavifinished = true;
	logInfo("over merge avi");
	return getmerge_avi_File();
}
//void openposeUtil::run()
//{
//
//    //op::opLog("OpenPose Library Tutorial - Example 1.", op::Priority::High);
//    // ------------------------- INITIALIZATION -------------------------
//    // Step 1 - Set logging level
//    // - 0 will output all the logging messages
//    // - 255 will output nothing
//    op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
//              __LINE__, __FUNCTION__, __FILE__);
//    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
//    op::opLog("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
//    // Step 2 - Read Google flags (user defined configuration)
//    // outputSize
//    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution.data()), "-1x-1");
//    // netInputSize
//    const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution.data()), "-1x368");
//    // poseModel
//    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose.data()));
//    // Check no contradictory flags enabled
//    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
//        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
//    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
//        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
//                  __LINE__, __FUNCTION__, __FILE__);
//    // Enabling Google Logging
//    const bool enableGoogleLogging = true;
//    // Logging
//    op::opLog("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
//    // Step 3 - Initialize all required classes
//    op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
//    op::CvMatToOpInput cvMatToOpInput;
//    op::CvMatToOpOutput cvMatToOpOutput;
//    op::PoseExtractorCaffe poseExtractorCaffe{poseModel, FLAGS_model_folder,
//                                              FLAGS_num_gpu_start, {}, op::ScaleMode::ZeroToOne, enableGoogleLogging};
//    op::PoseCpuRenderer poseRenderer{poseModel, (float)FLAGS_render_threshold, !FLAGS_disable_blending,
//                                     (float)FLAGS_alpha_pose};
//    op::OpOutputToCvMat opOutputToCvMat;
//    op::FrameDisplayer frameDisplayer{"OpenPose Tutorial - Example 1", outputSize};
//    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
//    poseExtractorCaffe.initializationOnThread();
//    poseRenderer.initializationOnThread();
//    //cv::Mat inputImage = cv::imread("/home/neu-lu/openpose/examples/media/COCO_val2014_000000000257.jpg");
//    //cv::imshow("test",inputImage);
//
//
//
//    cv::Mat frame;
//    while(cap.isOpened())
//    {
//        /////////////////////////////////
//        // Stop thread if stopped=TRUE //测试是否停止的过程要加锁
//        /////////////////////////////////
//        stoppedMutex1.lock();
//        if (stopped1)
//        {
//            stopped1=false;
//            stoppedMutex1.unlock();
//            break;
//        }
//        stoppedMutex1.unlock();
//        /////////////////////////////////
//
//        cap>>frame;
//        cap>>frame;
//        cap>>frame;
//        updateMembersMutex.lock();
//
//        cv::Mat inputImage = frame;
//        if(inputImage.empty())
//            op::error("Could not open or find the image: ");
//        //op::error("Could not open or find the image: " + FLAGS_image_path, __LINE__, __FUNCTION__, __FILE__);
//        const op::Point<int> imageSize{inputImage.cols, inputImage.rows};
//        // Step 2 - Get desired scale sizes
//        std::vector<double> scaleInputToNetInputs;
//        std::vector<op::Point<int>> netInputSizes;
//        double scaleInputToOutput;
//        op::Point<int> outputResolution;
//        std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
//                = scaleAndSizeExtractor.extract(imageSize);
//        // Step 3 - Format input image to OpenPose input and output formats
//        const auto netInputArray = cvMatToOpInput.createArray(OP_CV2OPCONSTMAT(inputImage), scaleInputToNetInputs, netInputSizes);
//        auto outputArray = cvMatToOpOutput.createArray(OP_CV2OPCONSTMAT(inputImage), scaleInputToOutput, outputResolution);
//        // Step 4 - Estimate poseKeypoints
//        poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
//        const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
//        // Step 5 - Render poseKeypoints
//        poseRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput);
//        // Step 6 - OpenPose output format to cv::Mat
//        auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);
//
//
//
//        //------------------------------处理每一帧的关键点---------------------------------
//        const auto numberPeopleDetected = poseKeypoints.getSize(0);
//        const auto numberBodyParts = poseKeypoints.getSize(1);
//        // Easy version
//        int person=0;
//        if(numberPeopleDetected>1)person=1;
//        vector<pair<float,float> > points;
//        points.clear();
//        for(int i=0;i<=16;i++)
//        {
//            float x = poseKeypoints[{person, i, 0}];
//            float y = poseKeypoints[{person, i, 1}];
//            points.push_back(make_pair(x,y));
//        }
//
//
//        // ------------------------- SHOWING RESULT AND CLOSING -------------------------
//        // Step 1 - Show results
//        //frameDisplayer.displayFrame(outputImage, 0);
//        //cv::imshow("test",outputImage);// + cv::waitKey(0)
//        // Step 2 - Logging information message
//
//        // Convert Mat to QImage: Show BGR frame
//        //QImage img= QImage((const unsigned char*)(outputImage.data),outputImage.cols,outputImage.rows,QImage::Format_RGB888);
//		QImage img = fromCvMat(OP_OP2CVMAT(outputImage));
//        img=img.scaled(500,250);
//        updateMembersMutex.unlock();
//
//        // Inform GUI thread of new frame (QImage)
//        //发出信号，通知GUI线程有新处理好的一帧
//        emit newFrame(img);
//        emit newPoint(points);
//        //ui->label_player->setPixmap(QPixmap::fromImage(img));
//
//        //cv::waitKey(1);
//        //qApp->processEvents();
//    }
//}


//QImage openposeUtil::fromCvMat(cv::Mat cvimgmat)
//{
//	const uchar* pSrc = (const uchar*)cvimgmat.data;
//	QImage temp(pSrc, cvimgmat.cols, cvimgmat.rows, cvimgmat.step, QImage::Format_RGB888);
//	return temp;
//}

void openposeUtil::createSvodescmap()
{

	

	std::string line, filename, strnum, filefullname;
	op::Rectangle<int> _rect;
	int idxpos = 1, i = 0, j = 0;
	int hposidx = -1;

	bool isfirst = false;
	int fframeidx = -1;


	int ppersonidx = -1;
	//	ImgIdx = -1;
		//bodyposrect.empty();
	//m_svoimgmap.clear();
	//m_svopersonmap.clear();

	infile.clear(infile.goodbit);
	infile.seekg(ios::beg);
	while (std::getline(infile, line)) {
		
		if (i % 2 == 0)
		{
		//	filefullname = line;
			//filename = getFilename(line);
			filename = line;
			//cout << "filename is " << filename << endl;
			if (filename.compare("") != 0)
			{
				hposidx = filename.find_first_of("_");
				if (hposidx != string::npos)
				{
					strnum = filename.substr(0, hposidx);
					///t << "num is " << strnum << endl;
					fframeidx = std::atoi(strnum.c_str());
			
					strnum = filename.substr(hposidx + 1, hposidx + 2);

					ppersonidx = std::atoi(strnum.c_str());

				}
			}
		}
		else
		{
			m_svoimgmap.insert(make_pair(fframeidx, line));
			m_svopersonmap.insert(make_pair(fframeidx, ppersonidx));
		
		}

		i++;
	}


	infile.close();
	isopen = false;
	

}

void openposeUtil::selectSvoPosAndSet()
{
	if (m_inputDataType == InputDataType::SVO_TXT)
	{
      //打开数据描述文件，并定向zed svo文件位置

		if (isopen == false)
		{

			infile.open(m_svotxtDir + "filelist.txt", std::ifstream::in);

			if (infile.is_open())
			{
				isopen = true;

				cout << "open filelist.txt okay" << endl;
			}
			else
			{
				isopen = false;
				cout << "open filelist.txt error" << endl;
			}
		}


		std::string line, filename, strnum;
	
	
		int hposidx = -1;

	
		//int fframeidx = -1;
//		ImgIdx = -1;
		//bodyposrect.empty();
		//bodyposrect.clear();

		//bodyposrectfile.clear();
		//infile.clear(infile.goodbit);
		//infile.seekg(ios::beg);

		//reInitFilterRect();


		if (!std::getline(infile, line))
		{
			m_blastsvo = true;
			cout << "read file last " << endl;
			return;
		}


		filename = line;




		hposidx = filename.find_first_of("_");
		if (hposidx != string::npos)
		{
		strnum = filename.substr(0, hposidx);

		m_svodescIdx = std::atoi(strnum.c_str());

		strnum = filename.substr(hposidx + 1, hposidx + 2);

		personIdx  = std::atoi(strnum.c_str());
	//	cout << "person idx is " << personIdx << endl;


		}
		std::getline(infile, line);

		filterRect = getRect(line);


		//zed.setSVOPosition(m_svodescIdx);
		logInfo("Zed set pos is");
		logInfo(m_svodescIdx);

		svoTxtPos++;	

	}
}

StringList openposeUtil::splitstr(const std::string& str, const std::string& pattern)
{
	StringList  li;
	std::string subStr;
	std::string tPattern;
	size_t      patternLen = pattern.length();

	//遍历字符串，将i位置的字符放入子串中，当遇到pattern子串时完成一次切割
	//遍历之后得到切割后的子串列表
	for (size_t i = 0; i < str.length(); i++)
	{
		if (pattern[0] == str[i])//遇到需要检测pattern的情况
		{
			tPattern = str.substr(i, patternLen);
			if (tPattern == pattern)//找到一个匹配的pattern，完成切割
			{
				i += patternLen - 1;
				if (!subStr.empty())
				{
					li.push_back(subStr);
					subStr.clear();
				}
			}
			else//不是匹配的pattern，将i位置的字符放入子串
			{
				subStr.push_back(str[i]);
			}
		}
		else//未遇到pattern，将i位置的字符放入子串
		{
			subStr.push_back(str[i]);
		}
	}

	if (!subStr.empty())//将子串中的剩余字符放入子字符串队列
	{
		li.push_back(subStr);
	}

	return li;
}
std::string openposeUtil::getFilename(std::string fullfilename)
{
	StringList res = splitstr(fullfilename, "/");

	if (res.size() > 0)
	{
		//ut << "filename is " << res[res.size() - 1] << endl;
		return res[res.size() - 1];
	}
	else
		return "";
	/*std::cout << "string list count:" << res.size() << endl;
	for (int i = 0; i < res.size(); i++)
	{
		std::cout << res[i] << endl;
	}
	return std::string();*/
}

op::Rectangle<int> openposeUtil::getRect(std::string linerect)
{
	op::Rectangle<int> rect;

	std::string strrect = linerect.substr(1, linerect.length() - 2);

	//cout << "rect is " << strrect << endl;

	StringList res = splitstr(strrect, ",");
	std::string strnum;
	if (res.size() == 4)
	{
		strnum = res[0];

		rect.x = std::atoi(strnum.c_str());
		strnum = res[1];

		rect.y = std::atoi(strnum.c_str());
		strnum = res[2];

		rect.width = std::atoi(strnum.c_str());
		strnum = res[3];

		rect.height = std::atoi(strnum.c_str());

	}
	else
	{
		rect.x = -1;
		rect.y = -1;
		rect.width = -1;
		rect.height = -1;
	}
	
	return rect;
}

std::list<AngleInfo> openposeUtil::getAnglesFromonebodypose(Json::Value _json_bodypose)
{
	std::list<AngleInfo> rs;

	//json_is.open(_json_file, std::ios::binary);
	//if (json_reader.parse(json_is, json_root, FALSE))


	int file_size = _json_bodypose.size();  // 得到"files"的数组个数  
	//std::wstring wsdesc;
	std::string sdesc = "";
	AngleInfo angle;
	for (int i = 0; i < file_size; ++i)  // 遍历数组  
	{
		Json::Value leftright = _json_bodypose[i]["LEFTRIGHT"];
		double _angle = _json_bodypose[i]["angle"].asDouble();
		double _anglexoy = _json_bodypose[i]["anglexoy"].asDouble();
		double _angleyoz = _json_bodypose[i]["angleyoz"].asDouble();
		int x = _json_bodypose[i]["x"].asInt();
		int y = _json_bodypose[i]["y"].asInt();
		sdesc = _json_bodypose[i]["desc"].asString();

		//wsdesc = s2ws(sdesc);
		angle.angle.angle = _angle;
		angle.angle.anglexoy = _anglexoy;
		angle.angle.angleyoz = _angleyoz;
		angle.x = x;
		angle.y = y;
		angle.desc = sdesc;
		rs.emplace_back(angle);
	}
	return rs;
}
void openposeUtil::recalcposedatimg(Json::Value _json_bodypose)
{

	std::list<AngleInfo> angls = getAnglesFromonebodypose(_json_bodypose);


	zedposedataimg.release();
	zedposedataimg.create(image_height, image_width, CV_8UC3);
	int anggount = angls.size();
	int ydiv = image_height / anggount;

	int i = 0;
	std::string angleinfo;
	for (list <AngleInfo> ::iterator it = angls.begin(); it != angls.end(); ++it)
	{
		logInfo(*it);
		angleinfo = printangleinfo(*it);
		drawText(&zedposedataimg, angleinfo, cv::Point(0, i * ydiv));
		i++;
	}



	//cv::Mat img1(600, 800, CV_8UC3, Scalar(100, 250, 30));
	//zedposedataimg.copyTo(img);
	//Mat::zeros(Size(image.cols, image.rows), CV_8UC1);
	logInfo("----------------oonvert json to img");


}
void openposeUtil::fillouterRect(cv::Mat& inputimg)
{
	//outerrect.clear();
	//cv::Rect recttop, rectbom, rectleft, rectright;
	poseImage_width;
	poseImage_height;
	int inx = 0, iny = 0, rinx = 0, riny = 0;




	cout << "ImgIdx is  " << m_svoImgIdx <<" svo img is "<<m_svodescIdx << endl;
	cout << "cut pic filterRect " << " " << filterRect.x << "," << filterRect.y << "," << filterRect.width << "," << filterRect.height << endl;


	inx = filterRect.x;
	iny = filterRect.y;

	rinx = filterRect.x + filterRect.width;
	riny = filterRect.y + filterRect.height;

	cv::Rect recttop(0,0, poseImage_width, filterRect.y);

	cv::Rect rectleft(0, filterRect.y, filterRect.x, filterRect.height);
	cv::Rect rectbom(0, filterRect.y + filterRect.height, poseImage_width, poseImage_height - filterRect.y-filterRect.height);

	cv::Rect rectright(filterRect.x+ filterRect.width, filterRect.y, poseImage_width - filterRect.x - filterRect.width, poseImage_height - filterRect.y - filterRect.height);
	
	/*outerrect[0] = recttop;
	outerrect[1] = rectleft;
	outerrect[2] = rectbom;
	outerrect[3] = rectright;
*/
	cv::Mat dstImg = inputimg.clone();
	cv::Mat zone1(recttop.height, recttop.width, CV_8UC3, Scalar(0));
	cv::Mat tmpMat = dstImg(recttop);

	zone1.copyTo(tmpMat);
	//cv::imshow("1323", dstImg);
	//waitKey(0);

	cv::Mat zone2(rectleft.height, rectleft.width, CV_8UC3, Scalar(0));
	tmpMat = dstImg(rectleft);

	zone2.copyTo(tmpMat);
	//cv::imshow("1323", dstImg);
	//waitKey(0);
	cv::Mat zone3(rectbom.height, rectbom.width, CV_8UC3, Scalar(0));
	tmpMat = dstImg(rectbom);

	zone3.copyTo(tmpMat);
	//cv::imshow("1323", dstImg);
	//waitKey(0);
	cv::Mat zone4(rectright.height, rectright.width, CV_8UC3, Scalar(0));
	tmpMat = dstImg(rectright);

	zone4.copyTo(tmpMat);

	inputimg = dstImg.clone();
	//cv::imshow("1323", inputimg);
	//waitKey(0);

}
void openposeUtil::test1()
{


	int n_count=1;
	int frames = 10;
	int flag = 0;
	int y_max = 0;
	int n_frame = 0;



	/*if (flag == 0)
	{

		if (ret_y >= y_max)
		{
			y_max = ret_y;
		}
		else
		{
			y_max = 0;
			flag = 1;
		}
	}
	else
	{
		n_frame++;
		if (hist_max - ret_y >= 1)
		{
			flag++;
				if (ret_y > y_max)
				{
					y_max = ret_y;
				}
		}
		if (flag >= n_count || n_frame >= frames)
		{
			flag = 0;
		}
	}*/
	//return ret_rect
}
std::list<Json::Value> openposeUtil::getbodyposesfromjsonfile(std::string _json_file)
{
	std::list<Json::Value> rs;
	json_is.open(_json_file, std::ios::binary);
	Json::Value jsls;
	while (json_reader.parse(json_is, json_root, FALSE))
	{

		//	logInfo(json_root.asString());
		rs.emplace_back(json_root);
		//code = json_root.get("bodypose", "null").asString();// 访问节点，Return the member named key if it exist, defaultValue otherwise.    
		//jsls = json_root["bodypose"];
		//int file_size = json_root["bodypose"].size();  // 
		////std::wstring wsdesc;
		//std::string sdesc = "";
		//Angle angle;
		//for (int i = 0; i < file_size; ++i)  // 遍历数组  
		//{
		//	Json::Value jsonbodypose = json_root["bodypose"][i];
		//	rs.emplace_back(jsonbodypose);
		//}
	}
	json_is.close();

	return rs;
}
std::list<AngleInfo> openposeUtil::fromjsonfile(std::string _json_file)
{
	std::list<AngleInfo> rs;

	json_is.open(_json_file, std::ios::binary);

	cout << _json_file << endl;

	if (json_reader.parse(json_is, json_root, FALSE))
	{

		//if (!json_root["id"].isNull())  // 访问节点，Access an object value by name, create a null member if it does not exist.  
		//	code = json_root["bodypose"].asString();

		//code = json_root.get("bodypose", "null").asString();// 访问节点，Return the member named key if it exist, defaultValue otherwise.    

		int file_size = json_root["bodypose"].size();  // 得到"files"的数组个数  

		cout << file_size << endl;
		//std::wstring wsdesc;
		std::string sdesc = "";
		std::string sdescxoy = "";
		std::string sdescyoz = "";

		AngleInfo angle;
		for (int i = 0; i < file_size; ++i)  // 遍历数组  
		{
		//	cout << "++++++++++++++++++++++sfsdfsdf" << endl;
			Json::Value leftright = json_root["bodypose"][i]["LEFTRIGHT"];
			double _angle = json_root["bodypose"][i]["angle"].asDouble();
			double _anglexoy = json_root["bodypose"][i]["anglexoy"].asDouble();
			double _angleyoz = json_root["bodypose"][i]["angleyoz"].asDouble();
			int x = json_root["bodypose"][i]["x"].asInt();
			int y = json_root["bodypose"][i]["y"].asInt();
			sdesc = json_root["bodypose"][i]["desc"].asString();
			sdescxoy = json_root["bodypose"][i]["descxoy"].asString();
			sdescyoz = json_root["bodypose"][i]["descyoz"].asString();

			//wsdesc = s2ws(sdesc);
			angle.angle.angle = _angle;
			angle.angle.anglexoy = _anglexoy;
			angle.angle.angleyoz = _angleyoz;
			angle.x = x;
			angle.y = y;
			angle.desc = sdesc;
			angle.descxoy = sdescxoy;
			angle.descyoz = sdescyoz;
			rs.emplace_back(angle);

		}
	}
	/*else
		cout << "++over dsfsdfsdfsdfsddf" << endl;*/

	

	json_is.close();


	return rs;
}

cv::Mat openposeUtil::fromjson(std::string json_file)
{
	
	//Mat img（500，1000，CV_8UC3，Scalar（0, 0，100））;
	///zedposedataimg.release();
	//cv::Mat img(image_height,image_width, CV_8UC3, Scalar(100, 250, 30));
	std::string geeks_window = "COLORED BLANK IMAGE";

	// crearting window for image display 
	//namedWindow(geeks_window);
	cout << json_file << endl;
	cv::Mat textimg(image_height, image_width, CV_8UC3,Scalar(0, 0, 0));
	//cout << "22222222222222222222222222" << endl;
	std::list<AngleInfo> posels = fromjsonfile(json_file);
	//cout << "3333333333333333333333333333333" << endl;
	int anggount = posels.size();


	int ydiv = 0;
	if (anggount != 0)
		ydiv = image_height / anggount;
	else
		cout << "-----------------zero-------------" << endl;

	int i = 0;
	std::string angleinfo;
	//cout << "1$$$$$$$$$$$$$$$$$$$$$$" << endl;
	for (list <AngleInfo> ::iterator it = posels.begin(); it != posels.end(); ++it)
	{
	//	logInfo(*it);
		angleinfo = printangleinfo(*it);
		drawText(&textimg, angleinfo, cv::Point(0, i * ydiv));
		i++;
		cout <<"print angle:"<< angleinfo << endl;
	}


	//cout << "over json to img" << endl;
	//cv::Mat img1(600, 800, CV_8UC3, Scalar(100, 250, 30));
	//zedposedataimg.copyTo(img);
	//Mat::zeros(Size(image.cols, image.rows), CV_8UC1);
	//logInfo("----------------oonvert json to img");
	///drawText(&zedposedataimg, "fsfsdffsdlfjjsldfjlsj", cv::Point(0, 0));




	// image shown inside the window 
	//imshow(geeks_window, zedposedataimg);

	// wait for any key press 
	//waitKey(0);

	// destroying the created window 
	//destroyWindow(geeks_window);
	//imshow("drawtext", img1);
	//imshow("drawtext", zedposedataimg);
	//return cv::Mat();
	return textimg;
}

bool openposeUtil::initFloorZED(sl::Camera & zed) {
	bool init = false;
#if ENABLE_FLOOR_PLANE_DETECTION

	const int timeout = 20;
	int count = 0;

	logInfo("Looking for the floor plane to initialize the tracking...");

	while (!init && count++ < timeout) {
		zed.grab();
		init = (zed.findFloorPlane(plane, resetTrackingFloorFrame) == sl::ERROR_CODE::SUCCESS);


		string strmatrix = resetTrackingFloorFrame.getInfos();



		if (init) {
			logInfo("resetTrackingFloorFrame");
			logInfo(resetTrackingFloorFrame);
			zed.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
			logInfo("Floor found at : \n");

			Translation tr1 = camera_pose.pose_data.getTranslation();
			logInfo(plane.getClosestDistance(camera_pose.pose_data.getTranslation()));
			logInfo("m");
			Transform tr = plane.getPose();


			logInfo(tr);
			logInfo(tr1);
			sl::float3 n1 = plane.getNormal();
			sl::float3 pc = plane.getCenter();

			logInfo("floor normal is ");
			logInfo(n1);
			logInfo("floor center point is ");
			logInfo(pc);

			zed.resetPositionalTracking(resetTrackingFloorFrame);
		}
		sl::sleep_ms(20);
	}
	if (init) for (int i = 0; i < 4; i++) zed.grab();
	else logInfo("Floor plane not found, starting anyway");
#endif
	return init;
}




void openposeUtil::startZED() {
	quit = false;

	
	//zed_callback.detach();
	
	//std::thread zed_callback;
	if (zed_callback.joinable())
		zed_callback.join();
	zed_callback = std::thread(&openposeUtil::runZed, this);
	//zed_callback = std::thread(, this);
}

void openposeUtil::startOpenpose() {
	//openpose_callback.detach();

	//std::thread openpose_callback;
	if (openpose_callback.joinable())
		openpose_callback.join();

	openpose_callback = std::thread(&openposeUtil::findpose, this);
}


void openposeUtil::reinit()
{
	stdsvo_File = "";

	stdpose_data_Dir = "";

	stdzed_avi_File = "";

	stdpose_avi_File = "";


	stdmerge_avi_File = "";



	reInitFilterRect();


	bshow = true;

	bsavePose = true;


	bzero = false;

	bposeavifinished = false;

	bcomposeavifinished = false;

	m_svodescIdx = 0;

	m_svoImgIdx = 0;

	totaljsons.clear();



	//ImgIdx = -1;
	//bodyposrect.empty();
	//bodyposrect.clear();

	//bodyposrectfile.clear();

	personIdx = -1;

	//m_svoimgmap.clear();  //
	//m_svopersonmap.clear();  //

	if (isopen)
	{
		infile.clear(infile.goodbit);
		infile.seekg(ios::beg);
	}
	//image_width = 1920;
	//image_height = 1080;

	//simage_width = 1920;
	//simage_height = 1080;

}

void openposeUtil::reInitFilterRect()
{
	filterRect.x = -1;
	filterRect.y = -1;
	filterRect.width = -1;
	filterRect.height = -1;
}

bool openposeUtil::isPoseAviFinished()
{
	return bposeavifinished;
}

bool openposeUtil::isComposeaviFinished()
{
	return bcomposeavifinished;
}

void openposeUtil::findpose() {

	while (!ready_to_start) sl::sleep_ms(2); // Waiting for the ZED
	std::string user_model_folder = FLAGS_openpose_root_dir + FLAGS_model_folder;

	op::PoseExtractorCaffe poseExtractorCaffe{ poseModel, user_model_folder, FLAGS_num_gpu_start,{}, op::ScaleMode::ZeroToOne, 1 };
	poseExtractorCaffe.initializationOnThread();


	
	while (!quit) {
		INIT_TIMER;
		//  Estimate poseKeypoints
		
		if (!need_new_image) { // No new image
			data_in_mtx.lock();
			need_new_image = true;


			if (filterRect.x == -1 && filterRect.y == -1 && filterRect.width == -1 && filterRect.height == -1)
			{
				opLog("decide 55555555555555555555", op::Priority::Max, -1, "");
			}
			else
			{
				//opLog("decide 55555555555555555555", op::Priority::Max, -1, "");

				opLog("select pose1 x" + std::to_string(filterRect.x), op::Priority::Max, -1, "");
				opLog("select pose1 y" + std::to_string(filterRect.y), op::Priority::Max, -1, "");
				opLog("select pose1 w" + std::to_string(filterRect.width), op::Priority::Max, -1, "");
				opLog("select pose1 h" + std::to_string(filterRect.height), op::Priority::Max, -1, "");

				poseExtractorCaffe.setPoseKeyPointRectangle(filterRect.x, filterRect.y, filterRect.width, filterRect.height);
			}
			

			poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
			data_in_mtx.unlock();

			

			// Extract poseKeypoints
			data_out_mtx.lock();
			poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
			data_out_mtx.unlock();
			///reInitFilterRect();
			//STOP_TIMER("OpenPose");
		}
		else sl::sleep_ms(1);
	}
}

bool openposeUtil::readImgRectFromText(int npos)
{
	if (isopen == false)
		return false;


	std::string line, filename, strnum,filefullname;
	op::Rectangle<int> _rect;
	int idxpos = 1,i=0,j=0;
	int hposidx = -1;

	bool isfirst = false;
	int fframeidx = -1;
//	ImgIdx = -1;
	//bodyposrect.empty();
	bodyposrect.clear();

	bodyposrectfile.clear();
	infile.clear(infile.goodbit);
	infile.seekg(ios::beg);
	while (std::getline(infile, line)) {
		// using printf() in all tests for consistency

		//printf("%s", line.c_str());
		if (i%2 == 0)
		{
			filefullname = line;
			filename = getFilename(line);
			//cout << "filename is " << filename << endl;
			if (filename.compare("")!=0)
			{
				hposidx = filename.find_first_of("_");
				if (hposidx != string::npos)
				{
					strnum = filename.substr(0, hposidx);
					///t << "num is " << strnum << endl;
					fframeidx = std::atoi(strnum.c_str());
				//	cout << "Img frame idx is " << fframeidx << endl;

				
				}
			}
		}
		else
		{

			//cout << "here " << line << endl;
			_rect = getRect(line);
			//cout << "_rect " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;

			//cout << "fframeidx =" << fframeidx << ":ImgIdx=" <<ImgIdx << ":Isfirst =" << isfirst << endl;
			if (npos==fframeidx)
			{
				//cout << "777777777777777777777777" << endl;
				if (isfirst == false)
				{
					m_svodescIdx = fframeidx;
					isfirst = true;
					bodyposrectfile.insert(make_pair(j, filename));
					bodyposrect.insert(make_pair(j, _rect));
					j++;
				//	cout << "first insert rect " << _rect.x <<","<< _rect.y << "," << _rect.width << "," << _rect.height << endl;
				}
				else
				{

				//	cout << "88888888888888888888" << endl;
					if (m_svodescIdx == fframeidx && isfirst == true)
					{

				//		cout << j << "sfsdfsdfs insert rect " << endl;
						bodyposrectfile.insert(make_pair(j, filename));
						bodyposrect.insert(make_pair(j, _rect));
				//		cout << j << "insert rect " << _rect.x << "," << _rect.y << "," << _rect.width << "," << _rect.height << endl;
						j++;

					}
				}
			}
			else
			{
			
				if (bodyposrect.size() > 0)
					return true;
					//cout << "99999999999999999999999 " << endl;
					//isfirst = false;
					//bodyposrect.empty();
					//j = 0;
					//ImgIdx = -1;

			}
		}

		i++;
	}



	return false;
}

void openposeUtil::getPersonRect()
{
	//m_svoimgmap;  //
	//m_svopersonmap;  //

	//m_svoImgIdx;

	//m_svodescIdx;

	int isvoidx = m_svoImgIdx;

	int ic = 0;



	std::string strRect = "";
	cout << "start search use svoimgidx " << m_svoImgIdx << endl;

	cout << "map size is " << m_svoimgmap.size()<<"map size 2 is "<< m_svopersonmap.size() << endl;


	while (m_svodescIdx<= isvoidx)
	{
		try {
			
			strRect = m_svoimgmap[isvoidx];
			cout << "svodescidx " << m_svodescIdx << endl;
			cout << "dyn svodescidx " << isvoidx << endl;

			//cout << "file desc is " << strRect<<endl;
			if (strRect.compare("") != 0)
			{
				filterRect = getRect(strRect);
				cout << "filterRect " << filterRect.x << "," << filterRect.y << "," << filterRect.width << "," << filterRect.height << endl;
				personIdx = m_svopersonmap[isvoidx];
				m_svodescIdx = isvoidx;
				return;
			}

		}
		catch (std::out_of_range & const e)
		{
			//std::cerr << e.what() << std::endl;
			cout << "do not find svo desc item " << isvoidx <<" frame"<< endl;
		}
		isvoidx--;		
	//	ic++;		
	}

}


// The 3D of the point is not directly taken 'as is'. If the measurement isn't valid, we look around the point in 2D to find a close point with a valid depth

sl::float4 openposeUtil::getPatchIdx(const int& center_i, const int& center_j, sl::Mat & xyzrgba) {
	sl::float4 out(NAN, NAN, NAN, NAN);
	bool valid_measure;
	int i, j;

	const int R_max = 10;

	for (int R = 0; R < R_max; R++) {
		for (int y = -R; y <= R; y++) {
			for (int x = -R; x <= R; x++) {
				i = center_i + x;
				j = center_j + y;
				xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::CPU);
				valid_measure = isfinite(out.z);
				if (valid_measure) return out;
			}
		}
	}

	out = sl::float4(NAN, NAN, NAN, NAN);
	return out;
}



//// 计算得到系统需要的各种角度

void openposeUtil::calcmanpose(op::Array<float> & poseKeypoints, sl::Mat & xyz)
{
	// Common parameters needed
	const auto numberPeopleDetected = poseKeypoints.getSize(0);
	const auto numberBodyParts = poseKeypoints.getSize(1);
	std::vector<int> partsLink;

	switch (model_kp_number) {
	case 15:
		//https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/media/keypoints_pose_18.png
		partsLink = {
			0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 14, 14, 8, 8, 9, 9, 10, 14, 11, 11, 12, 12, 13
		};
		break;
	case 18:
		//https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/media/keypoints_pose_18.png
		partsLink = {
			//0, 1,
			2, 1,
			1, 5,
			8, 11,
			1, 8,
			11, 1,
			8, 9,
			9, 10,
			11, 12,
			12, 13,
			2, 3,
			3, 4,
			5, 6,
			6, 7,
			//0, 15,
			//15, 17,
			//0, 14,
			//14, 16,
			16, 1,
			17, 1,
			16, 17
		};
		break;

	case 25:
		//https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/media/keypoints_pose_25.png

		///std::vector<int> partsLink;
		partsLink = {
			0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 8, 12, 12,
			13, 13, 14, 14, 19, 19, 20, 14, 21, 8, 9, 9, 10, 10, 11, 11, 24,
			11, 22, 22, 23, 0, 16, 0, 15, 15, 17, 16, 18
		};
		break;

	}

	sl::float4 v1, v2;
	int i, j;

	std::vector<sl::float3> vertices;
	std::vector<sl::float3> clr;

	_manpose->init();
	for (int person = 0; person < numberPeopleDetected; person++) {

		std::map<int, sl::float4> keypoints_position; // 3D + score for each keypoints

		sl::float4 center_gravity(0, 0, 0, 0);
		int count = 0;
		float score;

		for (int k = 0; k < numberBodyParts; k++) {

			score = poseKeypoints[{person, k, 2}
			];
			keypoints_position[k] = sl::float4(NAN, NAN, NAN, score);

			if (score < FLAGS_render_threshold) continue; // skip low score

			i = round(poseKeypoints[{person, k, 0}
			]);
			j = round(poseKeypoints[{person, k, 1}
			]);

#if PATCH_AROUND_KEYPOINT
			xyz.getValue<sl::float4>(i, j, &keypoints_position[k], sl::MEM::CPU);
			if (!isfinite(keypoints_position[k].z))
				keypoints_position[k] = getPatchIdx((const int)i, (const int)j, xyz);
#else
			xyz.getValue<sl::float4>(i, j, &keypoints_position[k], sl::MEM::CPU);
#endif
			keypoints_position[k].w = score; // the score was overridden by the getValue

			if (score >= FLAGS_render_threshold && isfinite(keypoints_position[k].z)) {
				center_gravity += keypoints_position[k];
				count++;
			}
		}

		///////////////////////////
		center_gravity.x /= (float)count;
		center_gravity.y /= (float)count;
		center_gravity.z /= (float)count;

#if DISPLAY_BODY_BARYCENTER
		float size = 0.1;
		vertices.emplace_back(center_gravity.x, center_gravity.y + size, center_gravity.z);
		vertices.emplace_back(center_gravity.x, center_gravity.y - size, center_gravity.z);


		vertices.emplace_back(center_gravity.x + size, center_gravity.y, center_gravity.z);
		vertices.emplace_back(center_gravity.x - size, center_gravity.y, center_gravity.z);


		vertices.emplace_back(center_gravity.x, center_gravity.y, center_gravity.z + size);
		vertices.emplace_back(center_gravity.x, center_gravity.y, center_gravity.z - size);

#endif
		/////////////////////////// 
		int cvx = 0, cvy = 0;
		Angle _angle;

		InitAngle(_angle);

		sl::float4 pv0, pv1, pv2, pv3;
		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		int vidx[4];
		vidx[0] = _manpose->getbody_part_index("none_neckmidhip_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("none_neckmidhip_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("none_neckmidhip_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("none_neckmidhip_plane", SECOND_SEC_END);

		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("臀部上脊柱骨与地面平面法线的夹角 none_neckmidhip_plane");

		logInfo(vidx);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_midhip_plane_normal(_angle, cvx, cvy, "臀部上脊柱骨与地面平面法线的夹角");



		//logInfo(_manpose->getang_midhip_plane_normal());
		logInfo("臀部上脊柱骨与地面平面法线的夹角 none_neckmidhip_plane  over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_hipknee_to_midhip", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_hipknee_to_midhip", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_hipknee_to_midhip", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_hipknee_to_midhip", SECOND_SEC_END);

		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];


		logInfo("大腿与上身的夹角右 right_hipknee_to_midhip");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_hipknee_midhip(_angle, cvx, cvy, "大腿与上身的夹角右", BODY_RIGHT);



		logInfo("right_hipknee_to_midhip");
		//logInfo(_manpose->getang_hipknee_midhip(BODY_RIGHT));

		logInfo("大腿与上身的夹角右 right_hipknee_to_midhip  over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_hipknee_to_midhip", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_hipknee_to_midhip", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_hipknee_to_midhip", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_hipknee_to_midhip", SECOND_SEC_END);

		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];


		logInfo("大腿与上身的夹角左 right_hipknee_to_midhip");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_hipknee_midhip(_angle, cvx, cvy, "大腿与上身的夹角左", BODY_LEFT);



		logInfo("left_hipknee_to_midhip");
		//logInfo(_manpose->getang_hipknee_midhip(BODY_LEFT));

		logInfo("大腿与上身的夹角左 right_hipknee_to_midhip over");

		////////---------------------------------------------

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_hipknee_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_hipknee_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_hipknee_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_hipknee_plane", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];


		logInfo("大腿与地面平面法线的夹角右 right_hipknee_plane");

		logInfo(vidx);

		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_hipknee_plane_normal(_angle, cvx, cvy, "大腿与地面平面法线的夹角右", BODY_RIGHT);



		logInfo("right_hipknee_plane");
		//logInfo(_manpose->getang_hipknee_plane_normal(BODY_RIGHT));

		logInfo("大腿与地面平面法线的夹角右 right_hipknee_plane over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_hipknee_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_hipknee_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_hipknee_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_hipknee_plane", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("大腿与地面平面法线的夹角左 left_hipknee_plane");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_hipknee_plane_normal(_angle, cvx, cvy, "大腿与地面平面法线的夹角左", BODY_LEFT);



		logInfo("left_hipknee_plane");
		//logInfo(_manpose->getang_hipknee_plane_normal(BODY_LEFT));

		logInfo("大腿与地面平面法线的夹角左 left_hipknee_plane over");


		////////---------------------------------------------

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_kneeankle_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_kneeankle_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_kneeankle_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_kneeankle_plane", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];


		logInfo("小腿与地面平面法线的夹角右 right_kneeankle_plane");

		logInfo(vidx);

		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_kneeankle_plane_normal(_angle, cvx, cvy, "小腿与地面平面法线的夹角右", BODY_RIGHT);



		logInfo("right_kneeankle_plane");
		//logInfo(_manpose->getang_kneeankle_plane_normal(BODY_RIGHT));

		logInfo("小腿与地面平面法线的夹角右 right_kneeankle_plane  over");


		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_kneeankle_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_kneeankle_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_kneeankle_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_kneeankle_plane", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];


		logInfo("小腿与地面平面法线的夹角左 left_kneeankle_plane");

		logInfo(vidx);
		InitAngle(_angle);

		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);

		_manpose->setang_kneeankle_plane_normal(_angle, cvx, cvy, "小腿与地面平面法线的夹角左", BODY_LEFT);



		logInfo("left_kneeankle_plane");
		//logInfo(_manpose->getang_kneeankle_plane_normal(BODY_LEFT));

		logInfo("小腿与地面平面法线的夹角左 left_kneeankle_plane  over");


		////////---------------------------------------------

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_kneeankle_hipknee", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_kneeankle_hipknee", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_kneeankle_hipknee", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_kneeankle_hipknee", SECOND_SEC_END);

		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];



		logInfo("小腿与大腿的夹角右 right_kneeankle_hipknee");

		logInfo(vidx);

		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);
		_manpose->setang_kneeankle_hipknee(_angle, cvx, cvy, "小腿与大腿的夹角右", BODY_RIGHT);



		logInfo("right_kneeankle_hipknee");
		//logInfo(_manpose->getang_kneeankle_hipknee(BODY_RIGHT));

		logInfo("小腿与大腿的夹角右 right_kneeankle_hipknee over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_kneeankle_hipknee", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_kneeankle_hipknee", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_kneeankle_hipknee", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_kneeankle_hipknee", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];



		logInfo("小腿与大腿的夹角左 left_kneeankle_hipknee");

		logInfo(vidx);

		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);

		_manpose->setang_kneeankle_hipknee(_angle, cvx, cvy, "小腿与大腿的夹角左", BODY_LEFT);



		logInfo("left_kneeankle_hipknee");
		//logInfo(_manpose->getang_kneeankle_hipknee(BODY_LEFT));

		logInfo("小腿与大腿的夹角左 left_kneeankle_hipknee  over");

		////////---------------------------------------------

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_anklebigtoe_kneeankle", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_anklebigtoe_kneeankle", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_anklebigtoe_kneeankle", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_anklebigtoe_kneeankle", SECOND_SEC_END);

		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];



		logInfo("踝关节与小腿的夹角右 right_anklebigtoe_kneeankle");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);


		_manpose->setang_anklebigtoe_kneeankle(_angle, cvx, cvy, "踝关节与小腿的夹角右", BODY_RIGHT);



		logInfo("right_anklebigtoe_kneeankle");
		//logInfo(_manpose->getang_anklebigtoe_kneeankle(BODY_RIGHT));


		logInfo("踝关节与小腿的夹角右 right_anklebigtoe_kneeankle over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_anklebigtoe_kneeankle", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_anklebigtoe_kneeankle", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_anklebigtoe_kneeankle", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_anklebigtoe_kneeankle", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("踝关节与小腿的夹角左 left_anklebigtoe_kneeankle");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);

		_manpose->setang_anklebigtoe_kneeankle(_angle, cvx, cvy, "踝关节与小腿的夹角左", BODY_LEFT);



		logInfo("left_anklebigtoe_kneeankle");
		//logInfo(_manpose->getang_anklebigtoe_kneeankle(BODY_LEFT));

		logInfo("踝关节与小腿的夹角左 left_anklebigtoe_kneeankle over");

		////////---------------------------------------------

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_anklebigtoe_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_anklebigtoe_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_anklebigtoe_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_anklebigtoe_plane", SECOND_SEC_END);



		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("踝关节与地面平面法线的夹角右 right_anklebigtoe_plane");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);


		_manpose->setang_anklebigtoe_plane_normal(_angle, cvx, cvy, "踝关节与地面平面法线的夹角右", BODY_RIGHT);



		logInfo("right_anklebigtoe_plane");
		//logInfo(_manpose->getang_anklebigtoe_plane_normal(BODY_RIGHT));

		logInfo("踝关节与地面平面法线的夹角右 right_anklebigtoe_plane over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_anklebigtoe_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_anklebigtoe_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_anklebigtoe_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_anklebigtoe_plane", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("踝关节与地面平面法线的夹角左 left_anklebigtoe_plane");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);

		_manpose->setang_anklebigtoe_plane_normal(_angle, cvx, cvy, "踝关节与地面平面法线的夹角左", BODY_LEFT);



		logInfo("left_anklebigtoe_plane");
		//logInfo(_manpose->getang_anklebigtoe_plane_normal(BODY_LEFT));


		logInfo("踝关节与地面平面法线的夹角左 left_anklebigtoe_plane over");


		////////---------------------------------------------

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("right_heel_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("right_heel_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("right_heel_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("right_heel_plane", SECOND_SEC_END);



		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("脚底板与地面平面法线的夹角右 right_heel_plane");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);


		_manpose->setang_anklebigtoe_plane_normal(_angle, cvx, cvy, "脚底板与地面平面法线的夹角右", BODY_RIGHT);



		logInfo("right_heel_plane");
		//logInfo(_manpose->getang_anklebigtoe_plane_normal(BODY_RIGHT));

		logInfo("脚底板与地面平面法线的夹角右 right_heel_plane over");

		pv0 = sl::float4(NAN, NAN, NAN, 0);
		pv1 = sl::float4(NAN, NAN, NAN, 0);
		pv2 = sl::float4(NAN, NAN, NAN, 0);
		pv3 = sl::float4(NAN, NAN, NAN, 0);

		vidx[0] = _manpose->getbody_part_index("left_heel_plane", FIRST_SEC_START);
		vidx[1] = _manpose->getbody_part_index("left_heel_plane", FIRST_SEC_END);

		vidx[2] = _manpose->getbody_part_index("left_heel_plane", SECOND_SEC_START);
		vidx[3] = _manpose->getbody_part_index("left_heel_plane", SECOND_SEC_END);


		cvx = poseKeypoints[{0, vidx[1], 0}];
		cvy = poseKeypoints[{0, vidx[1], 1}];

		logInfo("脚底板与地面平面法线的夹角左 left_heel_plane");

		logInfo(vidx);
		InitAngle(_angle);
		_angle = calctwovectorang(vidx, pv0, pv1, pv2, pv3, keypoints_position);

		_manpose->setang_anklebigtoe_plane_normal(_angle, cvx, cvy, "脚底板与地面平面法线的夹角左", BODY_LEFT);



		logInfo("left_heel_plane");
		//logInfo(_manpose->getang_anklebigtoe_plane_normal(BODY_LEFT));


		logInfo("踝关节与地面平面法线的夹角左 left_heel_plane over");


		calcplaneangle(keypoints_position);

		sl::double3 jointpos;
		sl::float4 jointtemp;

		sl::float4 v10=sl::float4(NAN, NAN, NAN, 0), v13= sl::float4(NAN, NAN, NAN, 0);
		sl::float4 v21 = sl::float4(NAN, NAN, NAN, 0), v24 = sl::float4(NAN, NAN, NAN, 0);

		for (int part = 0; part < poseKeypoints.getSize(1); part++)
		{

			logInfo("body part index");
			logInfo(part);

			jointtemp = keypoints_position[part];

			jointpos.x = jointtemp.x;
			jointpos.y = jointtemp.y;
			jointpos.z = jointtemp.z;
			if (part == 10)
				v10 = jointtemp;
			else if(part==13)
				v13 = jointtemp;
			else if (part == 21)
				v21 = jointtemp;
			else if (part == 24)
				v24 = jointtemp;

			_manpose->bodyjointposmap.insert(make_pair(part, jointpos));

		}

		if (isnan(v10.x) || isnan(v10.y) || isnan(v10.z) || isnan(v13.x) || isnan(v13.y) || isnan(v13.z))
		{
			_manpose->twokneev = -1;
		}
		else
		{
			double pl = sqrt((v10.x - v13.x) * (v10.x - v13.x) + (v10.y - v13.y) * (v10.y - v13.y) + (v10.z - v13.z) * (v10.z - v13.z));
			cout << "pl is " << pl << endl;
			_manpose->twokneev = fabs(0.2- pl) / 0.2;
			cout << "pl two is " << _manpose->twokneev << endl;
		}

		if (isnan(v21.x) || isnan(v21.y) || isnan(v21.z) || isnan(v24.x) || isnan(v24.y) || isnan(v24.z))
		{
			_manpose->twoheelv = -1;
		}
		else
		{
			double pl = sqrt((v21.x - v24.x) * (v21.x - v24.x) + (v21.y - v24.y) * (v21.y - v24.y) + (v21.z - v24.z) * (v21.z - v24.z));
			cout << "jpl is " << pl << endl;
			_manpose->twoheelv = fabs(0.1 - pl) / 0.1;
			cout << "jpl two is " << _manpose->twoheelv << endl;
		}

		//for (int part = 0; part < partsLink.size() - 1; part += 2) {
		//	v1 = keypoints_position[partsLink[part]];
		//	v2 = keypoints_position[partsLink[part + 1]];

		//	IsUseful(v1, v2, center_gravity);
		//}
	}
	//peopleObj.setVert(vertices, clr);



}

void openposeUtil::InitAngle(Angle& _angle)
{
	_angle.angle = NAN;
	_angle.anglexoy = NAN;
	_angle.angleyoz = NAN;
}

void openposeUtil::calcplaneangle(std::map<int, sl::float4> keypoints_position)
{
	//sl::float4 pv0, pv1, pv2, pv3;


	int vidx[2];
	vidx[0] = _manpose->getbody_part_index("none_neckmidhip_plane", FIRST_SEC_START);
	vidx[1] = _manpose->getbody_part_index("none_neckmidhip_plane", FIRST_SEC_END);



	sl::double3 pt1 = sl::double3(0, 0, 0);
	sl::double3 ns = fromfloat3(plane.getNormal());
	sl::double3 ptcode8 = sl::double3(NAN, NAN, NAN);
	sl::double3 ptcode1 = sl::double3(NAN, NAN, NAN);

	sl::double3 pt2 = sl::double3(NAN, NAN, NAN);
	sl::double3 pt3 = sl::double3(NAN, NAN, NAN);
	sl::double3 pt4 = sl::double3(NAN, NAN, NAN);
	ptcode1 = fromfloat4(keypoints_position[vidx[0]]);
	ptcode8 = fromfloat4(keypoints_position[vidx[1]]);


	//pt1= sl::double3(1, 0, 0);
	//pt2 = sl::double3(0, 1, 0);
	//pt3 = sl::double3(0, 0, 1);

	pt2 = ns;




	pt4 = ns - ptcode8;

	plane1.setplanefromthreepoint(pt1, pt2, ptcode8);
	plane2.setplanefromthreepoint(ptcode1, pt4, ptcode8);

	logInfo("a1 = " + doubleTostring(plane1.a(), 8));
	logInfo("b1 = " + doubleTostring(plane1.b(), 8));
	logInfo("c1 = " + doubleTostring(plane1.c(), 8));
	logInfo("d1 = " + doubleTostring(plane1.d(), 8));

	logInfo("a2 = " + doubleTostring(plane2.a(), 8));
	logInfo("b2 = " + doubleTostring(plane2.b(), 8));
	logInfo("c2 = " + doubleTostring(plane2.c(), 8));
	logInfo("d2 = " + doubleTostring(plane2.d(), 8));


	//plane.setplanefromthreepoint(pt1, pt2, pt3);

	/*cout << "a =" << DoubleToString(plane.a(), 8);
	cout << "b =" << DoubleToString(plane.b(), 8);
	cout << "c =" << DoubleToString(plane.c(), 8);
	cout << "d =" << DoubleToString(plane.d(), 8);*/
	cline3d line;

	line.setOriginPt(ptcode1);

	sl::double3 direc = ptcode1 - ptcode8;

	line.setDirection(direc);

	bool br = false;
	br = plane1.LineIntersectPlane(line);

	br = plane2.LineIntersectPlane(line);

	double rt = plane1.pointistoponplane(ptcode1 - plane1.getintpt());
	//vidx[2] = _manpose->getbody_part_index("none_neckmidhip_plane", SECOND_SEC_START);
	//vidx[3] = _manpose->getbody_part_index("none_neckmidhip_plane", SECOND_SEC_END);

	_manpose->setang_plane1_left_right(rt, 0, 0, "左右区分");

	if (rt > 0)
	{
		logInfo("point is plane left");
	}
	else if (rt == 0)
	{
		logInfo("point is plane1 ");
	}
	else
	{
		logInfo("point is plane right");
	}


	double rt2 = plane2.pointistoponplane(ptcode1 - plane2.getintpt());

	_manpose->setang_plane2_before_after(rt2, 0, 0, "前后区分");


	if (rt2 > 0)
	{
		logInfo("point is plane after");
	}
	else if (rt2 == 0)
	{
		logInfo("point is plane2 ");
	}
	else
	{
		logInfo("point is plane before");
	}

}

////计算pv0,pv1 和 pv2,pv3之间的夹角，如果是平面法线则pv3不考虑
Angle openposeUtil::calctwovectorang(int  vidx[4], sl::float4& pv0, sl::float4& pv1, sl::float4& pv2, sl::float4& pv3, std::map<int, sl::float4>& keypoints_position)
{
	sl::float3 pn;
	////地板平面法线



	Angle angle;

	InitAngle(angle);
	if (vidx[2] == 999 || vidx[3] == 1000)
	{
		pn = plane.getNormal();
		pv2.x = pn.x;
		pv2.y = pn.y;
		pv2.z = pn.z;

		pv0 = keypoints_position[vidx[0]];
		pv1 = keypoints_position[vidx[1]];
		logInfo("pv0.x y z is ");
		logInfo(pv0);


		logInfo("pv1.x y z is ");
		logInfo(pv1);


		logInfo("pv2.x y z is ");
		logInfo(pv2);

	}
	else
	{
		pv0 = keypoints_position[vidx[0]];
		pv1 = keypoints_position[vidx[1]];
		pv2 = keypoints_position[vidx[2]];
		pv3 = keypoints_position[vidx[3]];

		logInfo("pv0.x y z is ");
		logInfo(pv0);


		logInfo("pv1.x y z is ");
		logInfo(pv1);


		logInfo("pv2.x y z is ");
		logInfo(pv2);

		logInfo("pv3.x y z is ");
		logInfo(pv3);

	}


	bool retflag;
	retflag = AssertVectorIsNAN(pv0) && AssertVectorIsNAN(pv1) && AssertVectorIsNAN(pv2);
	if (retflag == true)
	{
		logInfo("计算姿态角度时候向量没有初始化. ddddddddddddddddddddddddddddddd");
		return angle;
	}

	sl::float4 v1 = pv0 - pv1;
	sl::float4 vn, v2;
	v2 = sl::float4(NAN, NAN, NAN, 0);
	vn = sl::float4(NAN, NAN, NAN, 0);


	if (AssertVectorIsNAN(pv3) == true)
	{
		vn.z = pv1.z;
		vn.x = pv1.x;

		vn.y = pv2.y;

		v2 = pv2;
		logInfo("000000000000000000");

	}
	else
	{
		v2 = pv3 - pv2;
		logInfo("++++++++++++++++++++");
	}

	logInfo("v1.x y z is ");
	logInfo(v1);
	logInfo("v2.x y z is ");
	logInfo(v2);

	/*v0.Unitize();
	v1.Unitize();*/
	logInfo("start calc angle");

	//logInfo("v1 normal is ");
	//logInfo(v1.square());
	//logInfo("v2 normal is ");
	//logInfo(v2.square());


	//double dot = sl::float4::dot(v1,v2)/(v1.norm()*v2.norm());
	sl::float4 v1u = getvector_unit(v1);
	sl::float4 v2u = getvector_unit(v2);

	logInfo("v1u");

	logInfo(v1u);

	logInfo("v2u");

	logInfo(v2u);


	//double dot = sl::float4::dot(v1u, v2u);

	double dot = getdotproduct(v1u, v2u);
	logInfo("dot is ");
	logInfo(dot);
	// Force the dot product of the two input vectors to
	// fall within the domain for inverse cosine, which
	// is -1 <= x <= 1. This will prevent runtime
	// "domain error" math exceptions.
	dot = (dot < -1.0 ? -1.0 : (dot > 1.0 ? 1.0 : dot));
	logInfo("dot1 is ");
	double _angle;
	_angle = acos(dot);


	logInfo("arcangle is ");
	logInfo(_angle);
	//double reang = (M_PI * 2) - angle;
		//* reflex_angle = (ON_PI * 2) - angle;

	logInfo("ang is ");
	//logInfo(reang);
	_angle = _angle * 360 / (2 * M_PI);

	logInfo(_angle);

	angle.angle = _angle;




	double dotxoy = getdotproductxoy(v1u, v2u);
	logInfo("dotxoy is ");
	logInfo(dotxoy);
	// Force the dot product of the two input vectors to
	// fall within the domain for inverse cosine, which
	// is -1 <= x <= 1. This will prevent runtime
	// "domain error" math exceptions.
	dotxoy = (dotxoy < -1.0 ? -1.0 : (dotxoy > 1.0 ? 1.0 : dotxoy));
	logInfo("dot1xoy is ");



	logInfo(dotxoy);

	_angle = acos(dotxoy);

	angle.anglexoy = _angle * 360 / (2 * M_PI);

	double dotyoz = getdotproductyoz(v1u, v2u);
	logInfo("dotyoz is ");
	logInfo(dotyoz);
	// Force the dot product of the two input vectors to
	// fall within the domain for inverse cosine, which
	// is -1 <= x <= 1. This will prevent runtime
	// "domain error" math exceptions.
	dotyoz = (dotyoz < -1.0 ? -1.0 : (dotyoz > 1.0 ? 1.0 : dotyoz));
	logInfo("dot1yoz is ");



	logInfo(dotyoz);

	_angle = acos(dotyoz);

	angle.angleyoz = _angle * 360 / (2 * M_PI);

	

	
	return angle;




	//pv0 = 0;


}

bool openposeUtil::AssertVectorIsNAN(sl::float4 & pv0)
{
	logInfo("test pv is NAN");
	logInfo(pv0.x);

	logInfo(pv0.y);

	logInfo(pv0.z);


	if (isnan(pv0.x) == true || isnan(pv0.y) == true || isnan(pv0.z) == true)
	{
		logInfo("check vector start");
		logInfo(pv0.x);
		logInfo(pv0.y);
		logInfo(pv0.z);
		logInfo("check vector end");
		return true;

	}
	else
		return false;

}



bool openposeUtil::IsUseful(sl::float4 & v1, sl::float4 & v2, sl::float4 & center_gravity)
{
	// Filtering 3D Skeleton
	// Compute euclidian distance
	float distance = sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y) + (v1.z - v2.z) * (v1.z - v2.z));

	float distance_gravity_center = sqrt(pow((v2.x + v1.x) * 0.5f - center_gravity.x, 2) +
		pow((v2.y + v1.y) * 0.5f - center_gravity.y, 2) +
		pow((v2.z + v1.z) * 0.5f - center_gravity.z, 2));
	if (isfinite(distance_gravity_center) && distance_gravity_center < MAX_DISTANCE_CENTER && distance < MAX_DISTANCE_LIMB) {
		return true;
		//vertices.emplace_back(v1.x, v1.y, v1.z);
		//vertices.emplace_back(v2.x, v2.y, v2.z);
		//clr.push_back(generateColor(person));
		//clr.push_back(generateColor(person));
	}
	else
		return false;
}

void openposeUtil::fill_ptcloud(sl::Mat & xyzrgba) {
	std::vector<sl::float3> pts;
	std::vector<sl::float3> clr;
	int total = xyzrgba.getResolution().area();

	float factor = 1;

	pts.resize(total / factor);
	clr.resize(total / factor);

	sl::float4 * p_mat = xyzrgba.getPtr<sl::float4>(sl::MEM::CPU);

	sl::float3 * p_f3;
	sl::float4 * p_f4;
	unsigned char* color_uchar;

	int j = 0;
	for (int i = 0; i < total; i += factor, j++) {
		p_f4 = &p_mat[i];
		p_f3 = &pts[j];
		p_f3->x = p_f4->x;
		p_f3->y = p_f4->y;
		p_f3->z = p_f4->z;
		p_f3 = &clr[j];
		color_uchar = (unsigned char*)& p_f4->w;
		p_f3->x = color_uchar[0] * 0.003921569; // /255
		p_f3->y = color_uchar[1] * 0.003921569;
		p_f3->z = color_uchar[2] * 0.003921569;
	}
	//cloud.setVert(pts, clr);
}

void openposeUtil::runZed() {
	
	cout << "runzed in ..........." << endl;
	sl::Mat depth_img_buffer, depth_buffer, depth_buffer2;
	op::Array<float> outputArray, outputArray2;
	cv::Mat inputImage, depthImage, inputImageRGBA, outputImage;

	// ---- OPENPOSE INIT (io data + renderer) ----
	op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
	op::CvMatToOpInput cvMatToOpInput;
	op::CvMatToOpOutput cvMatToOpOutput;

	op::PoseCpuRenderer poseRenderer{ poseModel, (float)FLAGS_render_threshold, !FLAGS_disable_blending, (float)FLAGS_alpha_pose };
	op::OpOutputToCvMat opOutputToCvMat;
	// Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
	poseRenderer.initializationOnThread();

	// Init
	//sl::Resolution image_resolution(image_width, image_height);

	imageSize = op::Point<int>{ poseImage_width, poseImage_height };
	// Get desired scale sizes
	std::vector<op::Point<int>> netInputSizes;
	double scaleInputToOutput;
	op::Point<int> outputResolution;

	cout << "image SIze is " << image_width << "*" << image_height << endl;

	std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution) = scaleAndSizeExtractor.extract(imageSize);

	bool chrono_zed = false;

	// Prepare new image size to retrieve half-resolution images
	Resolution image_size;
	int new_width, new_height;


	image_size = zed.getCameraInformation().camera_configuration.resolution;
	



	new_width = image_size.width / 1;
	new_height = image_size.height / 1;

	cout << "img pose" << new_width << " rows " << new_height << endl;

	//cout << "input img size " << netInputSizes.size() << endl;
	//cout << "input img p0 " << netInputSizes[0].x << "," << netInputSizes[0].y << endl;
	//cout << "input img p1 " << netInputSizes[1].x << "," << netInputSizes[1].y << endl;
	//cout << "input img p2 " << netInputSizes[2].x << "," << netInputSizes[2].y << endl;
	//cout << "input img p3 " << netInputSizes[3].x << "," << netInputSizes[3].y << endl;


	image_height = new_height;
	image_width = new_width;

	/*logInfo("------------width--------");
	logInfo(new_width);

	logInfo("------------width--------");
	logInfo(new_height);*/


	new_width = poseImage_width / 1;
	new_height = poseImage_height / 1;



	sl::Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);

	image_ocv = slMat2cvMat(image_zed);
	sl::Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_img_buffer);
	cv::Mat rimg, rrimg;
	sl::Resolution new_image_size;
	new_image_size.height = new_height;
	new_image_size.width = new_width;


	new_image_size.height = poseImage_height;
	new_image_size.width = poseImage_width;



	//logInfo("zed frame is ");
	//logInfo(zed.getSVONumberOfFrames());
	//int zzz = zed.getSVONumberOfFrames();
	
	//cout << "zed frame is " << zzz << endl;
	cout << bshow << endl;

	int endnum = 0;

	
	while (!quit) {
		INIT_TIMER
			try {
			if (need_new_image) {


				//selectSvoPosAndSet();
			/*	if (m_inputDataType == InputDataType::SVO_TXT && m_blastsvo)
				{
					Exitzed(chrono_zed);
					sl::sleep_ms(1);
				}*/
				if (zed.grab() == ERROR_CODE::SUCCESS)
				{



					zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
					//zed.retrieveImage(img_buffer, VIEW::LEFT, sl::MEM::CPU, image_resolution);

					//cout << "get img----------" << endl;
					data_out_mtx.lock();
					depth_buffer2 = depth_buffer;
					data_out_mtx.unlock();


					//logInfo("zed svo files position is ");
					//logInfo(zed.getSVOPosition());
					//logInfo("over ");
					//cout << "get img idx----------" <<zed.getSVOPosition()<<" file img idx is" << m_svoIdx << endl;
					//char key = (char)cv::waitKey(5);
					//sl::sleep_ms(2);
					zed.retrieveMeasure(depth_buffer, MEASURE::XYZRGBA, sl::MEM::CPU, new_image_size);

					//logInfo(zed.getCurrentFPS());
					//logInfo("        \r");

					//inputImageRGBA = slMat2cvMat(img_buffer);

					//key = (char)cv::waitKey(5);

				//	sl::sleep_ms(2);
					//new
					//try {
					//	char* cplusplus = new char[20];
					//}
					//catch (const std::bad_alloc) {}

					////malloc
					//char* c = (char*)malloc(20 * sizeof(char));
					///cout << "1111111111111111111----------" << endl;
					if (!image_ocv.empty())
					{

						if (bshow)
						{

						//	cout << "show opencv"<< endl;
							cv::imshow("Image", image_ocv);
							cv::waitKey(5);
						}
						//zedimgnum++;
						cv::cvtColor(image_ocv, inputImage, cv::COLOR_RGBA2RGB);
						//if (bsaveZed) {
						//
						///	cv::resize(inputImage, inputImage, cv::Size(poseImage_width, poseImage_height));
						//	outzedVideo.write(rrimg);

						//	//logInfo("start save zed video");
						////	sl::sleep_ms(1);			

						//}



						if (FLAGS_depth_display)
							zed.retrieveImage(depth_img_buffer, VIEW::DEPTH, sl::MEM::CPU, new_image_size);

						if (FLAGS_opencv_display) {
							data_out_mtx.lock();
							outputArray2 = outputArray;
							data_out_mtx.unlock();
							outputArray = cvMatToOpOutput.createArray(op::Matrix(&inputImage), scaleInputToOutput, outputResolution);
						}
					//	cout << "1111111111111111111111111111111" << endl;
						//this->getFilterRect(zed.getSVOPosition());
						if (m_inputDataType == InputDataType::SVO_TXT)
						{



							getPersonRect();


							fillouterRect(inputImage);

						}
					//	cout << "2222222222222222222222222222" << endl;
						data_in_mtx.lock();
						netInputArray = cvMatToOpInput.createArray(op::Matrix(&inputImage), scaleInputToNetInputs, netInputSizes);
						///if (bzero == true)
						//{
						
					  


						//}
						need_new_image = false;
						data_in_mtx.unlock();
					}
					ready_to_start = true;
					chrono_zed = true;
				}
				else
				{
					
					sl::sleep_ms(1);
					endnum++;
				}
				
			}
			else sl::sleep_ms(1);

			// -------------------------  RENDERING -------------------------------

			///cout << "--------------------------------------------" << endl;
			// Render poseKeypoints
			if (data_out_mtx.try_lock())
			{
			//	cout << "333333333333333333333333333" << endl;
				//fill_people_ogl(poseKeypoints, depth_buffer2);

				//if(m_bfilter)
				//manFilter.startFilter(poseKeypoints);
				if(bsavePose&&bzero)
				calcmanpose(poseKeypoints, depth_buffer2);
				//			viewer.update(peopleObj);

							//if (FLAGS_ogl_ptcloud) {
							//	fill_ptcloud(depth_buffer2);
							////	viewer.update(cloud);
							//}
				//cout << "444444444444444444444444444" << endl;
				if (FLAGS_opencv_display) {
					if (!outputArray2.empty())
					{
						poseRenderer.renderPose(outputArray2, poseKeypoints, scaleInputToOutput);

						// OpenPose output format to cv::Mat

						outputImage = *((cv::Mat*)opOutputToCvMat.formatToCvMat(outputArray2).getCvMat());
					}
					data_out_mtx.unlock();
					// Show results
					if (!outputImage.empty())
					{
						

						if(bsavePose&&bzero)
						{
						drawText(&outputImage, _manpose->getang_plane1_left_right(), BODY_LEFT);
						
						drawText(&outputImage, _manpose->getang_plane2_before_after());
						drawText(&outputImage, _manpose->getang_midhip_plane_normal());
						drawText(&outputImage, _manpose->getang_hipknee_midhip(BODY_LEFT), BODY_LEFT);
						drawText(&outputImage, _manpose->getang_hipknee_midhip(BODY_RIGHT));

						drawText(&outputImage, _manpose->getang_hipknee_plane_normal(BODY_LEFT), BODY_LEFT);
						drawText(&outputImage, _manpose->getang_hipknee_plane_normal(BODY_RIGHT));

						drawText(&outputImage, _manpose->getang_kneeankle_plane_normal(BODY_LEFT), BODY_LEFT);
						drawText(&outputImage, _manpose->getang_kneeankle_plane_normal(BODY_RIGHT));


						drawText(&outputImage, _manpose->getang_kneeankle_hipknee(BODY_LEFT), BODY_LEFT);
						drawText(&outputImage, _manpose->getang_kneeankle_hipknee(BODY_RIGHT));

						drawText(&outputImage, _manpose->getang_anklebigtoe_kneeankle(BODY_LEFT), BODY_LEFT);
						drawText(&outputImage, _manpose->getang_anklebigtoe_kneeankle(BODY_RIGHT));


						drawText(&outputImage, _manpose->getang_anklebigtoe_plane_normal(BODY_LEFT), BODY_LEFT);
						drawText(&outputImage, _manpose->getang_anklebigtoe_plane_normal(BODY_RIGHT));


						 if (m_inputDataType == InputDataType::SVO_TXT)
						 {
							 cv::Point ppv;
							 ppv.x = filterRect.x;
							 ppv.y = filterRect.y;
							 std::string personstr;
							 personstr.append("第").append(std::to_string(personIdx)).append("学员");
							 drawText(&outputImage, personstr, ppv,20,0,0,255);

						  }
						}

						//logInfo("img channel is ");
						//logInfo(outputImage.channels());

						if (bshow)
						{
							

							

							cv::imshow("Pose", outputImage);
							sl::sleep_ms(1);
						}
						if (bsavePose&&bzero) {

							cout << "5555555555555555555555555555555" << endl;

							//cout << "write pose" << endl;
							_manpose->frameidx = m_svoImgIdx;
							addmanpose();
							appendonejson();

							//if (m_inputDataType != InputDataType::SVO_TXT)
							m_svoImgIdx++;
							//poseimgnum++;
							//data_out_mtx.lock();
							/*logInfo("outputimage height");
							logInfo(outputImage.cols);
							logInfo("outputimage width");
							logInfo(outputImage.rows);*/
							cv::resize(outputImage, rimg, cv::Size(simage_width, simage_height));
							outposeVideo.write(rimg);


							if (endnum >0)
							{

								Exitzed(chrono_zed);

							}


							//logInfo("---------------------------------------------");
							//logInfo(image_width);
							//logInfo(image_height);
							//logInfo("---------------------------------------------");
							//logInfo("start save pose avi");
							//sl::sleep_ms(1);
							//sl::sleep_ms(int(1000.0 / 25));
							//Sleep(int(1000.0 / fps));


							//data_out_mtx.unlock();
						/*	if (frameidx >= maxframecount)
							{

								bsavePose = false;
								outputVideo.release();
								logInfo("over here pose avi");
							}*/
						}
						else
						{
							bzero = true;

							if (m_inputDataType == InputDataType::SVO_TXT)
							{
								m_svodescIdx = 0;
								m_svoImgIdx = 0;
								if (isopen)
								{
									infile.clear(infile.goodbit);
									infile.seekg(ios::beg);
								}
							}
							else
							{ 
							zed.setSVOPosition(0);

							//zedimgnum=0;

							m_svoImgIdx =0;
//							poseimgnum=0;
							}

							cout << "true start frame idx 0" << endl;
						}

					}
					//  cv::put
				/*	if (FLAGS_depth_display)
						cv::imshow("Depth", depth_image_ocv);*/

						//cv::waitKey(10);
						//cout << "pose num is " << poseimgnum << endl;
				}
			}

			if (chrono_zed) {
				//STOP_TIMER("ZED")
				chrono_zed = false;
			}
		}
		catch (int)
		{
			cout << "int calc error"<< endl;
			logInfo("int calc error");
		}
		catch (float)
		{
			cout << "float calc error" << endl;
			logInfo("float calc error");
		}
		catch (double)
		{
			cout << "double calc error" << endl;
			logInfo("double calc error");
		}
		catch(std::exception ex)
		{

			cout <<"-------execption1----------"<< endl;
			cout << ex.what() << endl;
			logInfo(ex.what());

			//Exitzed(chrono_zed);
		}
		catch (cv::Exception ex)
		{
			cout << "-------execption2----------" << endl;
			cout << ex.what() << endl;
			logInfo(ex.what());
			//Exitzed(chrono_zed);
		}
		catch (runtime_error Runtime_error)
		{
			cout << "-------execption3----------" << endl;
			cout << Runtime_error.what() << endl;

			logInfo(Runtime_error.what());
			//Exitzed(chrono_zed);
		}
		
		//cout << "sleep 1 ms"<<endl;
	}
	destroyAllWindows();
}

void openposeUtil::Exitzed(bool& chrono_zed)
{

	if (m_inputDataType == InputDataType::SVO_TXT)
	{
		if (infile.is_open())
		{
			infile.close();
			isopen = false;
		}
		m_svoimgmap.clear();  //
		m_svopersonmap.clear();  //
	}
	if (!totaljsons.empty())
		writeJson();
	destroyAllWindows();

	if (chrono_zed) {
		//STOP_TIMER("ZED")
		chrono_zed = false;
	}
	stopposeservice();
}

void openposeUtil::closezeddevice()
{
}

void openposeUtil::startposeservice(std::string svo_files, bool isshow, int x, int y, int width, int height)
{

	


	

	reinit();


	bshow = isshow;

	// Set configuration parameters for the ZED
	InitParameters initParameters;
	///initParameters.camera_resolution = RESOLUTION::HD720;
	initParameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Might be GPU memory intensive combine with openpose
	initParameters.coordinate_units = unit;
	initParameters.camera_disable_self_calib = true;
	initParameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

	//initParameters.depth_stabilization = true;
	cout << svo_files << endl;
	logInfo(svo_files);
	initParameters.input.setFromSVOFile(svo_files.c_str());


	//logInfo("1111111111111111111111111111111");
	// Open the camera
	ERROR_CODE err = zed.open(initParameters);
	if (err != sl::ERROR_CODE::SUCCESS) {
		//logInfo(err);
		logInfo("open camera error");
		cout << "open camera error" << endl;
		zed.close();
		return; // Quit if an error occurred
	}


	Resolution image_size;
	image_size = zed.getCameraInformation().camera_configuration.resolution;
	ReInitResolution(image_size);
	//bsavePose = true;
	//logInfo("------------------------------------+++++");


	std::string tmpdir = dirutil::GetCurrentWorkingDir() + "\\" + getpose_data_Dir().c_str() + "\\";
	dirutil::createDirectory(tmpdir);
	sl::sleep_ms(2);

	logInfo(tmpdir);
	setpose_data_Dir(tmpdir);


	ffout.open(getpose_data_File().c_str(), std::ios::app);

	//logInfo(ii);
	//outputVideo = new VideoWriter(getpose_avi_File(), VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(672, 376),true);


	//setpose_data_Dir(tmpdir);


	logInfo("succeed open zed.");
	if (FLAGS_estimate_floor_plane)
		initFloorZED(zed);


	//logInfo("2222222222222222222222222222222");
	// Initialize OpenGL viewer
//	viewer.init();

	// init OpenPose
	//cout << "OpenPose : loading models..." << endl;

	logInfo("OpenPose : loading models...");
	// ------------------------- INITIALIZATION -------------------------
	// Read Google flags (user defined configuration)
	outputSize = op::flagsToPoint(std::string(FLAGS_output_resolution).c_str(), "-1x-1");
	netInputSize = op::flagsToPoint(std::string(FLAGS_net_resolution).c_str(), "-1x368");

	cout << netInputSize.x << "x" << netInputSize.y << endl;
	netOutputSize = netInputSize;
	poseModel = op::flagsToPoseModel(std::string(FLAGS_model_pose).c_str());

	//logInfo("fffffffffffffffffffffffffffffffffffffffffffffff");

	if (FLAGS_model_pose == "COCO") model_kp_number = 18;
	else if (FLAGS_model_pose.find("MPI") != std::string::npos) model_kp_number = 15;
	else if (FLAGS_model_pose == "BODY_25") model_kp_number = 25;


	//logInfo("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG");
	// Check no contradictory flags enabled
	if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.) op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
	if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1) op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.", __LINE__, __FUNCTION__, __FILE__);


	


	//bsavePose = true;
	//logInfo("------------------------------------+++++");


	//int ii = VideoWriter::fourcc('M', 'J', 'P', 'G');
	//logInfo("++++++++++++++++++++++++++++++++++++");
	//logInfo(ii);
	//outputVideo = new VideoWriter(getpose_avi_File(), VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(672, 376),true);

	logInfo(image_height);
	logInfo(image_width);



	cout <<"pose avi size:"<< image_width << "x" << image_height << endl;
	cout << poseImage_width << "x" << poseImage_height << endl;
	cout << simage_width << "x" << simage_height << endl;




	outposeVideo.release();
	outposeVideo.open(getpose_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 10, cv::Size(image_width, image_height), true);
	//outzedVideo.open(getzed_avi_File(), VideoWriter::fourcc('F', 'M', 'P', '4'), 10, cv::Size(image_width, image_height), true);

	if (!outposeVideo.isOpened())
	{
		logInfo("codec failed\n");
		cout << "codec failed\n" << endl;
		return;
	}
	else
	{ 
		logInfo("create pose avi ");
		cout << "create pose avi" << endl;
	}
		

	/*filterRect.x = x;
	filterRect.y = y;
	filterRect.width = width;
	filterRect.height = height;*/
	//saveposeavi_callback = std::thread(&openposeUtil::saveposeavi, this);
	//logInfo("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
	// Start ZED callback

	//startzedopenpose();


	startZED();
	////logInfo("end startZed");
	startOpenpose();
	///logInfo("IIIIIIIIIIII");
	// Set the display callback
	//glutCloseFunc(clean);
	//glutMainLoop();
}

void openposeUtil::stoppposewithexitsys() {


	if (quit == true)
	{
		logInfo("restop!!!!");
		return;
	}
	quit = true;

	cout << "start clean resource" << endl;
	//viewer.exit();


	openpose_callback.join();
	cout << "stop openpose callback" << endl;

	zed_callback.join();
	//cout << "22233333333333" << endl;


	if (zed.isOpened())
	{
		cout << "close zed " << endl;
		zed.close();
	}
	else
		cout << "zed not need close" << endl;


	cout << "11155555555555555555555555" << endl;
	//logInfo("exit Processing Thread");

}



int openposeUtil::getlogging_level()
{
	return FLAGS_logging_level;
}

bool openposeUtil::getdisable_multi_thread()
{
	return FLAGS_disable_multi_thread;
}

std::string openposeUtil::getoutput_resolution()
{
	std::string ss = FLAGS_output_resolution;
	return ss;

	//return op::String(FLAGS_output_resolution);
}

std::string openposeUtil::getmodel_folder()
{
	std::string ss = FLAGS_model_folder;
	return ss;


}

std::string openposeUtil::getwrite_json()
{
	std::string ss = FLAGS_write_json;
	return ss;

}

std::string openposeUtil::getnet_resolution()
{
	std::string ss = FLAGS_net_resolution;
	return ss;

}

std::string openposeUtil::getvideo()
{
	std::string ss = FLAGS_video;
	return ss;

}

std::string openposeUtil::getimage_dir()
{

	std::string ss = FLAGS_image_dir;
	return ss;


}

std::string openposeUtil::getwrite_images()
{

	std::string ss = FLAGS_write_images;
	return ss;


}

std::string openposeUtil::getwrite_video()
{

	std::string ss = FLAGS_write_video;
	return ss;
}

void openposeUtil::setlogging_level(int logging_level)
{
	FLAGS_logging_level = logging_level;
}

void openposeUtil::setdisable_multi_thread(bool disable_multi_thread)
{
	FLAGS_disable_multi_thread = disable_multi_thread;
}

void openposeUtil::setoutput_resolution(std::string output_resolution)
{
	FLAGS_output_resolution = output_resolution;
}

void openposeUtil::setmodel_folder(std::string model_folder)
{
	FLAGS_model_folder = model_folder;
}

void openposeUtil::setwrite_json(std::string write_json)
{
	FLAGS_write_json = write_json;
}

void openposeUtil::setnet_resolution(std::string net_resolution)
{
	FLAGS_net_resolution = net_resolution;
}

void openposeUtil::setvideo(std::string video)
{

	FLAGS_video = video;
}

void openposeUtil::setimage_dir(std::string image_dir)
{
	FLAGS_image_dir = image_dir;
}

void openposeUtil::setwrite_images(std::string write_images)
{
	FLAGS_write_images = write_images;
}

void openposeUtil::setwrite_video(std::string write_video)
{

	FLAGS_write_video = write_video;
}


extern "C" void clean()
{
	//Control::getCurrentThread()->close();
}