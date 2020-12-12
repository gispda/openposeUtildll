#ifndef MANPOSE_H
#define MANPOSE_H
#pragma once
#include <map>
#include <string.h>
#include <string>
#ifndef M_PI
#define M_PI 3.1416f
#endif

#include "cplane.h"
#include "cline3d.h"

using namespace std;

////
////这个类是跳伞人员的姿态角度封装类，一次传输一个该类即可。
////
////
////
enum Body {
	BODY_LEFT=1,BODY_RIGHT=0
};
struct Angle {
	double angle;
	double anglexoy;
	double angleyoz;
};

struct AngleInfo {
	Angle angle;

	int x;
	int y;

	sl::float3 pos;

	std::string desc;

	std::string descxoy;
	std::string descyoz;

	//double anglexoy;
	//double angleyoz;
	//std::string  sdesc;

	void init();
};

//struct JointPos {
//
//	std::map<int, sl::double3> bodyjointposmap;
//
//}
//;


enum SecIdx {
   FIRST_SEC_START=0,
   FIRST_SEC_END=1,
   SECOND_SEC_START=2,
   SECOND_SEC_END=3
};


class manpose
{
public:
	manpose();
	~manpose();


	void init();
	void configKeyData();
	int getbody_part_index(string key, SecIdx secidx=FIRST_SEC_START);

	std::map<int, sl::double3> bodyjointposmap;

	double twokneev;
	double twoheelv;
	int frameidx;
	

private:
	//cplane lrplane;
	//cplane bbplane;



	/// <summary>
	///map(string(key),string(data index))
	/// string format:
	///ang_midhip_plane_normal
	///key:neck_to_midhip
	///data index:1-8-999-1000    1(颈部) 8（臀部上脊柱骨） - 代表颈部连接臀部上脊柱骨就是脊柱   
	///999-1000就代表地板平面法线方向             
	///
	/// </summary>
	std::map<string,string> bodypartindexmap;

	////臀部上脊柱骨与地面平面法线的夹角
	////1(颈部),8(臀部上脊柱骨)  
	AngleInfo ang_midhip_plane_normal;



	////大腿与上身的夹角
	////9(臀部),10（膝盖）(右)12,13（左） to 1,8
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_hipknee_midhip[2];
	

	////大腿与地面平面法线的夹角
	////9(臀部),10（膝盖）(右)12,13（左） 
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_hipknee_plane_normal[2];
	

	////小腿与地面平面法线的夹角
	////10（膝盖）,11(踝关节)(右)13,14（左） 
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_kneeankle_plane_normal[2];


	////小腿与大腿的夹角
	////10（膝盖）,11(踝关节)(右) to 9(臀部),10（膝盖）;13,14（左） to 12,13（左）
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_kneeankle_hipknee[2];
	
	////踝关节与小腿的夹角
	////22（大脚趾）,11(踝关节)(右) to 10（膝盖）,11(踝关节);19,14（左） to 14,13（左）
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_anklebigtoe_kneeankle[2];


	////踝关节与地面平面法线的夹角
	////22（大脚趾）,11(踝关节)(右) ;19,14（左） 
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_anklebigtoe_plane_normal[2];


	////踝关节与地面平面法线的夹角
	////22（大脚趾）,24(踝关节)(右) ;19,21（左） 
	////[0]为身体右面的，[1]为身体左面的
	AngleInfo ang_heel_plane_normal[2];



	AngleInfo ang_plane1_left_right;
	

	AngleInfo ang_plane2_before_after;



public:
	AngleInfo getang_plane1_left_right();
	void setang_plane1_left_right(double _ang_plane1_left_right,int _x,int _y, std::string _desc,sl::float3 _pos=sl::float3(NAN,NAN,NAN));

	AngleInfo getang_plane2_before_after();
	void setang_plane2_before_after(double _ang_plane2_before_after, int _x, int _y, std::string _desc, sl::float3 _pos = sl::float3(NAN, NAN, NAN));


	AngleInfo getang_midhip_plane_normal();
	void setang_midhip_plane_normal(Angle _ang_midhip_plane_normal, int _x, int _y, std::string _desc, sl::float3 _pos = sl::float3(NAN, NAN, NAN));

	AngleInfo getang_hipknee_midhip(Body body = BODY_LEFT);
	void setang_hipknee_midhip(Angle _ang_hipknee_midhip,  int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));



	AngleInfo getang_hipknee_plane_normal(Body body = BODY_LEFT);
	void setang_hipknee_plane_normal(Angle _ang_hipknee_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));

	AngleInfo getang_kneeankle_plane_normal(Body body = BODY_LEFT);
	void setang_kneeankle_plane_normal(Angle _ang_kneeankle_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));


	AngleInfo getang_kneeankle_hipknee(Body body = BODY_LEFT);
	void setang_kneeankle_hipknee(Angle _ang_kneeankle_hipknee, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));




	AngleInfo getang_anklebigtoe_kneeankle(Body body = BODY_LEFT);
	void setang_anklebigtoe_kneeankle(Angle _ang_anklebigtoe_kneeankle, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));
	AngleInfo getang_anklebigtoe_plane_normal(Body body = BODY_LEFT);
	void setang_anklebigtoe_plane_normal(Angle _ang_anklebigtoe_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));


	
	AngleInfo getang_heel_plane_normal(Body body = BODY_LEFT);
	void setang_heel_plane_normal(Angle _ang_heel_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));
};

#endif
