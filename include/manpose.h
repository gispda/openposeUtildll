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
////���������ɡ��Ա����̬�Ƕȷ�װ�࣬һ�δ���һ�����༴�ɡ�
////
////
////
enum Body {
	BODY_LEFT=1,BODY_RIGHT=0
};

struct Angle {
	double angle;

	int x;
	int y;

	sl::float3 pos;

	std::string desc;
	//std::string  sdesc;
};

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



	void configKeyData();
	int getbody_part_index(string key, SecIdx secidx=FIRST_SEC_START);


private:
	cplane lrplane;
	cplane bbplane;



	/// <summary>
	///map(string(key),string(data index))
	/// string format:
	///ang_midhip_plane_normal
	///key:neck_to_midhip
	///data index:1-8-999-1000    1(����) 8���β��ϼ����ǣ� - �����������β��ϼ����Ǿ��Ǽ���   
	///999-1000�ʹ���ذ�ƽ�淨�߷���             
	///
	/// </summary>
	std::map<string,string> bodypartindexmap;

	////�β��ϼ����������ƽ�淨�ߵļн�
	////1(����),8(�β��ϼ�����)  
	Angle ang_midhip_plane_normal;



	////����������ļн�
	////9(�β�),10��ϥ�ǣ�(��)12,13���� to 1,8
	////[0]Ϊ��������ģ�[1]Ϊ���������
	Angle ang_hipknee_midhip[2];
	

	////���������ƽ�淨�ߵļн�
	////9(�β�),10��ϥ�ǣ�(��)12,13���� 
	////[0]Ϊ��������ģ�[1]Ϊ���������
	Angle ang_hipknee_plane_normal[2];
	

	////С�������ƽ�淨�ߵļн�
	////10��ϥ�ǣ�,11(�׹ؽ�)(��)13,14���� 
	////[0]Ϊ��������ģ�[1]Ϊ���������
	Angle ang_kneeankle_plane_normal[2];


	////С������ȵļн�
	////10��ϥ�ǣ�,11(�׹ؽ�)(��) to 9(�β�),10��ϥ�ǣ�;13,14���� to 12,13����
	////[0]Ϊ��������ģ�[1]Ϊ���������
	Angle ang_kneeankle_hipknee[2];
	
	////�׹ؽ���С�ȵļн�
	////22�����ֺ��,11(�׹ؽ�)(��) to 10��ϥ�ǣ�,11(�׹ؽ�);19,14���� to 14,13����
	////[0]Ϊ��������ģ�[1]Ϊ���������
	Angle ang_anklebigtoe_kneeankle[2];


	////�׹ؽ������ƽ�淨�ߵļн�
	////22�����ֺ��,11(�׹ؽ�)(��) ;19,14���� 
	////[0]Ϊ��������ģ�[1]Ϊ���������
	Angle ang_anklebigtoe_plane_normal[2];




	Angle ang_plane1_left_right;
	

	Angle ang_plane2_before_after;
public:
	Angle getang_plane1_left_right();
	void setang_plane1_left_right(double _ang_plane1_left_right,int _x,int _y, std::string _desc,sl::float3 _pos=sl::float3(NAN,NAN,NAN));

	Angle getang_plane2_before_after();
	void setang_plane2_before_after(double _ang_plane2_before_after, int _x, int _y, std::string _desc, sl::float3 _pos = sl::float3(NAN, NAN, NAN));


	Angle getang_midhip_plane_normal();
	void setang_midhip_plane_normal(double _ang_midhip_plane_normal, int _x, int _y, std::string _desc, sl::float3 _pos = sl::float3(NAN, NAN, NAN));

	Angle getang_hipknee_midhip(Body body = BODY_LEFT);
	void setang_hipknee_midhip(double _ang_hipknee_midhip,  int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));



	Angle getang_hipknee_plane_normal(Body body = BODY_LEFT);
	void setang_hipknee_plane_normal(double _ang_hipknee_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));

	Angle getang_kneeankle_plane_normal(Body body = BODY_LEFT);
	void setang_kneeankle_plane_normal(double _ang_kneeankle_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));


	Angle getang_kneeankle_hipknee(Body body = BODY_LEFT);
	void setang_kneeankle_hipknee(double _ang_kneeankle_hipknee, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));




	Angle getang_anklebigtoe_kneeankle(Body body = BODY_LEFT);
	void setang_anklebigtoe_kneeankle(double _ang_anklebigtoe_kneeankle, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));
	Angle getang_anklebigtoe_plane_normal(Body body = BODY_LEFT);
	void setang_anklebigtoe_plane_normal(double _ang_anklebigtoe_plane_normal, int _x, int _y, std::string _desc, Body body = BODY_LEFT, sl::float3 _pos = sl::float3(NAN, NAN, NAN));
};

#endif
