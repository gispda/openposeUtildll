#include "manpose.h"
#include "utils.h"


manpose::manpose()
{

	//bodypartindexmap = nullptr;

	twokneev = 0;
	twoheelv = 0;
	frameidx = -1;
}


manpose::~manpose()
{

	if (!bodypartindexmap.empty())
	{
		bodypartindexmap.clear();
	}

	if (!bodyjointposmap.empty())
	{
		bodyjointposmap.clear();
	}
	if (!jointsposmap2d.empty())
	{
		jointsposmap2d.clear();
	}
	


}

void manpose::init()
{

	twokneev = 0;
	twoheelv = 0;
	frameidx = -1;
	

	//ang_midhip_plane_normal.angle = NAN;
	ang_anklebigtoe_kneeankle[0].init();
	ang_anklebigtoe_kneeankle[1].init();

	this->ang_anklebigtoe_plane_normal[0].init();
	this->ang_anklebigtoe_plane_normal[1].init();


	this->ang_heel_plane_normal[0].init();
	this->ang_heel_plane_normal[1].init();

	this->ang_hipknee_midhip[0].init();
	this->ang_hipknee_midhip[1].init();

	this->ang_hipknee_plane_normal[0].init();
	this->ang_hipknee_plane_normal[1].init();

	this->ang_kneeankle_hipknee[0].init();
	this->ang_kneeankle_hipknee[1].init();

	this->ang_kneeankle_plane_normal[0].init();
	this->ang_kneeankle_plane_normal[1].init();


	this->ang_midhip_plane_normal.init();

	this->ang_plane1_left_right.init();

	this->ang_plane2_before_after.init();
	
}

void manpose::configKeyData()
{
	if (!bodypartindexmap.empty())
	{
		return;
	}

	//qDebug() << "------------------------------configKeyData";

	bodypartindexmap.insert(make_pair(string("none_neckmidhip_plane"), string("1-8-999-1000")));

	bodypartindexmap.insert(make_pair(string("right_hipknee_to_midhip"), string("1-8-9-10")));
	bodypartindexmap.insert(make_pair(string("left_hipknee_to_midhip"), string("1-8-12-13")));

	bodypartindexmap.insert(make_pair(string("right_hipknee_plane"), string("9-10-999-1000")));
	bodypartindexmap.insert(make_pair(string("left_hipknee_plane"), string("12-13-999-1000")));

	bodypartindexmap.insert(make_pair(string("right_kneeankle_plane"), string("10-11-999-1000")));
	bodypartindexmap.insert(make_pair(string("left_kneeankle_plane"), string("13-14-999-1000")));

	bodypartindexmap.insert(make_pair(string("right_kneeankle_hipknee"), string("10-11-9-10")));
	bodypartindexmap.insert(make_pair(string("left_kneeankle_hipknee"), string("13-14-12-13")));


	
	bodypartindexmap.insert(make_pair(string("right_anklebigtoe_kneeankle"), string("11-22-10-11")));
	bodypartindexmap.insert(make_pair(string("left_anklebigtoe_kneeankle"), string("14-19-13-14")));

	bodypartindexmap.insert(make_pair(string("right_anklebigtoe_plane"), string("11-22-999-1000")));
	bodypartindexmap.insert(make_pair(string("left_anklebigtoe_plane"), string("14-19-999-1000")));


	bodypartindexmap.insert(make_pair(string("right_heel_plane"), string("22-24-999-1000")));
	bodypartindexmap.insert(make_pair(string("left_heel_plane"), string("19-21-999-1000")));
	//qDebug() << "end------------------------------configKeyData";
}

int manpose::getbody_part_index(string key, SecIdx secidx)
{
	int ret = 0;
	string value = bodypartindexmap[key];
	std::vector<std::string> elems;
	string strvalue;
	elems=Utils::splitData(value, '-');
	switch (secidx)
	{
	case(FIRST_SEC_START):
		strvalue = elems.at(0);
		break;

	case(FIRST_SEC_END):
		strvalue = elems.at(1);
		break;
	case(SECOND_SEC_START):
		strvalue = elems.at(2);
		break;

	case(SECOND_SEC_END):
		strvalue = elems.at(3);
		break;
	default:		break;

	}
	ret = stoi(strvalue);

	return ret;
}

AngleInfo manpose::getang_plane1_left_right()
{
	return ang_plane1_left_right;
}

void manpose::setang_plane1_left_right(double _ang_plane1_left_right, int _x, int _y, std::string _desc, sl::float3 _pos)
{
	ang_plane2_before_after.angle.angle = _ang_plane1_left_right;
	ang_plane2_before_after.x = _x;
	ang_plane2_before_after.y = _y;
	ang_plane2_before_after.desc = _desc;
	ang_plane2_before_after.pos = _pos;
	ang_plane2_before_after.descxoy = " 平面角度";
	ang_plane2_before_after.descyoz = " 矢角度";

}

AngleInfo manpose::getang_plane2_before_after()
{
	return ang_plane2_before_after;
}

void manpose::setang_plane2_before_after(double _ang_plane2_before_after, int _x, int _y, std::string _desc,  sl::float3 _pos )
{
	ang_plane2_before_after.angle.angle = _ang_plane2_before_after;
	ang_plane2_before_after.x = _x;
	ang_plane2_before_after.y = _y;
	ang_plane2_before_after.desc = _desc;
	ang_plane2_before_after.pos = _pos;
	ang_plane2_before_after.descxoy = " 平面角度";
	ang_plane2_before_after.descyoz = " 矢角度";

}

AngleInfo manpose::getang_midhip_plane_normal()
{
	return ang_midhip_plane_normal;
}

void manpose::setang_midhip_plane_normal(Angle _ang_midhip_plane_normal,int _x, int _y, std::string _desc, sl::float3 _pos )
{
	ang_midhip_plane_normal.angle = _ang_midhip_plane_normal;
	ang_midhip_plane_normal.x = _x;
	ang_midhip_plane_normal.y = _y;
	ang_midhip_plane_normal.desc = _desc;
	ang_midhip_plane_normal.pos = _pos;
	ang_midhip_plane_normal.descxoy = " 平面角度";
	ang_midhip_plane_normal.descyoz = " 矢角度";

}

AngleInfo manpose::getang_hipknee_midhip(Body body)
{
	if(body == BODY_LEFT)
	return ang_hipknee_midhip[0];
	else if(body == BODY_RIGHT)
	return ang_hipknee_midhip[1];
}

void manpose::setang_hipknee_midhip(Angle _ang_hipknee_midhip, int _x, int _y, std::string _desc, Body body, sl::float3 _pos)
{

	if (body == BODY_LEFT)
	{
		ang_hipknee_midhip[0].angle = _ang_hipknee_midhip;
		ang_hipknee_midhip[0].x = _x;
		ang_hipknee_midhip[0].y = _y;
		ang_hipknee_midhip[0].desc = _desc;
		ang_hipknee_midhip[0].pos = _pos;
		ang_hipknee_midhip[0].descxoy = " 平面角度";
		ang_hipknee_midhip[0].descyoz = " 矢角度";
	}
	else if (body == BODY_RIGHT)
	{
		ang_hipknee_midhip[1].angle = _ang_hipknee_midhip;
		ang_hipknee_midhip[1].x = _x;
		ang_hipknee_midhip[1].y = _y;
		ang_hipknee_midhip[1].desc = _desc;
		ang_hipknee_midhip[1].pos = _pos;
		ang_hipknee_midhip[1].descxoy = " 平面角度";
		ang_hipknee_midhip[1].descyoz = " 矢角度";
	}
}

AngleInfo manpose::getang_hipknee_plane_normal(Body body)
{
	if (body == BODY_LEFT)
		return ang_hipknee_plane_normal[0];
	else if (body == BODY_RIGHT)
		return ang_hipknee_plane_normal[1];
}

void manpose::setang_hipknee_plane_normal(Angle _ang_hipknee_plane_normal, int _x, int _y, std::string _desc, Body body, sl::float3 _pos )
{

	if (body == BODY_LEFT)
	{
		ang_hipknee_plane_normal[0].angle = _ang_hipknee_plane_normal;
		ang_hipknee_plane_normal[0].x = _x;
		ang_hipknee_plane_normal[0].y = _y;
		ang_hipknee_plane_normal[0].desc = _desc;
		ang_hipknee_plane_normal[0].pos = _pos;
		ang_hipknee_plane_normal[0].descxoy = " 平面角度";
		ang_hipknee_plane_normal[0].descyoz = " 矢角度";

	}
	else if (body == BODY_RIGHT)
	{
		ang_hipknee_plane_normal[1].angle = _ang_hipknee_plane_normal;
		ang_hipknee_plane_normal[1].x = _x;
		ang_hipknee_plane_normal[1].y = _y;
		ang_hipknee_plane_normal[1].desc = _desc;
		ang_hipknee_plane_normal[1].pos = _pos;
		ang_hipknee_plane_normal[1].descxoy = " 平面角度";
		ang_hipknee_plane_normal[1].descyoz = " 矢角度";
	}
}

AngleInfo manpose::getang_kneeankle_plane_normal(Body body)
{
	if (body == BODY_LEFT)
		return ang_kneeankle_plane_normal[0];
	else if (body == BODY_RIGHT)
		return ang_kneeankle_plane_normal[1];
}

void manpose::setang_kneeankle_plane_normal(Angle _ang_kneeankle_plane_normal, int _x, int _y, std::string _desc, Body body, sl::float3 _pos)
{

	if (body == BODY_LEFT)
	{
		ang_kneeankle_plane_normal[0].angle = _ang_kneeankle_plane_normal;
		ang_kneeankle_plane_normal[0].x = _x;
		ang_kneeankle_plane_normal[0].y = _y;
		ang_kneeankle_plane_normal[0].desc = _desc;
		ang_kneeankle_plane_normal[0].pos = _pos;
		ang_kneeankle_plane_normal[0].descxoy = " 平面角度";
		ang_kneeankle_plane_normal[0].descyoz = " 矢角度";
	}
	else if (body == BODY_RIGHT)
	{
		ang_kneeankle_plane_normal[1].angle = _ang_kneeankle_plane_normal;
		ang_kneeankle_plane_normal[1].x = _x;
		ang_kneeankle_plane_normal[1].y = _y;
		ang_kneeankle_plane_normal[1].desc = _desc;
		ang_kneeankle_plane_normal[1].pos = _pos;
		ang_kneeankle_plane_normal[1].descxoy = " 平面角度";
		ang_kneeankle_plane_normal[1].descyoz = " 矢角度";
	}
}

AngleInfo manpose::getang_kneeankle_hipknee(Body body)
{
	if (body == BODY_LEFT)
		return ang_kneeankle_hipknee[0];
	else if (body == BODY_RIGHT)
		return ang_kneeankle_hipknee[1];
}

void manpose::setang_kneeankle_hipknee(Angle _ang_kneeankle_hipknee, int _x, int _y, std::string _desc, Body body, sl::float3 _pos )
{

	if (body == BODY_LEFT)
	{
		ang_kneeankle_hipknee[0].angle = _ang_kneeankle_hipknee;
		ang_kneeankle_hipknee[0].x = _x;
		ang_kneeankle_hipknee[0].y = _y;
		ang_kneeankle_hipknee[0].desc = _desc;
		ang_kneeankle_hipknee[0].pos = _pos;

		ang_kneeankle_hipknee[0].descxoy = " 平面角度";
		ang_kneeankle_hipknee[0].descyoz = " 矢角度";
	}
	else if (body == BODY_RIGHT)
	{
		ang_kneeankle_hipknee[1].angle = _ang_kneeankle_hipknee;
		ang_kneeankle_hipknee[1].x = _x;
		ang_kneeankle_hipknee[1].y = _y;
		ang_kneeankle_hipknee[1].desc = _desc;
		ang_kneeankle_hipknee[1].pos = _pos;
		ang_kneeankle_hipknee[1].descxoy = " 平面角度";
		ang_kneeankle_hipknee[1].descyoz = " 矢角度";

	}
}

AngleInfo manpose::getang_anklebigtoe_kneeankle(Body body)
{
	if (body == BODY_LEFT)
		return ang_anklebigtoe_kneeankle[0];
	else if (body == BODY_RIGHT)
		return ang_anklebigtoe_kneeankle[1];
}

void manpose::setang_anklebigtoe_kneeankle(Angle _ang_anklebigtoe_kneeankle, int _x, int _y, std::string _desc, Body body, sl::float3 _pos)
{

	if (body == BODY_LEFT)
	{
		ang_anklebigtoe_kneeankle[0].angle = _ang_anklebigtoe_kneeankle;
		ang_anklebigtoe_kneeankle[0].x = _x;
		ang_anklebigtoe_kneeankle[0].y = _y;
		ang_anklebigtoe_kneeankle[0].desc = _desc;
		ang_anklebigtoe_kneeankle[0].pos = _pos;

		ang_anklebigtoe_kneeankle[0].descxoy = " 平面角度";
		ang_anklebigtoe_kneeankle[0].descyoz = " 矢角度";
	}
	else if (body == BODY_RIGHT)
	{
		ang_anklebigtoe_kneeankle[1].angle = _ang_anklebigtoe_kneeankle;
		ang_anklebigtoe_kneeankle[1].x = _x;
		ang_anklebigtoe_kneeankle[1].y = _y;
		ang_anklebigtoe_kneeankle[1].desc = _desc;
		ang_anklebigtoe_kneeankle[1].pos = _pos;

		ang_anklebigtoe_kneeankle[1].descxoy = " 平面角度";
		ang_anklebigtoe_kneeankle[1].descyoz = " 矢角度";
	}
}

AngleInfo manpose::getang_anklebigtoe_plane_normal(Body body)
{
	if (body == BODY_LEFT)
		return ang_anklebigtoe_plane_normal[0];
	else if (body == BODY_RIGHT)
		return ang_anklebigtoe_plane_normal[1];
}

void manpose::setang_anklebigtoe_plane_normal(Angle _ang_anklebigtoe_plane_normal, int _x, int _y, std::string _desc, Body body, sl::float3 _pos )
{

	if (body == BODY_LEFT)
	{
		ang_anklebigtoe_plane_normal[0].angle = _ang_anklebigtoe_plane_normal;
		ang_anklebigtoe_plane_normal[0].x = _x;
		ang_anklebigtoe_plane_normal[0].y = _y;
		ang_anklebigtoe_plane_normal[0].desc = _desc;
		ang_anklebigtoe_plane_normal[0].pos = _pos;

		ang_anklebigtoe_plane_normal[0].descxoy = " 平面角度";
		ang_anklebigtoe_plane_normal[0].descyoz = " 矢角度";
	}
	else if (body == BODY_RIGHT)
	{
		ang_anklebigtoe_plane_normal[1].angle = _ang_anklebigtoe_plane_normal;
		ang_anklebigtoe_plane_normal[1].x = _x;
		ang_anklebigtoe_plane_normal[1].y = _y;
		ang_anklebigtoe_plane_normal[1].desc = _desc;
		ang_anklebigtoe_plane_normal[1].pos = _pos;

		ang_anklebigtoe_plane_normal[1].descxoy = " 平面角度";
		ang_anklebigtoe_plane_normal[1].descyoz = " 矢角度";
	}
}

AngleInfo manpose::getang_heel_plane_normal(Body body)
{
	if (body == BODY_LEFT)
		return ang_heel_plane_normal[0];
	else if (body == BODY_RIGHT)
		return ang_heel_plane_normal[1];
}

void manpose::setang_heel_plane_normal(Angle _ang_heel_plane_normal, int _x, int _y, std::string _desc, Body body, sl::float3 _pos)
{
	if (body == BODY_LEFT)
	{
		ang_heel_plane_normal[0].angle = _ang_heel_plane_normal;
		ang_heel_plane_normal[0].x = _x;
		ang_heel_plane_normal[0].y = _y;
		ang_heel_plane_normal[0].desc = _desc;
		ang_heel_plane_normal[0].pos = _pos;

		ang_heel_plane_normal[0].descxoy = " 平面角度";
		ang_heel_plane_normal[0].descyoz = " 矢角度";
	}
	else if (body == BODY_RIGHT)
	{
		ang_heel_plane_normal[1].angle = _ang_heel_plane_normal;
		ang_heel_plane_normal[1].x = _x;
		ang_heel_plane_normal[1].y = _y;
		ang_heel_plane_normal[1].desc = _desc;
		ang_heel_plane_normal[1].pos = _pos;

		ang_heel_plane_normal[1].descxoy = " 平面角度";
		ang_heel_plane_normal[1].descyoz = " 矢角度";
	}
}

void AngleInfo::init()
{
	angle.angle = NAN;
	angle.anglexoy = NAN;
	angle.angleyoz = NAN;


	desc = "";
	pos = sl::float3{NAN,NAN,NAN};
	descxoy = "";
	descyoz = "";

}
