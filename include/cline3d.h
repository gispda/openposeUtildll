

#ifndef LINE3D_HPP
#define LINE3D_HPP

#pragma once



#include <sl/Camera.hpp>


class cline3d
{
public:
	cline3d();
	~cline3d();

public:
	void setOriginPt(sl::double3 _originpt);
	sl::double3 getOriginPt();

	void setDirection(sl::double3 _direc);
	sl::double3 getDirection();

	void setT(double _t);
	double getT();

	//sl::double3 getNormal();
	sl::double3 intersectionpt();
private:
	sl::double3 originpt;
	sl::double3 direction;
	double t;
};

#endif  //LINE3D_HPP

