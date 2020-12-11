#ifndef PLANE_HPP
#define PLANE_HPP

#pragma once


#include "cline3d.h"
#include <sl/Camera.hpp>


class cplane
{
public:
	cplane();


	~cplane();

public:
	void setplanefromthreepoint(sl::double3 pt1, sl::double3 pt2, sl::double3 pt3);
	sl::double4 getequation();

	double a();
	double b();
	double c();
	double d();


	double pointistoponplane(sl::double3 pt0);
	void setequation(double a, double b, double c, double d);
	double static dot(sl::double3 vec1, sl::double3 vec2);

	sl::double3 getNormal();
	bool LineIntersectPlane(cline3d line);
	sl::double3 getintpt();


private:
	double const epsilon = 1e-10;
private:
	sl::double4 equation;
	sl::double3 intpt;
};

#endif  //PLANE_HPP