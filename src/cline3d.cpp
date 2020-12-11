#include "cline3d.h"



cline3d::cline3d()
{
}


cline3d::~cline3d()
{
}

void cline3d::setOriginPt(sl::double3 _originpt)
{
	originpt = _originpt;
}

sl::double3 cline3d::getOriginPt()
{
	return originpt;
}

void cline3d::setDirection(sl::double3 _direc)
{
	direction = _direc;
}

sl::double3 cline3d::getDirection()
{
	return direction;
}

void cline3d::setT(double _t)
{
	t = _t;
}

double cline3d::getT()
{
	return t;
}

sl::double3 cline3d::intersectionpt()
{
	sl::double3 intpt = sl::double3(NAN, NAN, NAN);

	intpt = originpt + t * direction;

	return intpt;
}
