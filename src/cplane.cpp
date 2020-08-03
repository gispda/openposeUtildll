#include "cplane.h"



cplane::cplane()
{
}


cplane::~cplane()
{
}

void cplane::setplanefromthreepoint(sl::double3 p1, sl::double3 p2, sl::double3 p3)
{

	double a, b, c, d;
	a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));

	b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));

	c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));

	d = (0 - (a * p1.x + b * p1.y + c * p1.z));

	setequation(a, b, c, d);
}

sl::double4 cplane::getequation()
{
	return equation;
}

double cplane::a()
{
	return equation.v[0];
}

double cplane::b()
{
	return equation.v[1];
}

double cplane::c()
{
	return equation.v[2];
}

double cplane::d()
{
	return equation.v[3];
}

double cplane::pointistoponplane(sl::double3 pt0)
{


	double nsdot = 0;
	//bool bret = false;

	//rt = a() * pt0.x + b() * pt0.y + c() * pt0.z + d();	
	sl::double3 ns = getNormal();
	nsdot = dot(ns-intpt, pt0);

	return nsdot;
}

void cplane::setequation(double a, double b, double c, double d)
{

	equation.v[0] = a;
	equation.v[1] = b;
	equation.v[2] = c;
	equation.v[3] = d;
}

double cplane::dot(sl::double3 vec1, sl::double3 vec2)
{
	return vec1.x*vec2.x+vec1.y*vec2.y+vec1.z*vec2.z;
}

sl::double3 cplane::getNormal()
{

	sl::double3 ns = sl::double3(NAN, NAN, NAN);
	ns.x = a();
	ns.y = b();
	ns.z = c();

	return ns;
}



bool cplane::LineIntersectPlane(cline3d line)
{

	double denominator = 0;
	double t = 0;
	sl::double3 interpt = sl::double3(NAN,NAN,NAN);
	sl::double3 ns = getNormal();
	denominator = dot(line.getDirection(), ns);

	if (abs(denominator) < epsilon)
	{
		if (abs(line.getOriginPt().x * a() + line.getOriginPt().y * b() + line.getOriginPt().z * c() + d())<epsilon)
		{
			t = 0;
			line.setT(t);
			return true;
		}
		else
		{
			return false;
		}
	}

	t = -(a()*line.getOriginPt().x+ b() * line.getOriginPt().y+ c() * line.getOriginPt().z+d());
	t = t / denominator;

	line.setT(t);
				
	intpt = line.intersectionpt();
	
	
	return true;
}

sl::double3 cplane::getintpt()
{
	return intpt;
}
