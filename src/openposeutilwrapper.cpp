#define DLLEXPORT extern "C" __declspec(dllexport)
#include "zedservice.h"

#include "stdio.h"

DLLEXPORT wchar_t* sum(wchar_t *str, int b, float f) 
{   static wchar_t a[] = L"ÄãºÃ,world";
	static wchar_t szBuffer[100];
	float x = b + f;
	swprintf(szBuffer, 255, L"%s,%s,%f", a, str, x);
	return szBuffer;
}


DLLEXPORT void startposeservice(char* str)
{
	std::string svofile(str);
	zedservice::startposeservice(svofile);
	
}

DLLEXPORT void endposeservice()
{
	zedservice::stopposeservice();
}

DLLEXPORT void startmergereportavi(char* hmavi_file, char* zedsvo_file, char* zedposeavi_file, char* posedata_file)
{
	std::string s1(hmavi_file);
	std::string s2(zedsvo_file);
	std::string s3(zedposeavi_file);
	std::string s4(posedata_file);
	zedservice::startmergereportavi(s1, s2, s3, s4);
}

