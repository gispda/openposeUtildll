#define DLLEXPORT extern "C" __declspec(dllexport)
#include "zedservice.h"
#include "ostream"
#include "stdio.h"

DLLEXPORT wchar_t* sum(wchar_t *str, int b, float f) 
{   static wchar_t a[] = L"ÄãºÃ,world";
	static wchar_t szBuffer[100];
	float x = b + f;
	swprintf(szBuffer, 255, L"%s,%s,%f", a, str, x);
	return szBuffer;
}


DLLEXPORT char* getposeavifile()
{
	char* pfile = (char*)zedservice::getpose_avi_file().data();	
	/*wchar_t szBuffer[400];
	printf("pose avi in here111111111111");
	swprintf(szBuffer, 402, L"%s", pfile);
	printf("pose avi in here222222222");
	wprintf(szBuffer);*/

	return pfile;
}

DLLEXPORT char* getposedatadir()
{
	//char* pfile = (char*)zedservice::getpose_data_dir().data();
	


	char* pfile = (char*)zedservice::getpose_data_dir().data();
	//wchar_t szBuffer[400];

	//swprintf(szBuffer, 402, L"%s", pfile);
	return pfile;
}


DLLEXPORT char* startposeservice(char* str,bool isshow=false)
{
	std::string svofile(str);

	char* resbuffer = (char*)zedservice::startposeservice(svofile,isshow).data();
	return resbuffer;
}

DLLEXPORT void endposeservice()
{
	zedservice::stopposeservice();
}

DLLEXPORT bool isCreatePoseAvi()
{
	return zedservice::isCreatePoseAvi();
}

DLLEXPORT char* startmergereportavi(char* hmavi_file, char* zedsvo_file, char* zedposeavi_file, char* posedata_file, bool isshow = false)
{
	std::string s1(hmavi_file);
	std::string s2(zedsvo_file);
	std::string s3(zedposeavi_file);
	std::string s4(posedata_file);

	char* pfile = (char*)zedservice::startmergereportavi(s1, s2, s3, s4,isshow).data();

	return pfile;
}

