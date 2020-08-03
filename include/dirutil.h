
#ifndef DIRUTIL_H
#define DIRUTIL_H
#pragma once

#ifdef WIN32
#include <io.h>
#include <direct.h> 
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <stdint.h>
#include <string>
#define MAX_PATH_LEN 256

#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif



#include <stdio.h>  /* defines FILENAME_MAX */
// #define WINDOWS  /* uncomment this line to use it for windows.*/ 
#ifdef WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
#include<iostream>


class dirutil
{
public:
	dirutil();
	~dirutil();

	static std::string GetCurrentWorkingDir();
	static int32_t createDirectory(const std::string& directoryPath);
};
#endif

