/* DebugPrintf.cpp */
/*
	Copyright 2014, 2015 Phantom Intelligence Inc.

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <stdarg.h>


#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include "AWLSettings.h"
#include "DebugPrintf.h"

using namespace std;
using namespace awl;

namespace awl
{

// Default debug file name
const char *sDebugFileName = "debug.dbg";

// Path for the debug and log files.  
// Per default is empty string, which amounts to writing to the application directory
std::string sFilePath("");

std::string sLogFileName("distanceLog.csv");

// Field separator used in debug files.
const char cFieldSeparator = ';'; 

#ifdef _DEBUG
const bool bIsInDebug = true;
#else
const bool bIsInDebug = false;
#endif

const int maxStrLen = 1024;

void DebugFilePrintf(const char *format, ...)
{

	if (AWLSettings::GetGlobalSettings()->bWriteDebugFile) 
	{

		// Open the debug file
		ofstream debugFile;
		bool bOpen = OpenDebugFile(debugFile, sDebugFileName, true); 
		if( !bOpen) return;


		// Format the string and output

		boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());

		std::string timeStr(boost::posix_time::to_simple_string(myTime));
		timeStr += " ";

		va_list argList;
		va_start(argList, format);
		size_t len = std::vsnprintf(NULL, 0, format, argList);
		va_end(argList);

		std::vector<char> vec(len + 1);
		va_start(argList, format);
		std::vsnprintf(&vec[0], len + 1, format, argList);
		va_end(argList);


		timeStr += std::string(vec.data());
		timeStr += "\n";

		debugFile << timeStr;
		// Close the debug file
		CloseDebugFile(debugFile);

	}
}

void DebugFilePrintf(ofstream &debugFile, const char *format, ...)
{
	if (AWLSettings::GetGlobalSettings()->bWriteDebugFile) 
	{
		if (!debugFile.is_open()) return;

		boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());

		std::string timeStr(boost::posix_time::to_simple_string(myTime));
		timeStr[11]=cFieldSeparator;
		timeStr += cFieldSeparator;

		va_list argList;
		va_start(argList, format);
		size_t len = std::vsnprintf(NULL, 0, format, argList);
		va_end(argList);

		std::vector<char> vec(len + 1);
		va_start(argList, format);
		std::vsnprintf(&vec[0], len + 1, format, argList);
		va_end(argList);


		timeStr += std::string(vec.data());
		timeStr += "\n";

		debugFile << timeStr;
		debugFile.flush();
	}
}

bool OpenDebugFile(ofstream &debugFile, const char *fileName, bool bAppend)
{
	if (AWLSettings::GetGlobalSettings()->bWriteDebugFile) 
	{
		if (debugFile.is_open())
		{
			CloseDebugFile(debugFile);
		}

		std::string sFileName = sFilePath + fileName;

		if (bAppend) 
		{
			debugFile.open(sFileName, ofstream::out | ofstream::app);
		}
		else 
		{
			debugFile.open(sFileName);
		}

		if ( debugFile.fail() ) 
		{
			std::string sErr = " Failed to open ";
			sErr += fileName;
			fprintf(stderr, sErr.c_str());

			return (false);
		}
	}
	return(true);
}

bool CloseDebugFile(ofstream &debugFile)

{
	if (AWLSettings::GetGlobalSettings()->bWriteDebugFile) 
	{
		if (debugFile.is_open())
			debugFile.close();
	}
	return(true);
}


bool OpenLogFile(ofstream &logFile, const char *fileName, bool bAppend)
{
	if (AWLSettings::GetGlobalSettings()->bWriteLogFile)
	{
		if (logFile.is_open())
		{
			CloseLogFile(logFile);
		}

		std::string sFileName = sFilePath + fileName;

		if (bAppend) 
		{
			logFile.open(sFileName, ofstream::out | ofstream::app);
		}
		else 
		{
			logFile.open(sFileName);
		}

		if ( logFile.fail() ) 
		{
			std::string sErr = " Failed to open ";
			sErr += fileName;
			fprintf(stderr, sErr.c_str());

			return (false);
		}
	}

	return(true);
}

bool CloseLogFile(ofstream &logFile)

{
	if (AWLSettings::GetGlobalSettings()->bWriteLogFile)
	{
		if (logFile.is_open())
			logFile.close();
	}

	return(true);
}

void LogFilePrintf(ofstream &logFile, const char *format, ...)
{
	if (AWLSettings::GetGlobalSettings()->bWriteLogFile)
	{
		if (!logFile.is_open()) return;

		boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());

		std::string timeStr(boost::posix_time::to_simple_string(myTime));
		timeStr[11]=cFieldSeparator;
		timeStr += cFieldSeparator;

		va_list argList; 
		va_start(argList, format);
		size_t len = std::vsnprintf(NULL, 0, format, argList);
		va_end(argList);

		std::vector<char> vec(len + 1);
		va_start(argList, format);
		std::vsnprintf(&vec[0], len + 1, format, argList);
		va_end(argList);


		timeStr += std::string(vec.data());
		timeStr += "\n";

		logFile << timeStr;
	}
}

bool SetLogAndDebugFilePath(std::string newFilePath)
{
	sFilePath = newFilePath;
	return(true);
}

std::string GetLogAndDebugFilePath()
{
	return(sFilePath);
}

bool SetLogFileName(std::string newFileName)
{
	sLogFileName = newFileName;
	return(true);
}

std::string GetLogFileName()
{
	return(sLogFileName);
}



}  // namespace awl
