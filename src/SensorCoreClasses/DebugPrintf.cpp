/* DebugPrintf.cpp */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/

#include <iostream>
#include <fstream>
#include <string>
#include <stdarg.h>


#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include "SensorCoreClassesGlobal.h"
#include "SensorSettings.h"
#include "DebugPrintf.h"

SENSORCORE_USE_NAMESPACE


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

	if (SensorSettings::GetGlobalSettings()->bWriteDebugFile) 
	{

		// Open the debug file
		std::ofstream debugFile;
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

void DebugFilePrintf(std::ofstream &debugFile, const char *format, ...)
{
	if (SensorSettings::GetGlobalSettings()->bWriteDebugFile) 
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

bool OpenDebugFile(std::ofstream &debugFile, const char *fileName, bool bAppend)
{
	if (SensorSettings::GetGlobalSettings()->bWriteDebugFile) 
	{
		if (debugFile.is_open())
		{
			CloseDebugFile(debugFile);
		}

		std::string sFileName = sFilePath + fileName;

		if (bAppend) 
		{
			debugFile.open(sFileName, std::ofstream::out | std::ofstream::app);
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

bool CloseDebugFile(std::ofstream &debugFile)

{
	if (SensorSettings::GetGlobalSettings()->bWriteDebugFile) 
	{
		if (debugFile.is_open())
			debugFile.close();
	}
	return(true);
}


bool OpenLogFile(std::ofstream &logFile, const char *fileName, bool bAppend)
{
	if (SensorSettings::GetGlobalSettings()->bWriteLogFile)
	{
		if (logFile.is_open())
		{
			CloseLogFile(logFile);
		}

		std::string sFileName = sFilePath + fileName;

		if (bAppend) 
		{
			logFile.open(sFileName, std::ofstream::out | std::ofstream::app);
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

bool CloseLogFile(std::ofstream &logFile)

{
	if (SensorSettings::GetGlobalSettings()->bWriteLogFile)
	{
		if (logFile.is_open())
			logFile.close();
	}

	return(true);
}

void LogFilePrintf(std::ofstream &logFile, const char *format, ...)
{
	if (SensorSettings::GetGlobalSettings()->bWriteLogFile)
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

