
#include <iostream>
#include <fstream>
#include <string>
#include <stdarg.h>

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include "DebugPrintf.h"

using namespace std;
using namespace awl;

namespace awl
{

const char *sDebugFileName = "debug.dbg";
void DebugFilePrintf(const char *format, ...)
{
#ifdef _DEBUG

	// Open the debuug file
	ofstream debugFile;
	bool bOpen = OpenDebugFile(debugFile, sDebugFileName, true); 
	if( !bOpen) return;


	// Format the string and output
	char str[255];

	boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());

	std::string timeStr(boost::posix_time::to_simple_string(myTime));
	timeStr += " ";

	va_list argList;
	va_start (argList, format);
    vsprintf(str, format, argList);
	va_end(argList);
	
	timeStr += str;
	timeStr += "\n";

	debugFile << timeStr;
	// Close the debug file
	CloseDebugFile(debugFile);

#endif
}

void DebugFilePrintf(ofstream &debugFile, const char *format, ...)
{
#ifdef _DEBUG
	if (!debugFile.is_open()) return;
	char str[255];

	boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());

	std::string timeStr(boost::posix_time::to_simple_string(myTime));
	timeStr += " ";

	va_list argList;
	va_start (argList, format);
    vsprintf(str, format, argList);
	va_end(argList);
	
	timeStr += str;
	timeStr += "\n";

	debugFile << timeStr;
#endif
}

bool OpenDebugFile(ofstream &debugFile, const char *fileName, bool bAppend)
{
#ifdef _DEBUG
	if (debugFile.is_open())
	{
		CloseDebugFile(debugFile);
	}

	if (bAppend) 
	{
		debugFile.open(fileName, ofstream::out | ofstream::app);
	}
	else 
	{
		debugFile.open(fileName);
	}

	if ( debugFile.fail() ) 
	{
		std::string sErr = " Failed to open ";
		sErr += fileName;
		fprintf(stderr, sErr.c_str());

		return (false);
	}
#endif
	return(true);
}

bool CloseDebugFile(ofstream &debugFile)

{
#ifdef _DEBUG
	if (debugFile.is_open())
		debugFile.close();
#endif
	return(true);
}

}  // namespace awl
