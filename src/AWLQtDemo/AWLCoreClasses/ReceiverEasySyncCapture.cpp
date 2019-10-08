/* ReceiverEasyCapture.cpp */
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

#include <string>
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#include "DebugPrintf.h"
#include "BlockingReader.h"

#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"
#include "ReceiverEasySyncCapture.h"


using namespace std;
using namespace awl;

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects


const long		    defaultSerialPortRate = 921600;  // Default PC Serial Port rate for EasySync CAN adapter.

ReceiverEasySyncCapture::ReceiverEasySyncCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const std::string &inSerialPort, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers) :
ReceiverCANCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, 
                   inCANBitRate, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, 
				   inParametersAlgos, inParametersTrackers),
port(NULL),
reader(NULL),
io(),
responseString(""),
closeCANReentryCount(0)

{
	// Update settings from application
	sCommPort = inSerialPort;


	serialPortRate = defaultSerialPortRate;

	ConvertEasySyncCANBitRateCode();
}


ReceiverEasySyncCapture::ReceiverEasySyncCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
port(NULL),
reader(NULL),
io(),
responseString(""),
closeCANReentryCount(0)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);

	// Default values that are not in the configuration file anymore
	serialPortRate = defaultSerialPortRate;
	ConvertEasySyncCANBitRateCode();
}

ReceiverEasySyncCapture::~ReceiverEasySyncCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}
bool  ReceiverEasySyncCapture::OpenCANPort()

{
	reader = NULL;

	if (port) 
	{
		CloseCANPort();
	}

	// initialize the CAN - Serial - USB port
	try
	{
	port = new boost::asio::serial_port(io, sCommPort);
    port->set_option(boost::asio::serial_port_base::baud_rate(serialPortRate));
	}
	catch (...)
	{
		port = NULL;
		reader = NULL;

		std::string sErr = " Cannot open serial port ";
		sErr += sCommPort;
		fprintf(stderr, sErr.c_str());
		
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}

    // A blocking reader for this port that 
    // will time out a read after 500 milliseconds.
	reader = new blocking_reader(*port, receiveTimeOutInMillisec);
	port->set_option(boost::asio::serial_port_base::baud_rate(serialPortRate));

	// Send the initialization strings
	WriteString(sCANBitRateCode+"\r"); // Set CAN Rate ("S2"->50Kbps, "S3"->100Kbps, "S8"->1Gbps)
	WriteString("O\r");  // Open
	WriteString("Z0\r");  // Make sure no timestamps are attached
	WriteString("E\r");  // Flush/ Resync
	port->set_option(boost::asio::serial_port_base::baud_rate(serialPortRate));

	if (reader) 
		return(true);
	else {

		delete(port);
		port = NULL;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}

	bFrameInvalidated = false;
}

bool  ReceiverEasySyncCapture::CloseCANPort()

{
	if (closeCANReentryCount > 0) return(false);

	closeCANReentryCount++;
		try 
		{
			WriteString("C\n"); // Close port
		}
		catch (...)
		{
			DebugFilePrintf(debugFile, "Error during Write On Close");
		}


		if (port  && port->is_open()) port->close();
		reader = NULL;
		port = NULL;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
		return(true);
}


void ReceiverEasySyncCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
		AWLCANMessage msg;

		if (port && reader && port->is_open()) 
		{
			char c;
			// read from the serial port until we get a
			// carriage return or until a read times-out (500ms)

		if (reader->read_char(c)) 
			{
				responseString.push_back(c);
				if (c == '\r') 
				{
					if (ParseLine(responseString, msg))
					{
						ParseMessage(msg);
					}
					responseString.clear();
				}
				else if (c == 0x07) // BELL 
				{
					DebugFilePrintf(debugFile, "CanLine error: %s\n", responseString.c_str());
					InvalidateFrame();
					responseString.clear();
				}

			}
			// read_char returned false.  
			// This means we have a time out on the port after receiveTimeOutInMillisec.
			// Try to repoen the port
			else 
			{
				DebugFilePrintf(debugFile,  "Time Outon read_char.  Resetting CAN Port"); 
				CloseCANPort();
				if (OpenCANPort())
				{
					WriteCurrentDateTime();
					SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
					// Update all the info (eventually) from the status of the machine
					QueryAlgorithm();
					QueryTracker();
				}
			}

		} // if (port->is_open)
		else 
		{
			// Port is not opened.  Try to repoen after a certain delay.
			if (boost::posix_time::microsec_clock::local_time() > reconnectTime)
			{
				DebugFilePrintf(debugFile,  "Reconnecting CAN Port"); 
				if (OpenCANPort())
				{
					WriteCurrentDateTime();
					SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
					QueryAlgorithm();
					QueryTracker();
				}
			}

		}

	} // if  (!WasStoppped)
}

bool ReceiverEasySyncCapture::GetDataByte(std::string &inResponse, uint8_t &outByte, int startIndex, int len)

{
	if (len > 2) len = 2;

	outByte = 0;
	int shiftVal = (len-1) * 4;
	for (int i = 0; i < len; i++, shiftVal -= 4) 
	{
		char theChar = inResponse[startIndex+i];
		if (theChar >= '0' && theChar <= '9')
		{
			outByte = outByte + ((theChar - '0') << shiftVal);
		}
		else if (theChar >= 'A' && theChar <= 'F')
		{
			outByte = outByte + ((10 + (theChar - 'A')) << shiftVal);
		}
		else if (theChar >= 'a' && theChar <= 'f')
		{
			outByte = outByte + ((10 + (theChar - 'A')) << shiftVal);
		}
		else 
		{
			DebugFilePrintf(debugFile, "CanLine error - Invalid char: %s-%c", inResponse.c_str(), theChar);
			InvalidateFrame();
			return(false);
		}
	}

	return(true);
}

bool ReceiverEasySyncCapture::GetStandardID(std::string &inResponse, uint32_t &outID, int startIndex)

{
	uint8_t highByte;
	uint8_t lowByte;

	if (!GetDataByte(inResponse, highByte, startIndex, 1)) 
	{
		return(false);
	}

	if (!GetDataByte(inResponse, lowByte, startIndex + 1, 2)) 
	{
		return(false);
	}

	outID = (highByte * (uint32_t)256) + lowByte;

	return(true);
}

bool ReceiverEasySyncCapture::ParseLine(std::string inResponse, AWLCANMessage &outMsg)
{
	bool bResult = false;
	if (inResponse.length() < 2) 
	{
		DebugFilePrintf(debugFile, "CanLine empty %s", inResponse.c_str());
		InvalidateFrame();
		return bResult;
	}

	if (inResponse[0] == 'z')
	{
		DebugFilePrintf(debugFile, "CanLine ack: %s\n", inResponse.c_str());
		return (bResult);
	}
	else if (inResponse[0] != 't') 
	{
		DebugFilePrintf(debugFile, "CanLine bad: %s\n", inResponse.c_str());
		InvalidateFrame();
		return (bResult);
	}

	
	if (inResponse.length() < 6) 
	{
		DebugFilePrintf(debugFile, "CanFrame incomplete %s",  inResponse.c_str());
		InvalidateFrame();
		return (bResult);
	}

	if (!GetStandardID(inResponse, outMsg.id, 1))
	{
		InvalidateFrame();
		return(bResult);
	}

	if (!GetDataByte(inResponse, outMsg.len, 4, 1)) 
	{
		InvalidateFrame();
		return(bResult);
	}

	if (inResponse.length() < (uint16_t) (6 + outMsg.len))
	{
		InvalidateFrame();
		return(bResult);
	}

	if (outMsg.len > 8) 
	{
		InvalidateFrame();
		return(bResult);
	}

	for (int i = 0; i < outMsg.len; i++) 
	{
		if (!GetDataByte(inResponse, outMsg.data[i], 5+ (i*2), 2))
		{
		InvalidateFrame();
		return bResult;
		}
	}

	bResult = true;
	return(bResult);
}



void ReceiverEasySyncCapture::WriteString(std::string inString)
{
	if (!port) return;

	int stringSize = inString.size();

DebugFilePrintf(debugFile, "Out %d bytes - %s", stringSize, inString.c_str());

	std::size_t written = boost::asio::write(*port,boost::asio::buffer(inString.c_str(),stringSize));
DebugFilePrintf(debugFile, "Out %d bytes confirmed", written);
	// Messages must be at leat 1ms apart.
	boost::this_thread::sleep(boost::posix_time::milliseconds(10));
}

bool ReceiverEasySyncCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	std::string outResponse;
	char outString[10];

	if (!port) return(false);

	outResponse += "t";

	sprintf_s(outString, 10, "%01x%02x", (inMsg.id / 256),  inMsg.id & 0xFF);
	outResponse +=outString;

	sprintf_s(outString, 10, "%01x", inMsg.len);
	outResponse +=outString;


	for (int i = 0; i < inMsg.len; i++) 
	{
		sprintf_s(outString, 10 , "%02x", inMsg.data[i]);
		outResponse +=outString;
	}

	outResponse += "\r";

	WriteString(outResponse);
	return(true);
}



bool ReceiverEasySyncCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		ReceiverCANCapture::ReadConfigFromPropTree(propTree);

		std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverID);

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
		// Communication parameters
		sCommPort =  receiverNode.get<std::string>("commPort");

		return(true);
}

void ReceiverEasySyncCapture::ConvertEasySyncCANBitRateCode()
{
	switch (canRate)
	{		
	case canRate1Mbps: 
		{
			sCANBitRateCode = "S8";
		}
		break;

	case canRate500kbps:
		{
			sCANBitRateCode = "S6";
		}
		break;

	case canRate250kbps:
		{
			sCANBitRateCode = "S5";
		}
		break;

	case canRate125kbps:
		{
			sCANBitRateCode = "S4";
		}
		break;

	case canRate100kbps:
		{
			sCANBitRateCode = "S3";
		}
		break;

	case canRate50kbps:
		{
			sCANBitRateCode = "S2";
		}
		break;

	case canRate10kps:
		{
			sCANBitRateCode = "S0";
		}
		break;

	default:
		{
			sCANBitRateCode = "S8";  // Default is 1Mbps
		}
		break;
	}
}
