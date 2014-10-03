//#include <iostream>
//#include <cstdio>
//#include <fstream>

#include <string>
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#include <boost/foreach.hpp>
#endif

#include "DebugPrintf.h"
#include "BlockingReader.h"

#include "AWLSettings.h"
#include "Tracker.h"
#include "ReceiverCapture.h"
#include "ReceiverCANCapture.h"


using namespace std;
using namespace awl;

const float maxIntensity = 1024.0;
const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

const std::string	sDefaultEasySyncBitRate = "S8";  // Bit rate command for EasySDync CAN Adapter: "S2" = 50Kbps,  "S8" = 1Mbps
const long		    defaultSerialPortRate = 921600;  // Default PC Serial Port rate for EasySync CAN adapter.
const uint16_t		defaultYearOffset = 1900;		// On AWL, all CAN Dates are sent as offsets from 1900 
const uint16_t		defaultMonthOffset = 1;		// On AWL, all CAN months start at 0.  Posix months start at 1.


#define ConvertIntensityToSNR(v) (((v)/2.0) - 21.0)


ReceiverCANCapture::ReceiverCANCapture(int receiverID, int inReceiverChannelQty, const std::string &inSerialPort, 
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos):
ReceiverCapture(receiverID, inReceiverChannelQty, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, inParametersAlgos),
port(NULL),
reader(NULL),
io(),
responseString(""),
closeCANReentryCount(0)

{
	// Update settings from application
	sCommPort = inSerialPort;


	serialPortRate = defaultSerialPortRate;
	sBitRate = sDefaultEasySyncBitRate;// "S2" = 50Kbps,  "S8" = 1Mbps
	yearOffset = defaultYearOffset;
	monthOffset = defaultMonthOffset;

	OpenDebugFile(debugFile, "CanBusLog.dat");

	DebugFilePrintf(debugFile, "StartProgram %d", 22);
}


ReceiverCANCapture::ReceiverCANCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCapture(receiverID, propTree),
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
	sBitRate = sDefaultEasySyncBitRate;// "S2" = 50Kbps,  "S8" = 1Mbps
	yearOffset = defaultYearOffset;
	monthOffset = defaultMonthOffset;

	OpenDebugFile(debugFile, "CanBusLog.dat");

	DebugFilePrintf(debugFile, "StartProgram %d", 22);
}

ReceiverCANCapture::~ReceiverCANCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}

void  ReceiverCANCapture::Go() 
	
{
 	assert(!mThread);

	
	if (OpenCANPort())
	{
		WriteCurrentDateTime();
		ReceiverCapture::SetMessageFilters();
	}

	mWorkerRunning = true;
	startTime = boost::posix_time::microsec_clock::local_time();

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverCANCapture::DoThreadLoop, this)));
#ifdef _WINDOWS_
	// Set the priority under windows.  This is the most critical display thread 
	// for user interaction
	

	 HANDLE th = mThread->native_handle();
	 SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
	//   SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif

}

void  ReceiverCANCapture::Stop() 
{
	
	if (mWorkerRunning) 
	{
		CloseCANPort();
	}
	ReceiverCapture::Stop();
}


void ReceiverCANCapture::DoThreadLoop()

{

	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

bool  ReceiverCANCapture::OpenCANPort()

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
#if 1
	port->set_option(boost::asio::serial_port_base::baud_rate(serialPortRate));
#endif

	// Send the initialization strings
	WriteString(sBitRate+"\r"); // Set CAN Rate ("S2"->50Kbps, "S3"->100Kbps, "S8"->1Gbps)
	WriteString("O\r");  // Open
	WriteString("Z0\r");  // Make sure no timestamps are attached
	WriteString("E\r");  // Flush/ Resync
#if 1
	port->set_option(boost::asio::serial_port_base::baud_rate(serialPortRate));
#endif

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

bool  ReceiverCANCapture::CloseCANPort()

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
	    closeCANReentryCount--;
		return(true);
}


void ReceiverCANCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
		AWLCANMessage msg;
		float distance;

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
					ReceiverCapture::SetMessageFilters();
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
					ReceiverCapture::SetMessageFilters();
				}
			}

		}

	} // if  (!WasStoppped)


}

bool ReceiverCANCapture::GetDataByte(std::string &inResponse, uint8_t &outByte, int startIndex, int len)

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

bool ReceiverCANCapture::GetStandardID(std::string &inResponse,  unsigned long &outID, int startIndex)

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

	outID = (highByte * (unsigned long)256) + lowByte; 

	return(true);
}

bool ReceiverCANCapture::ParseLine(std::string inResponse, AWLCANMessage &outMsg)
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

	if (inResponse.length() < (6 + outMsg.len))
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


void ReceiverCANCapture::ParseMessage(AWLCANMessage &inMsg)

{
	unsigned long msgID = inMsg.id;

	if (msgID == 1) 
	{
		ParseSensorStatus(inMsg);
	}
	else if (msgID == 2) 
	{
		ParseSensorBoot(inMsg);
	}
	else if (msgID == 9)
	{
		ProcessCompletedFrame();
	}
	else if (msgID == 10) 
	{
		ParseObstacleTrack(inMsg);
	}
	else if (msgID == 11) 
	{
		ParseObstacleVelocity(inMsg);
	}
	else if (msgID == 12) 
	{
		ParseObstacleSize(inMsg);
	}
	else if (msgID == 13) 
	{
		ParseObstacleAngularPosition(inMsg);
	}
	else if (msgID >= 20 && msgID <= 26) 
	{
		ParseChannelDistance(inMsg);
	}
	else if (msgID >= 30 && msgID <= 36) 
	{
		ParseChannelDistance(inMsg);
	}
	else if (msgID >= 40 && msgID <= 46) 
	{
		ParseChannelIntensity(inMsg);
	}
	else if (msgID >= 50 && msgID <= 56) 
	{
		ParseChannelIntensity(inMsg);
	}
	else if (msgID == 80) /* Command */
	{
		ParseControlMessage(inMsg);
	}
	else
	{
		DebugFilePrintf(debugFile, "UnknownMessage %d", msgID);
		InvalidateFrame();
	}
}


void ReceiverCANCapture::ParseSensorStatus(AWLCANMessage &inMsg)

{
	uint16_t *uintDataPtr = (uint16_t *) inMsg.data;
	uint8_t *byteDataPtr = (uint8_t *) inMsg.data;
	int16_t *intDataPtr = (int16_t *) inMsg.data;

	if (inMsg.id != 1) return;

	boost::mutex::scoped_lock rawLock(GetMutex());

	int iTemperature = intDataPtr[0];
	receiverStatus.temperature =  iTemperature / 10.0;

	unsigned int  uiVoltage = uintDataPtr[1];
	receiverStatus.voltage = uiVoltage;

	receiverStatus.frameRate = byteDataPtr[4];

	receiverStatus.hardwareError.byteData = byteDataPtr[5];
	receiverStatus.receiverError.byteData = byteDataPtr[6];
	receiverStatus.status.byteData = byteDataPtr[7];
	receiverStatus.bUpdated = true;
	rawLock.unlock();

	DebugFilePrintf(debugFile, "Msg %d - Val %u %d %u %u %u %u", inMsg.id, 
			iTemperature, uiVoltage, 
			receiverStatus.frameRate, 
			receiverStatus.hardwareError.byteData,
			receiverStatus.receiverError.byteData,
			receiverStatus.status.byteData);

}

void ReceiverCANCapture::ParseSensorBoot(AWLCANMessage &inMsg)

{
	uint16_t *uintDataPtr = (uint16_t *) inMsg.data;
	uint8_t *byteDataPtr = (uint8_t *) inMsg.data;
	int16_t *intDataPtr = (int16_t *) inMsg.data;

	if (inMsg.id != 2) return;

	boost::mutex::scoped_lock rawLock(GetMutex());

	receiverStatus.version.major =  byteDataPtr[0];
	receiverStatus.version.minor =  byteDataPtr[1];

	receiverStatus.bootChecksumError.byteData = byteDataPtr[2];
	receiverStatus.bootSelfTest.byteData = byteDataPtr[3];
	receiverStatus.bUpdated = true;

	rawLock.unlock();

	DebugFilePrintf(debugFile, "Msg %d - Val %u %u %u %u", inMsg.id, 
			receiverStatus.version.major,
			receiverStatus.version.minor,
			receiverStatus.bootChecksumError.byteData,
			receiverStatus.receiverError.byteData,
			receiverStatus.bootSelfTest.byteData);
}

void ReceiverCANCapture::ParseChannelDistance(AWLCANMessage &inMsg)

{
	int channel;
	int block = 0;
	int detectOffset = 0;
	uint16_t *distancePtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= 30) 
	{
		channel = inMsg.id - 30;
		detectOffset = 4;
		block = 1;
	}
	else 
	{
		channel = inMsg.id - 20;
		block = 0;
	}

	if (channel >= 0) 
	{
		boost::mutex::scoped_lock rawLock(GetMutex());
		float distance = (float)(distancePtr[0]);
		distance /= 100;
		distance += measurementOffset;

		int detectionIndex = 0+detectOffset;
		Detection::Ptr detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		
		detection->distance = distance;

		detection->firstTimeStamp = currentFrame->timeStamp;
		detection->timeStamp = currentFrame->timeStamp;

		detection->trackID = 0;
		detection->velocity = 0;

		distance = (float)(distancePtr[1]);
		distance /= 100;
		distance += measurementOffset;

		detectionIndex = 1+detectOffset;
		detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->timeStamp;
		detection->timeStamp = currentFrame->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;
	
		
		distance = (float)(distancePtr[2]);
		distance /= 100;
		distance += measurementOffset;

		detectionIndex = 2+detectOffset;
		detection =currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->timeStamp;
		detection->timeStamp = currentFrame->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;
	

		distance = (float)(distancePtr[3]);
		distance /= 100;
		distance += measurementOffset;

		detectionIndex = 3+detectOffset;
		detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->timeStamp;
		detection->timeStamp = currentFrame->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;
	
		rawLock.unlock();
	}

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %d - Val %d %d %d %d", inMsg.id, distancePtr[0], distancePtr[1], distancePtr[2], distancePtr[3]);
}


void ReceiverCANCapture::ParseChannelIntensity(AWLCANMessage &inMsg)

{
	int channel;
	int detectOffset = 0;
	uint16_t *intensityPtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= 50) 
	{
		channel = inMsg.id - 50;
		detectOffset = 4;
	}
	else 
	{
		channel = inMsg.id - 40;
	}

	boost::mutex::scoped_lock rawLock(GetMutex());
	float intensity = ((float) intensityPtr[0]) / maxIntensity;
	int detectionIndex = 0+detectOffset;
	Detection::Ptr detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
	detection->intensity = ConvertIntensityToSNR(intensity);
	detection->trackID = 0;
	detection->velocity = 0;

	intensity = ((float) intensityPtr[1]) / maxIntensity;
	detectionIndex = 1+detectOffset;
	detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
	detection->intensity = ConvertIntensityToSNR(intensity);
	detection->trackID = 0;
	detection->velocity = 0;

	intensity = ((float) intensityPtr[2]) / maxIntensity;
	detectionIndex = 2+detectOffset;
	detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
	detection->intensity = ConvertIntensityToSNR(intensity);
	detection->trackID = 0;
	detection->velocity = 0;

	intensity = ((float) intensityPtr[3]) / maxIntensity;
	detectionIndex = 3+detectOffset;
	detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
	detection->intensity = ConvertIntensityToSNR(intensity);
	detection->trackID = 0;
	detection->velocity = 0;
	rawLock.unlock();

	DebugFilePrintf(debugFile, "Msg %d - Val %d %d %d %d", inMsg.id, intensityPtr[0], intensityPtr[1], intensityPtr[2], intensityPtr[3]);
}



void ReceiverCANCapture::ParseObstacleTrack(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID =  *(uint16_t *) &inMsg.data[0];
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);
	track->channels = *(uint8_t *) &inMsg.data[2];
	uint16_t trackType = *(uint16_t *) &inMsg.data[3];
	track->probability = *(uint8_t *) &inMsg.data[5];
	track->timeToCollision = (*(uint8_t *) &inMsg.data[6]) / 1000.0;  // Convert from ms to seconds.  Currently empty

	track->part1Entered = true;

	rawLock.unlock();
	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %d - Track %u Val %d %x %d %d", inMsg.id, track->trackID, track->channels, track->probability, track->timeToCollision);
}


void ReceiverCANCapture::ParseObstacleVelocity(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID =  *(uint16_t *) &inMsg.data[0];
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);

	track->distance = (*(uint16_t *) &inMsg.data[2]);
	track->distance /= 100; // Convert the distance from CM to meters.
	track->distance += measurementOffset;

	int16_t velocity = (*(int16_t *) &inMsg.data[4]);
	track->velocity = velocity / 100.0; // Convert the velocity from cm/s to m/s

	int16_t acceleration = (*(int16_t *) &inMsg.data[6]);
	track->acceleration = acceleration / 100.0; // Convert the velocity from cm/s to m/s

	track->part2Entered = true;

	rawLock.unlock();

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %d - Track %u Val %f %f %f %f", inMsg.id, track->trackID, track->distance, track->velocity, track->acceleration);
}


void ReceiverCANCapture::ParseObstacleSize(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID =  *(uint16_t *) &inMsg.data[0];
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);

	uint16_t height  = (*(uint16_t *) &inMsg.data[2]);
	uint16_t width = (*(uint16_t *) &inMsg.data[4]);
	uint16_t intensity = (*(uint16_t *) &inMsg.data[6]);

	track->intensity = ConvertIntensityToSNR(intensity);

	track->part3Entered = true;

	rawLock.unlock();

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %d - Track %u Val %u %u %u", inMsg.id, track->trackID, intensity, height, width);
}

void ReceiverCANCapture::ParseObstacleAngularPosition(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID =  *(uint16_t *) &inMsg.data[0];
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);

	uint16_t startAngle  = (*(uint16_t *) &inMsg.data[2]);
	uint16_t endAngle = (*(uint16_t *) &inMsg.data[4]);
	uint16_t angularVelocity = (*(uint16_t *) &inMsg.data[6]);


	// Nothing done with this message yet, just log that we have received it.
	track->part4Entered = true;

	rawLock.unlock();

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %d - Track %u Val %u %u %u", inMsg.id, track->trackID, startAngle, endAngle, angularVelocity);
}


void ReceiverCANCapture::ParseControlMessage(AWLCANMessage &inMsg)
{
	unsigned char  commandID = inMsg.data[0];

	switch(commandID)
	{
	case 0xC0:
		ParseParameterSet(inMsg);
		break;
	case 0xC1:
		ParseParameterQuery(inMsg);
		break;
	case 0xC2:
		ParseParameterResponse(inMsg);
		break;
	case 0xC3:
		ParseParameterError(inMsg);
		break;
	default:
		DebugFilePrintf(debugFile, "Error: Unhandled control message (%x).  Message skipped", inMsg.data[0]);
		break;
	}

}

void ReceiverCANCapture::ParseParameterSet(AWLCANMessage &inMsg)
{
	DebugFilePrintf(debugFile, "Error: Command - Parameter - Set received (%x).  Message skipped. Type: %d", inMsg.data[0], inMsg.data[1]);
}


void ReceiverCANCapture::ParseParameterQuery(AWLCANMessage &inMsg)
{
	DebugFilePrintf(debugFile, "Error: Command - Parameter - Set received (%x).  Message skipped. Type: %d", inMsg.data[0], inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterResponse(AWLCANMessage &inMsg)
{
	unsigned char paramType = inMsg.data[1];

	switch (paramType) {
	case 0x01:
		ParseParameterAlgoSelectResponse(inMsg);
		break;
	case 0x02:
		ParseParameterAlgoParameterResponse(inMsg);
		break;
	case 0x03:
		ParseParameterFPGARegisterResponse(inMsg);
		break;
	case 0x04:
		ParseParameterBiasResponse(inMsg);
		break;
	case 0x05:
		ParseParameterADCRegisterResponse(inMsg);
		break;
	case 0x06:
		ParseParameterPresetResponse(inMsg);
		break;
	case 0x07:
		ParseParameterGlobalParameterResponse(inMsg);
		break;
	case 0x08:
		ParseParameterGPIORegisterResponse(inMsg);
		break;
	case 0x20:
		ParseParameterDateTimeResponse(inMsg);
		break;
	case 0xD0:
		ParseParameterRecordResponse(inMsg);
		break;
	case 0xD1:
		ParseParameterPlaybackResponse(inMsg);
		break;
	default:
		break;
	}
}


void ReceiverCANCapture::ParseParameterError(AWLCANMessage &inMsg)
{
	unsigned char paramType = inMsg.data[1];

	switch (paramType) {
	case 0x01:
		ParseParameterAlgoSelectError(inMsg);
		break;
	case 0x02:
		ParseParameterAlgoParameterError(inMsg);
		break;
	case 0x03:
		ParseParameterFPGARegisterError(inMsg);
		break;
	case 0x04:
		ParseParameterBiasError(inMsg);
		break;
	case 0x05:
		ParseParameterADCRegisterError(inMsg);
		break;
	case 0x06:
		ParseParameterPresetError(inMsg);
		break;
	case 0x07:
		ParseParameterGlobalParameterError(inMsg);
		break;
	case 0x08:
		ParseParameterGPIORegisterError(inMsg);
		break;
	case 0x20:
		ParseParameterDateTimeError(inMsg);
		break;
	case 0xD0:
		ParseParameterRecordError(inMsg);
		break;
	case 0xD1:
		ParseParameterPlaybackError(inMsg);
		break;
	default:
		break;
	}
}


void ReceiverCANCapture::ParseParameterAlgoSelectResponse(AWLCANMessage &inMsg)
{

	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];  // Unused
	uint32_t registerValue=  *(uint32_t *) &inMsg.data[4];
	
	// Check that the algorithm is valid (just in case communication goes crazy)
	if (registerValue >= 1 && registerValue <= ALGO_QTY) 
	{
		receiverStatus.currentAlgo = registerValue;
		receiverStatus.currentAlgoPendingUpdates--;
	}
	else
	{
		DebugFilePrintf(debugFile, "Error: Algo select invalid %lx", registerValue); 
	}
}

void ReceiverCANCapture::ParseParameterAlgoParameterResponse(AWLCANMessage &inMsg)
{
	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t registerValue=  *(uint32_t *) &inMsg.data[4];

	AlgorithmParameter *parameter = FindAlgoParamByAddress(receiverStatus.currentAlgo, registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;

	if (parameter != NULL)
	{
		parameter->floatValue = *(float *) &registerValue; 
		parameter->intValue = *(int16_t *) &registerValue; 
		parameter->pendingUpdates--;
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverCANCapture::ParseParameterFPGARegisterResponse(AWLCANMessage &inMsg)
{
	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t registerValue=  *(uint32_t *) &inMsg.data[4];

	int index = FindRegisterByAddress(registersFPGA, registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;

	if (index >= 0)
	{
		registersFPGA[index].value = registerValue; 
		if (registersFPGA[index].pendingUpdates > 0)
		{
			registersFPGA[index].pendingUpdates--;
		}
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverCANCapture::ParseParameterBiasResponse(AWLCANMessage &inMsg)
{
	// Message not used. We ignore the message for the moment.
}

void ReceiverCANCapture::ParseParameterADCRegisterResponse(AWLCANMessage &inMsg)
{
	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t registerValue=  *(uint32_t *) &inMsg.data[4];

	int index = FindRegisterByAddress(registersADC, registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.adcRegisterAddressRead = registerAddress;
	receiverStatus.adcRegisterValueRead = registerValue;

	if (index >= 0)
	{
		registersADC[index].value = registerValue; 
		if (registersADC[index].pendingUpdates > 0)
		{
			registersADC[index].pendingUpdates--;
		}
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}


void ReceiverCANCapture::ParseParameterPresetResponse(AWLCANMessage &inMsg)
{
	// Message not used. We ignore the message for the moment.
}

void ReceiverCANCapture::ParseParameterGlobalParameterResponse(AWLCANMessage &inMsg)
{
	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t registerValue=  *(uint32_t *) &inMsg.data[4];
	int globalAlgo = 0; // Just so we know....

	AlgorithmParameter *parameter = FindAlgoParamByAddress(GLOBAL_PARAMETERS_INDEX, registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;

	if (parameter!= NULL)
	{
		parameter->floatValue = *(float *) &registerValue; 
		parameter->intValue = *(int16_t *) &registerValue; 
		parameter->pendingUpdates--;
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverCANCapture::ParseParameterGPIORegisterResponse(AWLCANMessage &inMsg)
{
	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t registerValue=  *(uint32_t *) &inMsg.data[4];

	int index = FindRegisterByAddress(registersGPIO, registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.gpioRegisterAddressRead = registerAddress;
	receiverStatus.gpioRegisterValueRead = registerValue;

	if (index >= 0)
	{
		registersGPIO[index].value = registerValue; 
		if (registersGPIO[index].pendingUpdates > 0)
		{
			registersGPIO[index].pendingUpdates--;
		}
	}


	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverCANCapture::ParseParameterDateTimeResponse(AWLCANMessage &inMsg)
{
	// Message should be sent as a response when we change the date.
	// Otherwise it is not used. We ignore the message for the moment.
}

void ReceiverCANCapture::ParseParameterRecordResponse(AWLCANMessage &inMsg)
{
	// Message should be sent as a response when we set record filename.
	// Otherwise it is not used. We ignore the message for the moment.
}


void ReceiverCANCapture::ParseParameterPlaybackResponse(AWLCANMessage &inMsg)
{
	// Message should be sent as a response when we set playbackfilename.
	// Otherwise it is not used. We ignore the message for the moment.
}


void ReceiverCANCapture::ParseParameterAlgoSelectError(AWLCANMessage &inMsg)
{
	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterAlgoParameterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterFPGARegisterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterBiasError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterADCRegisterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterPresetError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterGlobalParameterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterGPIORegisterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterDateTimeError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterRecordError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterPlaybackError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::WriteString(std::string inString)
{
	if (!port) return;

	int stringSize = inString.size();

DebugFilePrintf(debugFile, "Out %d bytes - %s", stringSize, inString.c_str());

	std::size_t written = boost::asio::write(*port,boost::asio::buffer(inString.c_str(),stringSize));
DebugFilePrintf(debugFile, "Out %d bytes confirmed", written);
	// Messages must be at leat 1ms apart.
	boost::this_thread::sleep(boost::posix_time::milliseconds(10));
}

bool ReceiverCANCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	std::string outResponse;
	char outString[10];

	if (!port) return(false);

	outResponse += "t";

	sprintf(outString, "%01x%02x", (inMsg.id / 256),  inMsg.id & 0xFF);
	outResponse +=outString;

	sprintf(outString, "%01x", inMsg.len);
	outResponse +=outString;


	for (int i = 0; i < inMsg.len; i++) 
	{
		sprintf(outString, "%02x", inMsg.data[i]);
		outResponse +=outString;
	}

	outResponse += "\r";

	WriteString(outResponse);
	return(true);
}


/*
	00: Command (0xC0 = SET_PARAMETER)
             0xC1 = QUERY_PARAMETER)
             0xC2 = RESPONSE_PARAMETER)
01: Type (0x01 = ALGO_SELECTED
          0x02 = ALGO_PARAMETER
          0x03 = AWL_REGISTER
          0x04 = BIAS
          0x05 = ADC_REGISTER
          0x06 = PRESET
		  0x07 = GLOBAL_PARAMETER (Histogram)
		  0x08 = GPIO_CONTROL
          0x20 = DATE_TIME
          0xD0 = RECORD_FILENAME (zero-terminated)
          0xD1 = PLAYBACK_FILENAME (zero-terminated)
02-03: Address (U16_LE)
04-07: Value (x32_LE or U8S)

for DATE:
04-05: YEAR (U16_LE)
06: MONTH
07: DAY-OF-MONTH

for TIME:
04: HOURS
05: MINUTES
06: SECONDS
07: 0x00
*/

bool ReceiverCANCapture::WriteCurrentDateTime()
{
	//First get indivitual elements of current date and time
	boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());
	boost::gregorian::date gregDate = myTime.date();
	boost::posix_time::time_duration timeOfDay = myTime.time_of_day();
	
	boost::gregorian::greg_year year = gregDate.year();
	boost::gregorian::greg_month month = gregDate.month();
	boost::gregorian::greg_day day = gregDate.day();

	boost::posix_time::time_duration::hour_type hours = timeOfDay.hours();
	boost::posix_time::time_duration::min_type minutes = timeOfDay.minutes();
	boost::posix_time::time_duration::sec_type seconds = timeOfDay.seconds();

	AWLCANMessage message;
	bool bMessageOk(true);

	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // SET_PARAMETER
	message.data[1] = 0x20;    // SET_DATE_TIME

	*((uint16_t*)&message.data[2]) = 0x0001; // SET_DATE
	*((uint16_t*)&message.data[4]) = year-yearOffset;
	message.data[6] = (unsigned char) month-monthOffset;
	message.data[7] = (unsigned char) day;

	bMessageOk = bMessageOk && WriteMessage(message);

	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // SET_PARAMETER
	message.data[1] = 0x20;    // SET_DATE_TIME

	*((uint16_t*)&message.data[2]) = 0x0002; // SET_TIME
	message.data[4] = (unsigned char) hours;
	message.data[5] = (unsigned char) minutes;
	message.data[6] = (unsigned char) seconds;
	message.data[7] = 0x00; // Not used


	bMessageOk = bMessageOk && WriteMessage(message);

	return(bMessageOk);
} 

const int nameBlockSize = 6;

bool ReceiverCANCapture::SetPlaybackFileName(std::string inPlaybackFileName)
{
	receiverStatus.sPlaybackFileName = inPlaybackFileName;

	int nameLength = inPlaybackFileName.length();
	bool bMessageOk(true);
	
	// Write name strings in block of 6 characters.
	// terminating NULL is end of message.
	for (int blockOffset = 0; blockOffset < nameLength+1; blockOffset += nameBlockSize)
	{
		AWLCANMessage message;
		message.id = 80;       // Message id: 80- Command message

		message.len = 8;       // Frame size (0.8)
		message.data[0] = 0xC0;   // SET_PARAMETER
		message.data[1] = 0xD1;    // PLAYBACK_FILENAME

		for(int offset = 0; (offset < nameBlockSize)  && bMessageOk; offset++)
		{
			if (blockOffset+offset < nameLength) 
			{
				message.data[offset+2] = inPlaybackFileName.at(blockOffset+offset);
			}
			else 
			{
				message.data[offset+2] = 0;
			}
		}

		bMessageOk = bMessageOk && WriteMessage(message);
	}

	if (bMessageOk) receiverStatus.sPlaybackFileName = inPlaybackFileName;

	return(bMessageOk);
}


bool ReceiverCANCapture::SetRecordFileName(std::string inRecordFileName)
{
	int nameLength = inRecordFileName.length();
	bool bMessageOk(true);
	
	// Write name strings in block of 6 characters.
	// terminating NULL is end of message.
	for (int blockOffset = 0; blockOffset < nameLength+1; blockOffset += nameBlockSize)
	{
		AWLCANMessage message;
		message.id = 80;       // Message id: 80- Command message

		message.len = 8;       // Frame size (0.8)
		message.data[0] = 0xC0;   // SET_PARAMETER
		message.data[1] = 0xD0;    // Record_FILENAME

		for(int offset = 0; (offset < nameBlockSize)  && bMessageOk; offset++)
		{
			if (blockOffset+offset < nameLength) 
			{
				message.data[offset+2] = inRecordFileName.at(blockOffset+offset);
			}
			else 
			{
				message.data[offset+2] = 0;
			}
		}

		bMessageOk = bMessageOk && WriteMessage(message);
	}

	if (bMessageOk) receiverStatus.sRecordFileName = inRecordFileName;
	return(bMessageOk);
}

bool ReceiverCANCapture::StartPlayback(uint8_t frameRate, ChannelMask channelMask)
{
	AWLCANMessage message;
	
	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xD1;   // PLAYBACK_RAW
	message.data[1] = channelMask.byteData;   // Channel mask. Mask at 0 stops playback

	message.data[2] = 0x00; // Not used
	message.data[3] = frameRate; // Frame rate in HZ. 00: Use actual
	message.data[4] = 0x00; // Not used
	message.data[5] = 0x00; // Not used
	message.data[6] = 0x00; // Not used
	message.data[7] = 0x00; // Not used

	bool bMessageOk = WriteMessage(message);

	receiverStatus.bInPlayback = bMessageOk;
	if (frameRate > 0) receiverStatus.frameRate = frameRate;
	return(bMessageOk);
}

bool ReceiverCANCapture::StartRecord(uint8_t frameRate, ChannelMask channelMask)
{
	AWLCANMessage message;
	
	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xD0;   // Record_RAW
	message.data[1] = channelMask.byteData;   // Channel mask. Mask at 0 stops record

	message.data[2] = 0x00; // Not used
	message.data[3] = frameRate; 
	message.data[4] = 0x00; // Not used
	message.data[5] = 0x00; // Not used
	message.data[6] = 0x00; // Not used
	message.data[7] = 0x00; // Not used

	bool bMessageOk = WriteMessage(message);

	receiverStatus.bInRecord = bMessageOk;
	if (frameRate > 0) receiverStatus.frameRate = frameRate;

	return(bMessageOk);
}

bool ReceiverCANCapture::StopPlayback()
{
	AWLCANMessage message;
	
	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xD1;   // PLAYBACK_RAW
	message.data[1] = 0x00;  // Mask at 0 stops the playback 

	message.data[2] = 0x00; // Not used
	message.data[3] = receiverStatus.frameRate; // Frame rate
	message.data[4] = 0x00; // Not used
	message.data[5] = 0x00; // Not used
	message.data[6] = 0x00; // Not used
	message.data[7] = 0x00; // Not used

	bool bMessageOk = WriteMessage(message);

	if (bMessageOk)
	{
		receiverStatus.bInPlayback = false;
		receiverStatus.bInRecord = false;
	}

	return(bMessageOk);
}
	
bool ReceiverCANCapture::StopRecord()
{
		AWLCANMessage message;
	
	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xD0;   // RECORD_RAW
	message.data[1] = 0x00;  // Mask at 0 stops the recording 

	message.data[2] = 0x00; // Not used
	message.data[3] = receiverStatus.frameRate; // Frame rate
	message.data[4] = 0x00; // Not used
	message.data[5] = 0x00; // Not used
	message.data[6] = 0x00; // Not used
	message.data[7] = 0x00; // Not used

	bool bMessageOk = WriteMessage(message);

	if (bMessageOk)
	{
		receiverStatus.bInPlayback = false;
		receiverStatus.bInRecord = false;
	}
	return(bMessageOk);
}

bool ReceiverCANCapture::StartCalibration(uint8_t frameQty, float beta, ChannelMask channelMask)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xDA;   // Record_Calibration
	message.data[1] = channelMask.byteData;   

	message.data[2] = frameQty; // Number of frames
	message.data[3] = 0; // Not used
	*((float *) &message.data[4]) = beta;

	bool bMessageOk = WriteMessage(message);
	return(bMessageOk);
}

bool ReceiverCANCapture::SetAlgorithm(uint16_t algorithmID)
{

	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x01; // Algo-select  

	* (int16_t *) &message.data[2] = 0L; // Unused
	* (int32_t *) &message.data[4] = algorithmID;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of the register settings.
	
	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentAlgo = algorithmID;
	receiverStatus.currentAlgoPendingUpdates = 1;

   return(bMessageOk);
}


bool ReceiverCANCapture::SetFPGARegister(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x03; // AWL_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	int index = FindRegisterByAddress(registersFPGA,registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersFPGA[index].pendingUpdates = 1;
	}

    return(bMessageOk);
}

bool ReceiverCANCapture::SetADCRegister(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x05; // ADC_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	int index = FindRegisterByAddress(registersADC, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersADC[index].pendingUpdates = 1;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::SetGPIORegister(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x08; // GPIO_Control  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	int index = FindRegisterByAddress(registersGPIO, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersGPIO[index].pendingUpdates = 1;
	}


	return(bMessageOk);
}

bool ReceiverCANCapture::SetAlgoParameter(int algoID, uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x02; // ALGO_PARAMETER 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	AlgorithmParameter *parameter = FindAlgoParamByAddress(algoID, registerAddress);
	if (parameter != NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = 1;
	}

 	return(bMessageOk);
}

bool ReceiverCANCapture::SetGlobalAlgoParameter(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x07; // GLOBAL_PARAMETER 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	AlgorithmParameter *parameter = FindAlgoParamByAddress(GLOBAL_PARAMETERS_INDEX, registerAddress);
	if (parameter!= NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = 1;
	}

 	return(bMessageOk);
}


bool ReceiverCANCapture::SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask)

{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xE1;   // Transmit_cooked enable flags

	message.data[1] = channelMask.byteData; // Channel mask
	message.data[2] = 0;  // Reserved
	message.data[3] = frameRate; // New frame rate. oo= use actual.
	message.data[4] = messageMask.byteData;
	message.data[5] = 0;  // Reserved
	message.data[6] = 0;  // Reserved
	message.data[7] = 0;  // Reserved

	bool bMessageOk = WriteMessage(message);

	// The message has no confirmation built in

   return(bMessageOk);
}

bool ReceiverCANCapture::QueryAlgorithm()
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x01; // Algo-select  

	* (int16_t *) &message.data[2] = 0L;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of the register settings.
	
	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentAlgoPendingUpdates = 1;

    return(bMessageOk);
}


bool ReceiverCANCapture::QueryFPGARegister(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x03; // AWL_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of the register settings.
	int index = FindRegisterByAddress(registersFPGA, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersFPGA[index].pendingUpdates = 1;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryADCRegister(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x05; // ADC_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	int index = FindRegisterByAddress(registersADC, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersADC[index].pendingUpdates = 1;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryGPIORegister(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x08; // GPIO Control  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	int index = FindRegisterByAddress(registersGPIO, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersGPIO[index].pendingUpdates = 1;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryAlgoParameter(int algoID, uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x02; // Algorithm parameter 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	AlgorithmParameter *parameter = FindAlgoParamByAddress(algoID, registerAddress);
	if (parameter != NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = 1;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryGlobalAlgoParameter(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x07; // Global parameter 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	AlgorithmParameter *parameter = FindAlgoParamByAddress(GLOBAL_PARAMETERS_INDEX, registerAddress);
	if (parameter!= NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = 1;
	}

	return(bMessageOk);
}


bool ReceiverCANCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		ReceiverCapture::ReadConfigFromPropTree(propTree);

		char receiverKeyString[32];
		sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
		std::string receiverKey = receiverKeyString;

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
		// Communication parameters
		sCommPort =  receiverNode.get<std::string>("commPort");

		return(true);
}

bool ReceiverCANCapture::ReadRegistersFromPropTree( boost::property_tree::ptree &propTree)
{
	using boost::property_tree::ptree;

	registersFPGA.clear();
	registersADC.clear();
	registersGPIO.clear();
	parametersAlgos.algorithms.clear();

	// Read all FPGA Registers default descriptions
	char registerDescKeyString[255];
	sprintf(registerDescKeyString, "config.registerDescription_RevC");
	std::string registerDescKey = registerDescKeyString;

	// The register configuration section may be absent from the configuration.
	// This is considered a normal situation.
	boost::property_tree::ptree *configurationNodePtr = NULL;

	try 
	{
		// The register configuration section may be absent from the configuration.
		// This is considered a normal situation.
		configurationNodePtr =  &propTree.get_child(registerDescKey);
	}
	catch (boost::exception &e) 
	{ 
		return (false);
	}

	BOOST_FOREACH(ptree::value_type &registersFPGANode, configurationNodePtr->get_child("registersFPGA"))
	{
		if( registersFPGANode.first == "register" ) {
			boost::property_tree::ptree &registerNode = registersFPGANode.second;

            RegisterSetting registerFPGA;
            registerFPGA.sIndex = registerNode.get<std::string>("index");
            registerFPGA.address = registerNode.get<uint16_t>("address");
		    registerFPGA.sDescription = registerNode.get<std::string>("description");
			registerFPGA.value = 0L;
			registerFPGA.pendingUpdates = 0;

			registersFPGA.push_back(registerFPGA);
        }
    }
 
 
	// Read all ADC Registers default descriptions

	BOOST_FOREACH(ptree::value_type &registersADCNode, configurationNodePtr->get_child("registersADC"))
	{
		if( registersADCNode.first == "register" ) 
		{
			boost::property_tree::ptree &registerNode = registersADCNode.second;

            RegisterSetting registerADC;
            registerADC.sIndex = registerNode.get<std::string>("index");
            registerADC.address  = registerNode.get<uint16_t>("address");
		    registerADC.sDescription = registerNode.get<std::string>("description");
			registerADC.value = 0L;
			registerADC.pendingUpdates = 0;

			registersADC.push_back(registerADC);
        }
    }
 
	// Read all GPIO Registers default descriptions

	BOOST_FOREACH(ptree::value_type &registersGPIONode, configurationNodePtr->get_child("GPIOs"))
	{
		if( registersGPIONode.first == "register" ) 
		{
			boost::property_tree::ptree &gpioNode = registersGPIONode.second;

            RegisterSetting registerGPIO;
            registerGPIO.sIndex = gpioNode.get<std::string>("index");
            registerGPIO.address  = gpioNode.get<uint16_t>("address");
		    registerGPIO.sDescription = gpioNode.get<std::string>("description");
			registerGPIO.value = 0L;
			registerGPIO.pendingUpdates = 0;

			registersGPIO.push_back(registerGPIO);
        }
    }

	// Load all algorithm parameters for all algorithms and for global parameters

	parametersAlgos.defaultAlgo = configurationNodePtr->get<uint16_t>("algos.defaultAlgo");
	BOOST_FOREACH(ptree::value_type &algosNode, configurationNodePtr->get_child("algos"))
	{
		if (algosNode.first == "algo")
		{
			boost::property_tree::ptree &algoNode = algosNode.second;
			AlgorithmDescription algoDescription;	
			algoDescription.algoID = algoNode.get<uint16_t>("algoID");
			algoDescription.sAlgoName = algoNode.get<std::string>("algoName");

			// All channel info for the receiver
			BOOST_FOREACH(ptree::value_type &parametersNode, algoNode/*.get_child("parameter")*/)
			{
				if( parametersNode.first == "parameter" ) 
				{
					boost::property_tree::ptree &parameterNode = parametersNode.second;
					AlgorithmParameter parameter;
					parameter.sIndex = parameterNode.get<std::string>("index");
					parameter.address = parameterNode.get<uint16_t>("address");
					parameter.sDescription = parameterNode.get<std::string>("description");
					std::string sType = parameterNode.get<std::string>("type");
					if (!sType.compare("int")) 
					{
						parameter.paramType = eAlgoParamInt;
						parameter.intValue = parameterNode.get<uint32_t>("default");
						parameter.floatValue = 0.0;
					}
					else if (!sType.compare("float")) 
					{
						parameter.paramType = eAlgoParamFloat;
						parameter.intValue = 0;
						parameter.floatValue = parameterNode.get<float>("default");
					}

					parameter.pendingUpdates = 0;
					algoDescription.parameters.push_back(parameter);
				} // if (parametersNode.first)
			} // BOOST_FOREACH (parametersNode)

			parametersAlgos.algorithms.push_back(algoDescription);
		} //		if (algoNode.first == "algo")
	} // BOOST_FOREACH(algosNode)

	return(true);
}

