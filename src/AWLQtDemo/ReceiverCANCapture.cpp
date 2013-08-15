#define CV_NO_BACKWARD_COMPATIBILITY


#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#endif
#include "DebugPrintf.h"
#include "BlockingReader.h"

#include "AWLSettings.h"
#include "Tracker.h"
#include "ReceiverCapture.h"
#include "ReceiverCANCapture.h"


using namespace std;
using namespace pcl;
using namespace awl;

const float maxIntensity = 1024.0;

boost::posix_time::ptime reconnectTime;

ofstream outFile;
const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

ReceiverCANCapture::ReceiverCANCapture(int inSequenceID, int inReceiverChannelQty, int inDetectionsPerChannel, int argc, char** argv):
ReceiverCapture(inSequenceID, inReceiverChannelQty, inDetectionsPerChannel),
port(NULL),
reader(NULL),

currentFrame(new SensorFrame(0, inReceiverChannelQty, inDetectionsPerChannel)),
lastMessageID(0),
io(),
responseString("")


{
	// Update settings from application
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	sBitRate = globalSettings->sCANBitRate.toStdString();// "S2" = 50Kbps,  "S8" = 1Mbps
	sCommPort = globalSettings->sCANCommPort.toStdString();
	serialPortRate = globalSettings->serialCANPortRate;
	yearOffset = globalSettings->yearOffsetCAN;
	monthOffset = globalSettings->monthOffsetCAN;

	OpenDebugFile(outFile, "CanBusLog.dat");
	ProcessCommandLineArguments(argc, argv);

	DebugFilePrintf(outFile, "StartProgram %d", 22);

	if (OpenCANPort())
	{
		WriteCurrentDateTime();
	}

	acquisitionSequence->Clear();
}

ReceiverCANCapture::~ReceiverCANCapture()
{

	CloseCANPort();
	CloseDebugFile(outFile);
	Stop(); // Stop the thread
}

void ReceiverCANCapture::ProcessCommandLineArguments(int argc, char** argv)

{
	const std::string bitRateOpt = "--bitRate=";
	const std::string commPortOpt = "--comPort=";

	// process input arguments
    for( int i = 1; i < argc; i++ )
    {
        if( bitRateOpt.compare( 0, bitRateOpt.length(), argv[i], bitRateOpt.length() ) == 0 )
        {
            sBitRate = argv[i] + bitRateOpt.length();
        }
		if( commPortOpt.compare( 0, commPortOpt.length(), argv[i], commPortOpt.length() ) == 0 )
        {
            sCommPort = argv[i] + commPortOpt.length();
        }
    }

	ReceiverCapture::ProcessCommandLineArguments(argc, argv);
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

	// Send the initialization strings
	WriteString(sBitRate+"\r"); // Set CAN Rate ("S2"->50Kbps, "S3"->100Kbps, "S8"->1Gbps)
	WriteString("O\r");  // Open
	WriteString("E\r");  // Flush/ Resync

	if (reader) 
		return(true);
	else {

		delete(port);
		port = NULL;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}

}

bool  ReceiverCANCapture::CloseCANPort()

{
		try 
		{
			WriteString("C\n"); // Close port
		}
		catch (...)
		{
			DebugFilePrintf(outFile, "Error during Write On Close");
		}


		if (port) port->close();
		reader = NULL;
		port = NULL;
		return(true);
}

void  ReceiverCANCapture::Go(bool inIsThreaded) 
	
{
	bIsThreaded = inIsThreaded;
	assert(!mThread);
    mStopRequested = false;

	startTime = boost::posix_time::microsec_clock::local_time();


	if (bIsThreaded) 
	{
		mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverCANCapture::DoThreadLoop, this)));
	}
}
 

void  ReceiverCANCapture::Stop() 
{
	if (mStopRequested) return;
    mStopRequested = true;
	if (bIsThreaded) 
	{
		bIsThreaded = false;
		assert(mThread);
		mThread->join();
	}
}


bool  ReceiverCANCapture::WasStopped()
{
	if (mStopRequested) return(true);
	return(false);
}

void ReceiverCANCapture::DoThreadLoop()

{

	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverCANCapture::DoThreadIteration()

{
	if (!bIsThreaded)
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

static double lastElapsed = 0;

void ReceiverCANCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
		lastMessageID = 0;
		AWLCANMessage msg;
		float distance;


		if (bSimulatedDataEnabled && ((GetElapsed() - lastElapsed) >= 10)) 
		{
			FakeChannelDistance(20);
			FakeChannelDistance(21);
			FakeChannelDistance(22);
			FakeChannelDistance(23);
			FakeChannelDistance(24);
			FakeChannelDistance(36);
			lastElapsed = GetElapsed();
		}

		if (port && reader && port->is_open()) 
		{
			char c;
			// read from the serial port until we get a
			// \n or until a read times-out (500ms)
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
			}
			// read_char returned false.  
			// This means we have a time out on the port after receiveTimeOutInMillisec.
			// Try to repoen the port
			else 
			{
				DebugFilePrintf(outFile,  "Time Outon read_char.  Resetting CAN Port"); 
				CloseCANPort();
				if (OpenCANPort())
				{
					WriteCurrentDateTime();
				}
			}

		} // if (port->is_open)
		else 
		{
			// Port is not opened.  Try to repoen after a certain delay.

			if (boost::posix_time::microsec_clock::local_time() > reconnectTime)
			{
				DebugFilePrintf(outFile,  "Reconmnecting CAN Port"); 
				if (OpenCANPort())
				{
					WriteCurrentDateTime();
				}
			}

		}

	} // if  (!WasStoppped)


}

bool GetDataByte(std::string &inResponse, uint8_t &outByte, int startIndex, int len = 1)

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
			DebugFilePrintf(outFile, "CanLine error1: %s", inResponse.c_str());
			return(false);
		}
	}

	return(true);
}

bool GetStandardID(std::string &inResponse,  unsigned long &outID, int startIndex)

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
		DebugFilePrintf(outFile, "CanLine empty");
		return bResult;
	}

	if (inResponse[0] != 't') 
	{
		DebugFilePrintf(outFile, "CanLine bad: %s", inResponse.c_str());
		return (bResult);
	}

	
	if (inResponse.length() < 6) 
	{
		DebugFilePrintf(outFile, "CanFrame incomplete %s",  inResponse.c_str());
		return (bResult);
	}

	if (!GetStandardID(inResponse, outMsg.id, 1))
	{
		return(bResult);
	}

	if (!GetDataByte(inResponse, outMsg.len, 4, 1)) 
	{
		return(bResult);
	}

	if (inResponse.length() < (6 + outMsg.len))
	{
		return(bResult);
	}

	if (outMsg.len > 8) 
	{
		return(bResult);
	}

	for (int i = 0; i < outMsg.len; i++) 
	{
		if (!GetDataByte(inResponse, outMsg.data[i], 5+ (i*2), 2))
		{
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
	else if (msgID >= 20 && msgID <= 26) 
	{
		ParseChannelDistance(inMsg);
		lastMessageID = msgID;
	}
	else if (msgID >= 30 && msgID <= 36) 
	{
		ParseChannelDistance(inMsg);
		lastMessageID = msgID;
		// On the last distance message, notify send the sensor frame to the application.
		if (msgID == 36) ProcessCompletedFrame();
	}
	else if (msgID >= 40 && msgID <= 46) 
	{
		ParseChannelIntensity(inMsg);
		lastMessageID = msgID;
	}
	else if (msgID >= 50 && msgID <= 56) 
	{
		ParseChannelIntensity(inMsg);
		lastMessageID = msgID;
	}
	else if (msgID == 80) /* Command */
	{
		ParseControlMessage(inMsg);
	}
	else
	{
		DebugFilePrintf(outFile, "UnknownMessage %d", msgID);
		lastMessageID = msgID;
	}
}


void ReceiverCANCapture::ParseSensorStatus(AWLCANMessage &inMsg)

{
	uint16_t *uintDataPtr = (uint16_t *) inMsg.data;
	uint8_t *byteDataPtr = (uint8_t *) inMsg.data;
	int16_t *intDataPtr = (int16_t *) inMsg.data;

	if (inMsg.id != 1) return;

	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

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

	DebugFilePrintf(outFile, "Msg %d - Val %u %d %u %u %u %u", inMsg.id, 
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

	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

	receiverStatus.version.major =  byteDataPtr[0];
	receiverStatus.version.minor =  byteDataPtr[1];

	receiverStatus.bootChecksumError.byteData = byteDataPtr[2];
	receiverStatus.bootSelfTest.byteData = byteDataPtr[3];
	receiverStatus.bUpdated = true;

	rawLock.unlock();

	DebugFilePrintf(outFile, "Msg %d - Val %u %u %u %u", inMsg.id, 
			receiverStatus.version.major,
			receiverStatus.version.minor,
			receiverStatus.bootChecksumError.byteData,
			receiverStatus.receiverError.byteData,
			receiverStatus.bootSelfTest.byteData);
}

static int channelReorder[] = {0, 1, 2, 3, 4, 5, 6};
//static int channelReorder[] = {0, 1, 2, 3, -1, -1, -1};


void ReceiverCANCapture::ParseChannelDistance(AWLCANMessage &inMsg)

{
	int channel;
	int detectOffset = 0;
	uint16_t *distancePtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= 30) 
	{
		channel = inMsg.id - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = inMsg.id - 20;
	}


	// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];

	if (channel >= 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		float distance = (float)(distancePtr[0]);
		distance /= 100;
		distance += measurementOffset;

		currentFrame->channelFrames[channel]->timeStamp = GetElapsed();
		if (distance < minDistance  || distance > maxDistance) distance = 0.0;

		int detectionIndex = 0+detectOffset;
		
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->firstTimeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->timeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		distance = (float)(distancePtr[1]);
		distance /= 100;
		distance += measurementOffset;

		if (distance < minDistance  || distance > maxDistance) distance = 0.0;
		detectionIndex = 1+detectOffset;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->firstTimeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->timeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		distance = (float)(distancePtr[2]);
		distance /= 100;
		distance += measurementOffset;

		if (distance < minDistance  || distance > maxDistance) distance = 0.0;
		detectionIndex = 2+detectOffset;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->firstTimeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->timeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		distance = (float)(distancePtr[3]);
		distance /= 100;
		distance += measurementOffset;

		if (distance < minDistance  || distance > maxDistance) distance = 0.0;
		detectionIndex = 3+detectOffset;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->firstTimeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->timeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;
		rawLock.unlock();
	}
	DebugFilePrintf(outFile, "Msg %d - Val %d %d %d %d", inMsg.id, distancePtr[0], distancePtr[1], distancePtr[2], distancePtr[3]);
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

	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	float intensity = ((float) intensityPtr[0]) / maxIntensity;
	currentFrame->channelFrames[channel]->detections[0+detectOffset]->intensity = intensity;
	currentFrame->channelFrames[channel]->detections[0+detectOffset]->trackID = 0;
	currentFrame->channelFrames[channel]->detections[0+detectOffset]->velocity = 0;

	intensity = ((float) intensityPtr[1]) / maxIntensity;
	currentFrame->channelFrames[channel]->detections[1+detectOffset]->intensity = intensity;
	currentFrame->channelFrames[channel]->detections[1+detectOffset]->trackID = 0;
	currentFrame->channelFrames[channel]->detections[1+detectOffset]->velocity = 0;

	intensity = ((float) intensityPtr[2]) / maxIntensity;
	currentFrame->channelFrames[channel]->detections[2+detectOffset]->intensity = intensity;
	currentFrame->channelFrames[channel]->detections[2+detectOffset]->trackID = 0;
	currentFrame->channelFrames[channel]->detections[2+detectOffset]->velocity = 0;

	intensity = ((float) intensityPtr[3]) / maxIntensity;
	currentFrame->channelFrames[channel]->detections[3+detectOffset]->intensity = intensity;
	currentFrame->channelFrames[channel]->detections[3+detectOffset]->trackID = 0;
	currentFrame->channelFrames[channel]->detections[3+detectOffset]->velocity = 0;
	rawLock.unlock();

	DebugFilePrintf(outFile, "Msg %d - Val %d %d %d %d", inMsg.id, intensityPtr[0], intensityPtr[1], intensityPtr[2], intensityPtr[3]);
}

void ReceiverCANCapture::ProcessCompletedFrame()

{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

	// timestamp the currentFrame
	double elapsed = GetElapsed();
	currentFrame->timeStamp = GetElapsed();
	// And timestamp all the distances
	int channelQty = currentFrame->channelFrames.size();
	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++) 
	{
			ChannelFrame::Ptr channelPtr = currentFrame->channelFrames.at(channelIndex);
			channelPtr->timeStamp = currentFrame->timeStamp;

			int detectionQty = channelPtr->detections.size();
			for (int detectionIndex = 0; detectionIndex < detectionQty; detectionIndex++) 
			{
				Detection::Ptr detection = channelPtr->detections.at(detectionIndex);
				detection->timeStamp = currentFrame->timeStamp;
				detection->firstTimeStamp = currentFrame->timeStamp;
			}
	}


	// Push the current frame in the frame buffer
	acquisitionSequence->sensorFrames.push(currentFrame);
	currentReceiverCaptureSubscriptions->PutNews();

	// Make sure we do not keep too many of those frames around.
	// Remove the older frame if we exceed the buffer capacity
	if (acquisitionSequence->sensorFrames.size() > maximumSensorFrames) 
	{
		acquisitionSequence->sensorFrames.pop();
	}

	
	// Recalculate the tracks
	acquisitionSequence->BuildTracks(currentFrame->timeStamp);

	// Create a new current frame.
	uint32_t frameID = acquisitionSequence->AllocateFrameID();
	int channelsRequired = acquisitionSequence->channelQty;
	int detectionsRequired = acquisitionSequence->detectionQty;
	
	currentFrame = SensorFrame::Ptr(new SensorFrame(frameID, channelsRequired, detectionsRequired));


	rawLock.unlock();

	DebugFilePrintf(outFile, "FrameID- %lu", frameID);
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
		DebugFilePrintf(outFile, "Error: Unhandled control message (%x).  Message skipped", inMsg.data[0]);
		break;
	}

}

void ReceiverCANCapture::ParseParameterSet(AWLCANMessage &inMsg)
{
	DebugFilePrintf(outFile, "Error: Command - Parameter - Set received (%x).  Message skipped. Type: %d", inMsg.data[0], inMsg.data[1]);
}


void ReceiverCANCapture::ParseParameterQuery(AWLCANMessage &inMsg)
{
	DebugFilePrintf(outFile, "Error: Command - Parameter - Set received (%x).  Message skipped. Type: %d", inMsg.data[0], inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterResponse(AWLCANMessage &inMsg)
{
	unsigned char paramType = inMsg.data[2];

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
		ParseParameterHistogramResponse(inMsg);
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
	unsigned char paramType = inMsg.data[2];

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
		ParseParameterHistogramError(inMsg);
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

	// Note the algorithm.


}

void ReceiverCANCapture::ParseParameterAlgoParameterResponse(AWLCANMessage &inMsg)
{

	// Everything went well the last time we changed the parameters. We ignore the message for the moment.

}

void ReceiverCANCapture::ParseParameterFPGARegisterResponse(AWLCANMessage &inMsg)
{
	uint16_t registerAddress = *(uint16_t *) &inMsg.data[2];
	uint16_t registerValue=  *(uint16_t *) &inMsg.data[4];

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;
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
	uint16_t registerValue=  *(uint16_t *) &inMsg.data[4];

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.adcRegisterAddressRead = registerAddress;
	receiverStatus.adcRegisterValueRead = registerValue;
	receiverStatus.bUpdated = true;
	rawLock.unlock();
}


void ReceiverCANCapture::ParseParameterPresetResponse(AWLCANMessage &inMsg)
{
	// Message not used. We ignore the message for the moment.
}

void ReceiverCANCapture::ParseParameterHistogramResponse(AWLCANMessage &inMsg)
{
	// Message not used yet. We ignore the message for the moment.
}

void ReceiverCANCapture::ParseParameterDateTimeResponse(AWLCANMessage &inMsg)
{
	// Message should be sent as a response when we change the date.
	// Otherwise it is not used. We ignore the message for the moment.
}

void ReceiverCANCapture::ParseParameterRecordResponse(AWLCANMessage &inMsg)
{
}

void ReceiverCANCapture::ParseParameterPlaybackResponse(AWLCANMessage &inMsg)
{
}

void ReceiverCANCapture::ParseParameterAlgoSelectError(AWLCANMessage &inMsg)
{
	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterAlgoParameterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);}

void ReceiverCANCapture::ParseParameterFPGARegisterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterBiasError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterADCRegisterError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterPresetError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterHistogramError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterDateTimeError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterRecordError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterPlaybackError(AWLCANMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf("Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::WriteString(std::string inString)
{
	if (!port) return;

	int stringSize = inString.size();
	boost::asio::write(*port,boost::asio::buffer(inString.c_str(),stringSize));
}

bool ReceiverCANCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	std::string outResponse;
	char outString[5];

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
          0x20 = DATE
          0x21 = TIME
          0x22 = reserved for TIMEZONE
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
	message.data[1] = 0x20;    // SET_DATE

	message.data[2] = 0x00; // Address[0] : Not used
	message.data[3] = 0x00; // Address[1] : Not used
	*((uint16_t*)&message.data[4]) = year-yearOffset;
	message.data[6] = (unsigned char) month-monthOffset;
	message.data[7] = (unsigned char) day;

	bMessageOk = bMessageOk && WriteMessage(message);

	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // SET_PARAMETER
	message.data[1] = 0x21;    // SET_TIME

	message.data[2] = 0x00; // Address[0] : Not used
	message.data[3] = 0x00; // Address[1] : Not used
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

bool ReceiverCANCapture::StartPlayback(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask)
{
	AWLCANMessage message;
	
	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xD1;   // PLAYBACK_RAW
	message.data[1] = channelMask.byteData;   

	message.data[2] = 0x00; // Not used
	message.data[3] = frameRate; // Address[1] : Not used
	message.data[4] = 0x00; // Not used
	message.data[5] = 0x00; // Not used
	message.data[6] = 0x00; // Not used
	message.data[7] = 0x00; // Not used

	bool bMessageOk = WriteMessage(message);

	receiverStatus.bInPlayback = bMessageOk;
	return(bMessageOk);
}

bool ReceiverCANCapture::StartRecord(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask)
{
	AWLCANMessage message;
	
	// Write date
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xD0;   // Record_RAW
	message.data[1] = channelMask.byteData;   

	message.data[2] = 0x00; // Not used
	message.data[3] = frameRate; // Address[1] : Not used
	message.data[4] = 0x00; // Not used
	message.data[5] = 0x00; // Not used
	message.data[6] = 0x00; // Not used
	message.data[7] = 0x00; // Not used

	bool bMessageOk = WriteMessage(message);

	receiverStatus.bInRecord = bMessageOk;
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

	if (bMessageOk) receiverStatus.bInPlayback = false;
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

	return(bMessageOk);	receiverStatus.bInRecord = false;
	return(bMessageOk);
}

bool ReceiverCANCapture::StartCalibration(uint8_t frameQty, float beta, ReceiverCapture::ChannelMask channelMask)
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

	receiverStatus.bInRecord = bMessageOk;
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


#if 1 // Simulate rsponse for tests
	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;
	receiverStatus.bUpdated = true;
	rawLock.unlock();

#endif
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

#if 1 // Simulate rsponse for tests
	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.adcRegisterAddressRead = registerAddress;
	receiverStatus.adcRegisterValueRead = registerValue;
	receiverStatus.bUpdated = true;
	rawLock.unlock();

#endif
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
	return(bMessageOk);
}

#if 0
static double lastDistance = 0;
void ReceiverCANCapture::FakeChannelDistance(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}
		// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];

	if (channel >= 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		int elapsed = (int) GetElapsed();
		float distance = elapsed % 4000 ;
		distance /= 100;

		currentFrame->channelFrames[channel]->timeStamp = GetElapsed();
		if (distance < minDistance  || distance > 40) distance = 0.0;

		lastDistance = distance;

		int detectionIndex = 0+detectOffset;
		
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		// Only the first channel displays a distance
		distance += 5;
		if (distance < minDistance  || distance  > 40) distance = 0.0;
		detectionIndex = 1+detectOffset;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		
		distance += 5;

		if (distance < minDistance  || distance  > 40) distance = 0.0;
		detectionIndex = 2+detectOffset;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		distance += 5;
	
		if (distance < minDistance  || distance > 40) distance = 0.0;
		detectionIndex = 3+detectOffset;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;
		rawLock.unlock();
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
	}

}

#else
const float simulatedDistance1 = 20.0;
const float simulatedDistanced2 = 12.0;

const float maxSimulatedJitter = 0.9;
const float simulatedPresenceRatio = 0.3;
const int   maxSimulatedFalsePositives = 3;

void ReceiverCANCapture::FakeChannelDistance(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}
		// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];

	if (channel == 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		int elapsed = (int) GetElapsed();

		double distance = simulatedDistance1; 
		// Add jitter to the detection
		distance += ((maxSimulatedJitter * rand() / RAND_MAX)) - (maxSimulatedJitter/2);
		
		// Check if we should create a "false negative to the detection
		bool bIsPresent = (rand()*1.0 / RAND_MAX) < simulatedPresenceRatio;

		if (distance < minDistance  || distance  > maxDistance) distance = 0.0;

		int detectionIndex = 0+detectOffset;
		if (bIsPresent) 
		{
			currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
			currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
			currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;
			detectionIndex++;
		}

		int falsePositiveQty = ((maxSimulatedFalsePositives * rand() / RAND_MAX));
		for (int i = 0; i < falsePositiveQty; i++) 
		{
			// Simulate the distance between minDistance maxDistance
			distance = minDistance + ((maxDistance - minDistance) * rand() / RAND_MAX);
			currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
			currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
			currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

			detectionIndex++;
		}


		rawLock.unlock();
	}

	if (channel == 6 && detectOffset == 4)
	{
		DebugFilePrintf(outFile, "Fake");
		ProcessCompletedFrame();
	}

}
#endif
