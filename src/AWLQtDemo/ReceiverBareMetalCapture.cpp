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
#include "ReceiverBareMetalCapture.h"


using namespace std;
using namespace pcl;
using namespace awl;

const float maxIntensity = 1024.0;

boost::posix_time::ptime reconnectTime;

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

ReceiverBareMetalCapture::ReceiverBareMetalCapture(int inSequenceID, int inReceiverChannelQty, int inDetectionsPerChannel, int argc, char** argv):
ReceiverCapture(inSequenceID, inReceiverChannelQty, inDetectionsPerChannel),
port(NULL),
reader(NULL),
lastMessageID(0),
io(),
responseString("")


{
	// Update settings from application
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	sCommPort = globalSettings->sBareMetalCommPort.toStdString();
	serialPortRate = globalSettings->serialBareMetalPortRate;

    OpenDebugFile(outFile, "BareBusLog.dat");

	DebugFilePrintf(outFile, "StartProgram %d", 22);

    if (OpenBareMetalPort())
	{
		WriteCurrentDateTime();
	}

	acquisitionSequence->Clear();
}

ReceiverBareMetalCapture::~ReceiverBareMetalCapture()
{

    CloseBareMetalPort();
	CloseDebugFile(outFile);
	Stop(); // Stop the thread
}

bool  ReceiverBareMetalCapture::OpenBareMetalPort()

{
	reader = NULL;

	if (port) 
	{
        CloseBareMetalPort();
	}

    // initialize the Serial - USB port
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

	if (reader) 
		return(true);
	else {

		delete(port);
		port = NULL;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}

}

bool  ReceiverBareMetalCapture::CloseBareMetalPort()

{
		if (port) port->close();
		reader = NULL;
		port = NULL;
		return(true);
}

void  ReceiverBareMetalCapture::Go(bool inIsThreaded)
	
{
	bIsThreaded = inIsThreaded;
	assert(!mThread);
    mStopRequested = false;

	startTime = boost::posix_time::microsec_clock::local_time();


	if (bIsThreaded) 
	{
        mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverBareMetalCapture::DoThreadLoop, this)));
	}
}
 

void  ReceiverBareMetalCapture::Stop()
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


bool  ReceiverBareMetalCapture::WasStopped()
{
	if (mStopRequested) return(true);
	return(false);
}

void ReceiverBareMetalCapture::DoThreadLoop()

{

	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverBareMetalCapture::DoThreadIteration()

{
	if (!bIsThreaded)
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}



void ReceiverBareMetalCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
		lastMessageID = 0;
        AWLBareMessage msg;

        float distance;

		// If the simulated data mode is enabled,
		// inject distance information by simulating receiver data.
		if (bSimulatedDataEnabled && ((GetElapsed() - lastElapsed) >= 10)) 
		{
			if (injectType == eInjectRamp)
			{
				FakeChannelDistanceRamp(20);
				FakeChannelDistanceRamp(21);
				FakeChannelDistanceRamp(22);
				FakeChannelDistanceRamp(23);
				FakeChannelDistanceRamp(24);
				FakeChannelDistanceRamp(36);
			}
			else if (injectType == eInjectNoisy)
			{
				FakeChannelDistanceNoisy(20);
				FakeChannelDistanceNoisy(21);
				FakeChannelDistanceNoisy(22);
				FakeChannelDistanceNoisy(23);
				FakeChannelDistanceNoisy(24);
				FakeChannelDistanceNoisy(36);
			}
			else if (injectType == eInjectSlowMove)
			{
				FakeChannelDistanceSlowMove(20);
				FakeChannelDistanceSlowMove(21);
				FakeChannelDistanceSlowMove(22);
				FakeChannelDistanceSlowMove(23);
				FakeChannelDistanceSlowMove(24);
				FakeChannelDistanceSlowMove(25);
				FakeChannelDistanceSlowMove(36);
			}
			else 
			{
				FakeChannelDistanceConstant(20);
				FakeChannelDistanceConstant(21);
				FakeChannelDistanceConstant(22);
				FakeChannelDistanceConstant(23);
				FakeChannelDistanceConstant(24);
				FakeChannelDistanceConstant(36);
			}


			lastElapsed = GetElapsed();
		}

		if (port && reader && port->is_open()) 
		{
			char c;
			// read from the serial port until we get a#
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
                DebugFilePrintf(outFile,  "Time Outon read_char.  Resetting Bare Port");
                CloseBareMetalPort();
				if (OpenBareMetalPort())
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
                DebugFilePrintf(outFile,  "Reconmnecting Bare Port");
				if (OpenBareMetalPort())
				{
					WriteCurrentDateTime();
				}
			}

		}

	} // if  (!WasStoppped)


}

bool ReceiverBareMetalCapture::GetDataByte(std::string &inResponse, uint8_t &outByte, int startIndex, int len)

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

bool ReceiverBareMetalCapture::GetStandardID(std::string &inResponse,  unsigned long &outID, int startIndex)

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

bool ReceiverBareMetalCapture::ParseLine(std::string inResponse, AWLBareMessage &outMsg)
{
	bool bResult = false;
	if (inResponse.length() < 2) 
	{
        DebugFilePrintf(outFile, "BareLine empty");
		return bResult;
	}
    if (inResponse[0] != 'R')
    {
        DebugFilePrintf(outFile, "BareLine bad: %s\n", inResponse.c_str());
        return bResult;
   }

	
    sscanf(inResponse.c_str(), "R %d C %d M %d V %d", 
                       &outMsg.frameID,
                       &outMsg.channelID,
                       &outMsg.peakIndex,
                       &outMsg.intensity);

	bResult = true;
	return(bResult);
}


void ReceiverBareMetalCapture::ParseMessage(AWLBareMessage &inMsg)

{
	unsigned long msgID = inMsg.id;
    ParseChannelDistance(inMsg);
    ParseChannelIntensity(inMsg);

    if (inMsg.channelID == 7)
	{
		ProcessCompletedFrame();
	}
 }


void ReceiverBareMetalCapture::ParseSensorStatus(AWLBareMessage &inMsg)

{

}

void ReceiverBareMetalCapture::ParseSensorBoot(AWLBareMessage &inMsg)

{

}

#if 0
static int channelReorder[] = {0, 1, 2, 3, 4, 5, 6};
//static int channelReorder[] = {0, 1, 2, 3, -1, -1, -1};
#endif

void ReceiverBareMetalCapture::ParseChannelDistance(AWLBareMessage &inMsg)

{
    int channel = inMsg.channelID;
#if 0
    // JYD:  Watch out ---- Channel order can be patched here, because of FPGA bug
	channel = channelReorder[channel];
#endif
    float distance(0.0);

	if (channel >= 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
        distance = inMsg.peakIndex;
        distance *= .375; // meters per sample;
		distance += measurementOffset;
		distance += sensorDepth;

		currentFrame->channelFrames[channel]->timeStamp = GetElapsed();
		if (distance < minDistance  || distance > maxDistance) distance = 0.0;

        int detectionIndex = 0;
		
        currentFrame->channelFrames[channel]->detections[detectionIndex]->distance = distance;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->firstTimeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->timeStamp = currentFrame->GetFrameID();
		currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
		currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

		rawLock.unlock();
	}
    DebugFilePrintf(outFile, "Msg distance - Val %d %f",inMsg.peakIndex, distance);
}


void ReceiverBareMetalCapture::ParseChannelIntensity(AWLBareMessage &inMsg)

{
    int channel = inMsg.channelID;
#if 0
    // JYD:  Watch out ---- Channel order can be patched here, because of FPGA bug
    channel = channelReorder[channel];
#endif
	int  intensity(0);

    if (channel >= 0)
    {

        boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
        intensity = inMsg.intensity;

        currentFrame->channelFrames[channel]->timeStamp = GetElapsed();
        int detectionIndex = 0;

        currentFrame->channelFrames[channel]->detections[detectionIndex]->intensity = intensity;
        currentFrame->channelFrames[channel]->detections[detectionIndex]->firstTimeStamp = currentFrame->GetFrameID();
        currentFrame->channelFrames[channel]->detections[detectionIndex]->timeStamp = currentFrame->GetFrameID();
        currentFrame->channelFrames[channel]->detections[detectionIndex]->trackID = 0;
        currentFrame->channelFrames[channel]->detections[detectionIndex]->velocity = 0;

        rawLock.unlock();
    }

    DebugFilePrintf(outFile, "Msg Intensity - Val %d %d",inMsg.intensity, intensity);
}

void ReceiverBareMetalCapture::ParseControlMessage(AWLBareMessage &inMsg)
{
#if 0
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
#endif
}

void ReceiverBareMetalCapture::ParseParameterSet(AWLBareMessage &inMsg)
{
	DebugFilePrintf(outFile, "Error: Command - Parameter - Set received (%x).  Message skipped. ");
}


void ReceiverBareMetalCapture::ParseParameterQuery(AWLBareMessage &inMsg)
{
	DebugFilePrintf(outFile, "Error: Command - Parameter - Set received (%x).  Message skipped. ");
}

void ReceiverBareMetalCapture::ParseParameterResponse(AWLBareMessage &inMsg)
{
#if 0
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
#endif
}


void ReceiverBareMetalCapture::ParseParameterError(AWLBareMessage &inMsg)
{
#if 0
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
#endif
}


void ReceiverBareMetalCapture::ParseParameterAlgoSelectResponse(AWLBareMessage &inMsg)
{

	receiverStatus.currentAlgo = inMsg.address;
	receiverStatus.currentAlgoPendingUpdates--;
}

void ReceiverBareMetalCapture::ParseParameterAlgoParameterResponse(AWLBareMessage &inMsg)
{

	uint16_t registerAddress = inMsg.address;
	uint32_t registerValue=  inMsg.value;

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	QList<AlgorithmParameters> algoParameters = globalSettings->parametersAlgos[receiverStatus.currentAlgo];
	int index = globalSettings->FindAlgoParamByAddress(globalSettings->parametersAlgos[receiverStatus.currentAlgo],
		                                           registerAddress);


	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;

	if (index >= 0)
	{
		globalSettings->parametersAlgos[receiverStatus.currentAlgo][index].floatValue = *(float *) &registerValue; 
		globalSettings->parametersAlgos[receiverStatus.currentAlgo][index].intValue = *(int16_t *) &registerValue; 
		globalSettings->parametersAlgos[receiverStatus.currentAlgo][index].pendingUpdates--;
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverBareMetalCapture::ParseParameterFPGARegisterResponse(AWLBareMessage &inMsg)
{
    uint16_t registerAddress = *(uint16_t *) &inMsg.address;
    uint32_t registerValue=  *(uint32_t *) &inMsg.value;

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterFPGAByAddress(registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;

	if (index >= 0)
	{
		globalSettings->registersFPGA[index].value = registerValue; 
		if (globalSettings->registersFPGA[index].pendingUpdates > 0)
		{
			globalSettings->registersFPGA[index].pendingUpdates--;
		}
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverBareMetalCapture::ParseParameterBiasResponse(AWLBareMessage &inMsg)
{
	// Message not used. We ignore the message for the moment.
}

void ReceiverBareMetalCapture::ParseParameterADCRegisterResponse(AWLBareMessage &inMsg)
{
    uint16_t registerAddress = *(uint16_t *) &inMsg.address;
    uint32_t registerValue=  *(uint32_t *) &inMsg.value;

    AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterADCByAddress(registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.adcRegisterAddressRead = registerAddress;
	receiverStatus.adcRegisterValueRead = registerValue;

	if (index >= 0)
	{
		globalSettings->registersADC[index].value = registerValue; 
		if (globalSettings->registersADC[index].pendingUpdates > 0)
		{
			globalSettings->registersADC[index].pendingUpdates--;
		}
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}


void ReceiverBareMetalCapture::ParseParameterPresetResponse(AWLBareMessage &inMsg)
{
	// Message not used. We ignore the message for the moment.
}

void ReceiverBareMetalCapture::ParseParameterGlobalParameterResponse(AWLBareMessage &inMsg)
{
	uint16_t registerAddress = inMsg.address;
	uint32_t registerValue=  inMsg.value;
	int globalAlgo = 0;  // PlaceHolder, so we know which algo pointer to use.

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	QList<AlgorithmParameters> algoParameters = globalSettings->parametersAlgos[globalAlgo];
	int index = globalSettings->FindAlgoParamByAddress(globalSettings->parametersAlgos[globalAlgo],
		                                           registerAddress);


	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.fpgaRegisterAddressRead = registerAddress;
	receiverStatus.fpgaRegisterValueRead = registerValue;

	if (index >= 0)
	{
		globalSettings->parametersAlgos[globalAlgo][index].floatValue = *(float *) &registerValue; 
		globalSettings->parametersAlgos[globalAlgo][index].intValue = *(int16_t *) &registerValue; 
		globalSettings->parametersAlgos[globalAlgo][index].pendingUpdates--;
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverBareMetalCapture::ParseParameterGPIORegisterResponse(AWLBareMessage &inMsg)
{
    uint16_t registerAddress = *(uint16_t *) &inMsg.address;
    uint32_t registerValue=  *(uint32_t *) &inMsg.value;

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterGPIOByAddress(registerAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.gpioRegisterAddressRead = registerAddress;
	receiverStatus.gpioRegisterValueRead = registerValue;

	if (index >= 0)
	{
		globalSettings->registersGPIO[index].value = registerValue; 
		if (globalSettings->registersGPIO[index].pendingUpdates > 0)
		{
			globalSettings->registersGPIO[index].pendingUpdates--;
		}
	}


	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverBareMetalCapture::ParseParameterDateTimeResponse(AWLBareMessage &inMsg)
{
	// Message should be sent as a response when we change the date.
	// Otherwise it is not used. We ignore the message for the moment.
}

void ReceiverBareMetalCapture::ParseParameterRecordResponse(AWLBareMessage &inMsg)
{
}

void ReceiverBareMetalCapture::ParseParameterPlaybackResponse(AWLBareMessage &inMsg)
{
}

void ReceiverBareMetalCapture::ParseParameterAlgoSelectError(AWLBareMessage &inMsg)
{
	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error");
}

void ReceiverBareMetalCapture::ParseParameterAlgoParameterError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");}

void ReceiverBareMetalCapture::ParseParameterFPGARegisterError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::ParseParameterBiasError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.  Type %x");
}

void ReceiverBareMetalCapture::ParseParameterADCRegisterError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::ParseParameterPresetError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.!");
}

void ReceiverBareMetalCapture::ParseParameterGlobalParameterError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::ParseParameterGPIORegisterError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::ParseParameterDateTimeError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::ParseParameterRecordError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::ParseParameterPlaybackError(AWLBareMessage &inMsg)
{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	receiverStatus.bUpdated = true;
    receiverStatus.lastCommandError = 0;
	rawLock.unlock();
    DebugFilePrintf("Control command error.");
}

void ReceiverBareMetalCapture::WriteString(std::string inString)
{
	if (!port) return;

	int stringSize = inString.size();
	boost::asio::write(*port,boost::asio::buffer(inString.c_str(),stringSize));
}

bool ReceiverBareMetalCapture::WriteMessage(const AWLBareMessage &inMsg)
{
    std::string outResponse("");
	char outString[5];

	if (!port) return(false);

    sprintf(outString, "%x %x %x %lx", inMsg.command, inMsg.address, inMsg.value);
	outResponse +=outString;
	outResponse += "\r";

	WriteString(outResponse);
	return(true);
}




bool ReceiverBareMetalCapture::WriteCurrentDateTime()
{
#if 1
    return(true);
#else
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

    AWLBareMessage message;
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
#endif
} 

const int nameBlockSize = 6;

bool ReceiverBareMetalCapture::SetPlaybackFileName(std::string inPlaybackFileName)
{
	receiverStatus.sPlaybackFileName = inPlaybackFileName;

	bool bMessageOk(true);
#if 0
    int nameLength = inPlaybackFileName.length();

	
	// Write name strings in block of 6 characters.
	// terminating NULL is end of message.
	for (int blockOffset = 0; blockOffset < nameLength+1; blockOffset += nameBlockSize)
	{
        AWLBareMessage message;
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

#endif
    if (bMessageOk) receiverStatus.sPlaybackFileName = inPlaybackFileName;

	return(bMessageOk);
}


bool ReceiverBareMetalCapture::SetRecordFileName(std::string inRecordFileName)
{
	bool bMessageOk(true);
#if 0
    int nameLength = inRecordFileName.length();

	// Write name strings in block of 6 characters.
	// terminating NULL is end of message.
	for (int blockOffset = 0; blockOffset < nameLength+1; blockOffset += nameBlockSize)
	{
        AWLBareMessage message;
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
#endif

	if (bMessageOk) receiverStatus.sRecordFileName = inRecordFileName;

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::StartPlayback(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask)
{
#if 0
    AWLBareMessage message;
	
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
#else
    bool bMessageOk(true);
#endif
	receiverStatus.bInPlayback = bMessageOk;
	if (frameRate > 0) receiverStatus.frameRate = frameRate;
	return(bMessageOk);
}

bool ReceiverBareMetalCapture::StartRecord(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask)
{
 #if 0
    AWLBareMessage message;
	
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
#else
    bool bMessageOk = true;
#endif
	receiverStatus.bInRecord = bMessageOk;
	if (frameRate > 0) receiverStatus.frameRate = frameRate;

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::StopPlayback()
{
#if 0
    AWLBareMessage message;
	
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
#else
    bool bMessageOk(true);
#endif
	if (bMessageOk)
	{
		receiverStatus.bInPlayback = false;
		receiverStatus.bInRecord = false;
	}

	return(bMessageOk);
}
	
bool ReceiverBareMetalCapture::StopRecord()
{
#if 0
        AWLBareMessage message;
	
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
#else
    bool bMessageOk(true);
#endif
    if (bMessageOk)
	{
		receiverStatus.bInPlayback = false;
		receiverStatus.bInRecord = false;
	}
	return(bMessageOk);
}

bool ReceiverBareMetalCapture::StartCalibration(uint8_t frameQty, float beta, ReceiverCapture::ChannelMask channelMask)
{
#if 0
    AWLBareMessage message;
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xDA;   // Record_Calibration
	message.data[1] = channelMask.byteData;   

	message.data[2] = frameQty; // Number of frames
	message.data[3] = 0; // Not used
	*((float *) &message.data[4]) = beta;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk(true);
#endif

	receiverStatus.bInRecord = bMessageOk;
	return(bMessageOk);
}

bool ReceiverBareMetalCapture::SetAlgorithm(uint16_t algorithmID)
{

    AWLBareMessage message;

#if 0
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x01; // Algo-select  

	* (int16_t *) &message.data[2] = algorithmID;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk(true);
#endif

	// Signal that we are waiting for an update of the register settings.
	
	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentAlgo = algorithmID;
	receiverStatus.currentAlgoPendingUpdates = 1;

    // Simulate rsponse for tests
     message.address = algorithmID;
     message.value = 0;
     ParseParameterAlgoSelectResponse(message);

   return(bMessageOk);
}


bool ReceiverBareMetalCapture::SetFPGARegister(uint16_t registerAddress, uint32_t registerValue)
{
    AWLBareMessage message;
 #if 0
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x03; // AWL_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk(true);
#endif

	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterFPGAByAddress(registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		globalSettings->registersFPGA[index].pendingUpdates = 1;
	}

    // Simulate rsponse for tests
     message.address = registerAddress;
     message.value = registerValue;
     ParseParameterFPGARegisterResponse(message);

   return(bMessageOk);
}

bool ReceiverBareMetalCapture::SetADCRegister(uint16_t registerAddress, uint32_t registerValue)
{
    AWLBareMessage message;
 #if 0
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x05; // ADC_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk = true;
#endif
	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterADCByAddress(registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		globalSettings->registersADC[index].pendingUpdates = 1;
	}


    // Simulate rsponse for tests
     message.address = registerAddress;
     message.value = registerValue;
     ParseParameterADCRegisterResponse(message);

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::SetGPIORegister(uint16_t registerAddress, uint32_t registerValue)
{
    AWLBareMessage message;
 #if 0

	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x08; // GPIO_Control  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk(true);
#endif
	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterGPIOByAddress(registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		globalSettings->registersGPIO[index].pendingUpdates = 1;
	}



    // Simulate rsponse for tests
     message.address = registerAddress;
     message.value = registerValue;
     ParseParameterGPIORegisterResponse(message);

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::SetAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress, uint32_t registerValue)
{
    AWLBareMessage message;
#if 0
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x02; // ALGO_PARAMETER 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);
 #else
    bool bMessageOk(true);
 #endif

	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings-> FindAlgoParamByAddress(parametersList, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parametersList[index].pendingUpdates = 1;
	}


    // Simulate rsponse for tests
     message.address = registerAddress;
     message.value = registerValue;
     ParseParameterAlgoParameterResponse(message);

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::SetGlobalAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress, uint32_t registerValue)
{
    AWLBareMessage message;
#if 0
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC0;   // Set Parameter
	message.data[1] = 0x07; // GLOBAL_PARAMETER 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	bool bMessageOk = WriteMessage(message);
 #else
    bool bMessageOk(true);
 #endif

	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings-> FindAlgoParamByAddress(parametersList, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parametersList[index].pendingUpdates = 1;
	}

     // Simulate rsponse for tests
     message.address = registerAddress;
     message.value = registerValue;
     ParseParameterGlobalParameterResponse(message);


	return(bMessageOk);
}

bool ReceiverBareMetalCapture::QueryAlgorithm()
{
    AWLBareMessage message;
 #if 0
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x01; // Algo-select  

	* (int16_t *) &message.data[2] = 0L;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk(true);
#endif
	// Signal that we are waiting for an update of the register settings.
	
	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentAlgoPendingUpdates = 1;

    // Simulate rsponse for tests
    ParseParameterAlgoSelectResponse(message);

   return(bMessageOk);
}


bool ReceiverBareMetalCapture::QueryFPGARegister(uint16_t registerAddress)
{
    AWLBareMessage message;
 #if 0
	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x03; // AWL_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
    bool bMessageOk(true);
#endif
	// Signal that we are waiting for an update of the register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterFPGAByAddress(registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		globalSettings->registersFPGA[index].pendingUpdates = 1;
	}

        // Simulate rsponse for tests
         ParseParameterFPGARegisterResponse(message);


	return(bMessageOk);
}

bool ReceiverBareMetalCapture::QueryADCRegister(uint16_t registerAddress)
{
    AWLBareMessage message;
#if 0	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x05; // ADC_Register  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
	bool bMessageOk(true);
#endif

	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterADCByAddress(registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		globalSettings->registersADC[index].pendingUpdates = 1;
	}

        // Simulate rsponse for tests
         ParseParameterADCRegisterResponse(message);

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::QueryGPIORegister(uint16_t registerAddress)
{
    AWLBareMessage message;
#if 0	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x08; // GPIO Control  

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
	bool bMessageOk(true);
#endif
	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindRegisterGPIOByAddress(registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		globalSettings->registersGPIO[index].pendingUpdates = 1;
	}

        // Simulate rsponse for tests
         ParseParameterGPIORegisterResponse(message);

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::QueryAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress)
{
    AWLBareMessage message;
#if 0	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x02; // Algorithm parameter 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
	bool bMessageOk(true);
#endif
	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindAlgoParamByAddress(parametersList, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parametersList[index].pendingUpdates = 1;
	}

        // Simulate rsponse for tests
         ParseParameterAlgoParameterResponse(message);

	return(bMessageOk);
}

bool ReceiverBareMetalCapture::QueryGlobalAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress)
{
    AWLBareMessage message;
#if 0	
	message.id = 80;       // Message id: 80- Command message

    message.len = 8;       // Frame size (0.8)
    message.data[0] = 0xC1;   // Query Parameter
	message.data[1] = 0x07; // Global parameter 

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);
#else
	bool bMessageOk(true);
#endif
	// Signal that we are waiting for an update of thet register settings.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int index = globalSettings->FindAlgoParamByAddress(parametersList, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parametersList[index].pendingUpdates = 1;
	}

    // Simulate rsponse for tests
     ParseParameterGlobalParameterResponse(message);

	return(bMessageOk);
}


