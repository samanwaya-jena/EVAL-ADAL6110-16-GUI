/* ReceiverCANCapture.cpp */
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
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#include "DebugPrintf.h"

#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "ReceiverCANCapture.h"


using namespace std;
using namespace awl;

const float maxIntensity = 1024.0;

const uint16_t		defaultYearOffset = 1900;		// On AWL, all CAN Dates are sent as offsets from 1900 
const uint16_t		defaultMonthOffset = 1;		// On AWL, all CAN months start at 0.  Posix months start at 1.
const ReceiverCANCapture::eReceiverCANRate defaultCANRate = ReceiverCANCapture::canRate1Mbps;

#define ConvertIntensityToSNR(v) (((v)/2.0) - 21.0)


ReceiverCANCapture::ReceiverCANCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
					   eReceiverCANRate inCANRate, int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, 
					   const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers):
ReceiverCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, inFrameRate, inChannelMask, inMessageMask, inRangeOffset, 
                inRegistersFPGA, inRegistersADC, inRegistersGPIO, inParametersAlgos, inParametersTrackers),
canRate(inCANRate),
sampleCount(0), max_msg_id(0x80), max_channel(0),
closeCANReentryCount(0)


{
#ifdef FORCE_FRAME_RESYNC_PATCH
	lastChannelMask.byteData = 0;
	lastChannelID = 0;
#endif
	yearOffset = defaultYearOffset;
	monthOffset = defaultMonthOffset;

	for (int i = 0; i < maxRawBufferCount; i++) {
		rawBuffers[i] = 0;
	}

	OpenDebugFile(debugFile, "CanBusLog.dat");

	DebugFilePrintf(debugFile, "StartProgram %d", 22);
}


ReceiverCANCapture::ReceiverCANCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCapture(receiverID, propTree),
sampleCount(0), max_msg_id(0x80), max_channel(0),
closeCANReentryCount(0)

{
#ifdef FORCE_FRAME_RESYNC_PATCH
	lastChannelMask.byteData = 0;
	lastChannelID = 0;
#endif

	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);

	// Default values that are not in the configuration file anymore
	yearOffset = defaultYearOffset;
	monthOffset = defaultMonthOffset;

	for (int i = 0; i < maxRawBufferCount; i++) {
		rawBuffers[i] = 0;
	}

	OpenDebugFile(debugFile, "CanBusLog.dat");

	DebugFilePrintf(debugFile, "StartProgram %d", 22);
}

ReceiverCANCapture::~ReceiverCANCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
	for (int i = 0; i < maxRawBufferCount; i++) {
		if (rawBuffers [i]) delete rawBuffers [i];
	}
}

void  ReceiverCANCapture::Go() 
	
{
 	assert(!mThread);

	
	if (OpenCANPort())
	{
		WriteCurrentDateTime();
		SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
		// Update all the info (eventually) from the status of the machine
		QueryAlgorithm();
		QueryTracker();
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

void ReceiverCANCapture::ParseMessage(AWLCANMessage &inMsg)

{
	unsigned long msgID = inMsg.id;
#ifdef FORCE_FRAME_RESYNC_PATCH
	ForceFrameResync(inMsg);
#endif //FORCE_FRAME_RESYNC_PATCH

	//printf ("ParseMessage %02x \n", msgID);
	if (msgID == AWLCANMSG_ID_SENSORSTATUS)
	{
		ParseSensorStatus(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_SENSORBOOT)
	{
		ParseSensorBoot(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_COMPLETEDFRAME)
	{
		ProcessCompletedFrame();
	}
	else if (msgID == AWLCANMSG_ID_OBSTACLETRACK)
	{
		ParseObstacleTrack(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_OBSTACLEVELOCITY)
	{
		ParseObstacleVelocity(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_OBSTACLESIZE)
	{
		ParseObstacleSize(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_OBSTACLEANGULARPOSITION)
	{
		ParseObstacleAngularPosition(inMsg);
	}
	else if (msgID >= AWLCANMSG_ID_CHANNELDISTANCE1_FIRST && msgID <= AWLCANMSG_ID_CHANNELDISTANCE1_LAST)
	{
		ParseChannelDistance(inMsg);
	}
	else if (msgID >= AWLCANMSG_ID_CHANNELDISTANCE2_FIRST && msgID <= AWLCANMSG_ID_CHANNELDISTANCE2_LAST)
	{
		ParseChannelDistance(inMsg);
	}
	else if (msgID >= AWLCANMSG_ID_CHANNELINTENSITY1_FIRST && msgID <= AWLCANMSG_ID_CHANNELINTENSITY1_LAST)
	{
		ParseChannelIntensity(inMsg);
	}
	else if (msgID >= AWLCANMSG_ID_CHANNELINTENSITY2_FIRST && msgID <= AWLCANMSG_ID_CHANNELINTENSITY2_LAST)
	{
		ParseChannelIntensity(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_CHANNELDISTANCEANDINTENSITY)
	{
		ParseChannelDistanceAndIntensity(inMsg);
	}
	else if (msgID == AWLCANMSG_ID_COMMANDMESSAGE) /* Command */
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

	if (inMsg.id != AWLCANMSG_ID_SENSORSTATUS) return;

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

	DebugFilePrintf(debugFile, "Msg %lu - Val %u %d %u %u %u %u", inMsg.id, 
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

	if (inMsg.id != AWLCANMSG_ID_SENSORBOOT) return;

	boost::mutex::scoped_lock rawLock(GetMutex());

	receiverStatus.version.major =  byteDataPtr[0];
	receiverStatus.version.minor =  byteDataPtr[1];

	receiverStatus.bootChecksumError.byteData = byteDataPtr[2];
	receiverStatus.bootSelfTest.byteData = byteDataPtr[3];
	receiverStatus.bUpdated = true;

	rawLock.unlock();

	DebugFilePrintf(debugFile, "Msg %lu - Val %u %u %u %u", inMsg.id, 
			receiverStatus.version.major,
			receiverStatus.version.minor,
			receiverStatus.bootChecksumError.byteData,
			receiverStatus.receiverError.byteData,
			receiverStatus.bootSelfTest.byteData);
}

void ReceiverCANCapture::ParseChannelDistance(AWLCANMessage &inMsg)

{
	int channel;
	int detectOffset = 0;
	uint16_t *distancePtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= AWLCANMSG_ID_CHANNELDISTANCE2_FIRST)
	{
		channel = inMsg.id - AWLCANMSG_ID_CHANNELDISTANCE2_FIRST;
		detectOffset = 4;
	}
	else 
	{
		channel = inMsg.id - AWLCANMSG_ID_CHANNELDISTANCE1_FIRST;
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
	DebugFilePrintf(debugFile, "Msg %lu - Val %d %d %d %d", inMsg.id, distancePtr[0], distancePtr[1], distancePtr[2], distancePtr[3]);
}

void ReceiverCANCapture::ParseChannelIntensity(AWLCANMessage &inMsg)

{
	int channel;
	int detectOffset = 0;
	uint16_t *intensityPtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= AWLCANMSG_ID_CHANNELINTENSITY2_FIRST)
	{
		channel = inMsg.id - AWLCANMSG_ID_CHANNELINTENSITY2_FIRST;
		detectOffset = 4;
	}
	else 
	{
		channel = inMsg.id - AWLCANMSG_ID_CHANNELINTENSITY1_FIRST;
	}


	if (channel >= 0)
	{
		boost::mutex::scoped_lock rawLock(GetMutex());

		float intensity = ((float)intensityPtr[0]) / maxIntensity;
		int detectionIndex = 0 + detectOffset;
		Detection::Ptr detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		detection->intensity = ConvertIntensityToSNR(intensity);
		detection->trackID = 0;
		detection->velocity = 0;

		intensity = ((float)intensityPtr[1]) / maxIntensity;
		detectionIndex = 1 + detectOffset;
		detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		detection->intensity = ConvertIntensityToSNR(intensity);
		detection->trackID = 0;
		detection->velocity = 0;

		intensity = ((float)intensityPtr[2]) / maxIntensity;
		detectionIndex = 2 + detectOffset;
		detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		detection->intensity = ConvertIntensityToSNR(intensity);
		detection->trackID = 0;
		detection->velocity = 0;

		intensity = ((float)intensityPtr[3]) / maxIntensity;
		detectionIndex = 3 + detectOffset;
		detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);
		detection->intensity = ConvertIntensityToSNR(intensity);
		detection->trackID = 0;
		detection->velocity = 0;
		rawLock.unlock();
	}

	DebugFilePrintf(debugFile, "Msg %lu - Val %d %d %d %d", inMsg.id, intensityPtr[0], intensityPtr[1], intensityPtr[2], intensityPtr[3]);
}

void ReceiverCANCapture::ParseChannelDistanceAndIntensity(AWLCANMessage &inMsg)

{
	int channel;
	uint16_t *dataPtr = (uint16_t *)inMsg.data;

	channel = dataPtr[0];

	if (channel >= 0)
	{
		boost::mutex::scoped_lock rawLock(GetMutex());

		float distance = (float)(dataPtr[1]);
		distance /= 100;
		distance += measurementOffset;

		float intensity = ((float)dataPtr[2]) / maxIntensity;

		int detectionIndex = (int)dataPtr[3];
		Detection::Ptr detection = currentFrame->MakeUniqueDetection(currentFrame->rawDetections, channel, detectionIndex);

		detection->distance = distance;
		detection->intensity = ConvertIntensityToSNR(intensity);

		detection->firstTimeStamp = currentFrame->timeStamp;
		detection->timeStamp = currentFrame->timeStamp;

		detection->trackID = 0;
		detection->velocity = 0;


		rawLock.unlock();
	}

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %lu - Val %d %d %d", inMsg.id, dataPtr[0], dataPtr[1], dataPtr[2]);
}

#define PATCH_CHANNEL_REORDER 0

#if PATCH_CHANNEL_REORDER

// Patch because LiBUSB Sensor sends channels ou of order
int channelReorder[16] = { 14, 12, 10, 8, 6, 4, 2, 0, 15, 13, 11, 9, 7, 5, 3, 1 };



#endif

void ReceiverCANCapture::ParseObstacleTrack(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID = *(uint16_t *)&inMsg.data[0];

	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);

	track->firstTimeStamp = currentFrame->timeStamp;
	track->timeStamp = currentFrame->timeStamp;

	track->trackChannels.byteData = *(uint8_t *)&inMsg.data[2];
	track->trackMainChannel = *(uint16_t *)&inMsg.data[3];
	uint16_t originalChannel = track->trackMainChannel;
#if PATCH_CHANNEL_REORDER
	track->trackMainChannel = channelReorder[track->trackMainChannel];

#endif


	// Compatibility patch: AWL-7 sends byte data only, but only one channel per channelMask.  
	// Other versions send trackMainChannel. For AWL-7 track main channel is 0.
	// Rebuild trackMainChannel from AWL 7 data.
	if (this->receiverChannelQty == 7)
	{
		track->trackMainChannel = 0;
		uint8_t channelMask = 0x01;
		for (int channel = 0; channel < 8; channel++)
		{
			if (track->trackChannels.byteData & channelMask)
			{
				track->trackMainChannel = channel;
				break;
			}
			channelMask <<= 1;
		}
	}

	// Decode rest of message
	track->probability = *(uint8_t *)&inMsg.data[5];

	uint16_t intensity = (*(uint16_t *)&inMsg.data[6]);
	track->intensity = ConvertIntensityToSNR(intensity);

	track->part1Entered = true;

	rawLock.unlock();
	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %lu - Track %u Val %x %d %f %f", inMsg.id, track->trackID, track->trackChannels, track->trackMainChannel, track->probability, track->intensity);
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
	DebugFilePrintf(debugFile, "Msg %lu - Track %u Val %f %f %f", (unsigned long)inMsg.id, (unsigned int) track->trackID, (float) track->distance, (float) track->velocity, (float)track->acceleration);
}


void ReceiverCANCapture::ParseObstacleSize(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID =  *(uint16_t *) &inMsg.data[0];
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);

	uint16_t height  = (*(uint16_t *) &inMsg.data[2]);  // Not transmitted. Should be 0.
	uint16_t width = (*(uint16_t *) &inMsg.data[4]);    // Not transmitted. Should be 0.
	uint16_t intensity = (*(uint16_t *) &inMsg.data[6]); // Also transmitted as  part of obstacle track.

	track->intensity = ConvertIntensityToSNR(intensity);

	track->part3Entered = true;
#if 0
	// Track is invalidated if intensity is invalid
	if (track->intensity < receiverStatus.signalToNoiseFloor)
	{
		track->part3Entered = false;
	}
#endif
	rawLock.unlock();

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %lu - Track %u Val %u %u %u", inMsg.id, track->trackID, intensity, height, width);
}

void ReceiverCANCapture::ParseObstacleAngularPosition(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	uint16_t trackID =  *(uint16_t *) &inMsg.data[0];
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackID);

	uint16_t startAngle  = (*(uint16_t *) &inMsg.data[2]); // Not transmitted. Should be 0.
	uint16_t endAngle = (*(uint16_t *) &inMsg.data[4]); // Not transmitted. Should be 0.
	uint16_t angularVelocity = (*(uint16_t *) &inMsg.data[6]); // Not transmitted. Should be 0.


	// Nothing done with this message (now deprecated), just log that we have received it.
	track->part4Entered = true;

	rawLock.unlock();

	// Debug and Log messages
	DebugFilePrintf(debugFile, "Msg %lu - Track %u Val %u %u %u", inMsg.id, track->trackID, startAngle, endAngle, angularVelocity);
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
0x07 = GLOBAL_PARAMETER
0x08 = GPIO_CONTROL
0x11 = TRACKER_SELECTED
0x12 = TRACKER_PARAMETER
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

void ReceiverCANCapture::ParseControlMessage(AWLCANMessage &inMsg)
{
	unsigned char  commandID = inMsg.data[0];

	switch(commandID)
	{
	case AWLCANMSG_ID_CMD_SET_PARAMETER:
		ParseParameterSet(inMsg);
		break;
	case AWLCANMSG_ID_CMD_QUERY_PARAMETER:
		ParseParameterQuery(inMsg);
		break;
	case AWLCANMSG_ID_CMD_RESPONSE_PARAMETER:
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
	case AWLCANMSG_ID_CMD_PARAM_ALGO_SELECTED:
		ParseParameterAlgoSelectResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_ALGO_PARAMETER:
		ParseParameterAlgoParameterResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_AWL_REGISTER:
		ParseParameterFPGARegisterResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_BIAS:
		ParseParameterBiasResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_ADC_REGISTER:
		ParseParameterADCRegisterResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_PRESET:
		ParseParameterPresetResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_GLOBAL_PARAMETER:
		ParseParameterGlobalParameterResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_GPIO_CONTROL:
		ParseParameterGPIORegisterResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_TRACKER_SELECTED:
		ParseParameterTrackerSelectResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_TRACKER_PARAMETER:
		ParseParameterTrackerParameterResponse(inMsg);
		break;	
	case AWLCANMSG_ID_CMD_PARAM_DATE_TIME:
		ParseParameterDateTimeResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_RECORD_FILENAME:
		ParseParameterRecordResponse(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_PLAYBACK_FILENAME:
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
	case AWLCANMSG_ID_CMD_PARAM_ALGO_SELECTED:
		ParseParameterAlgoSelectError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_ALGO_PARAMETER:
		ParseParameterAlgoParameterError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_AWL_REGISTER:
		ParseParameterFPGARegisterError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_BIAS:
		ParseParameterBiasError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_ADC_REGISTER:
		ParseParameterADCRegisterError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_PRESET:
		ParseParameterPresetError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_GLOBAL_PARAMETER:
		ParseParameterGlobalParameterError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_GPIO_CONTROL:
		ParseParameterGPIORegisterError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_TRACKER_SELECTED:
		ParseParameterTrackerSelectError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_TRACKER_PARAMETER:
		ParseParameterTrackerParameterError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_DATE_TIME:
		ParseParameterDateTimeError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_RECORD_FILENAME:
		ParseParameterRecordError(inMsg);
		break;
	case AWLCANMSG_ID_CMD_PARAM_PLAYBACK_FILENAME:
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
	
	AlgorithmDescription *algoDescription = NULL;
	algoDescription = FindAlgoDescriptionByID(parametersAlgos, registerValue);

	// Check that the algorithm is valid (just in case communication goes crazy)
	// Algo canot be 0, because 0 is reserved for GLOBAL_ALGO_ID 
	if (registerValue >= 1 && (algoDescription != NULL))  
	{
		receiverStatus.currentAlgo = registerValue;
		receiverStatus.currentAlgoPendingUpdates --;
	}
	else
	{
		DebugFilePrintf(debugFile, "Error: Algo select invalid %lx", registerValue); 
	}
}

void ReceiverCANCapture::ParseParameterAlgoParameterResponse(AWLCANMessage &inMsg)
{
	uint16_t parameterAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t parameterValue=  *(uint32_t *) &inMsg.data[4];

	AlgorithmParameter *parameter = FindAlgoParamByAddress(receiverStatus.currentAlgo, parameterAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());

	if (parameter != NULL)
	{
		parameter->floatValue = *(float *) &parameterValue; 
		parameter->intValue = *(int16_t *) &parameterValue; 
		parameter->pendingUpdates = updateStatusPendingVisual;
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverCANCapture::ParseParameterTrackerSelectResponse(AWLCANMessage &inMsg)
{

	uint16_t registerAddress = *(uint16_t *)&inMsg.data[2];  // Unused
	uint32_t registerValue = *(uint32_t *)&inMsg.data[4];

	AlgorithmDescription *trackerDescription = NULL;
	trackerDescription = FindAlgoDescriptionByID(parametersTrackers, registerValue);

	boost::mutex::scoped_lock rawLock(GetMutex());
	// Check that the tracker is valid (just in case communication goes crazy)
	if (trackerDescription != NULL)
	{
		receiverStatus.currentTracker = registerValue;
		receiverStatus.currentTrackerPendingUpdates--;
	}
	else
	{
		DebugFilePrintf(debugFile, "Error: Tracker select invalid %lx", registerValue);
	}

	receiverStatus.bUpdated = true;
	rawLock.unlock();
}

void ReceiverCANCapture::ParseParameterTrackerParameterResponse(AWLCANMessage &inMsg)
{
	uint16_t parameterAddress = *(uint16_t *)&inMsg.data[2];
	uint32_t parameterValue = *(uint32_t *)&inMsg.data[4];

	AlgorithmParameter *parameter = FindTrackerParamByAddress(receiverStatus.currentTracker, parameterAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());

	if (parameter != NULL)
	{
		parameter->floatValue = *(float *)&parameterValue;
		parameter->intValue = *(int16_t *)&parameterValue;
		parameter->pendingUpdates = updateStatusPendingVisual;
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
		registersFPGA[index].pendingUpdates = updateStatusPendingVisual;
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
		registersADC[index].pendingUpdates = updateStatusPendingVisual;
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
	uint16_t parameterAddress = *(uint16_t *) &inMsg.data[2];
	uint32_t parameterValue=  *(uint32_t *) &inMsg.data[4];
	int globalAlgo = 0; // Just so we know....

	AlgorithmParameter *parameter = FindAlgoParamByAddress(GLOBAL_PARAMETERS_ID, parameterAddress);

	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());

	if (parameter!= NULL)
	{
		parameter->floatValue = *(float *) &parameterValue; 
		parameter->intValue = *(int16_t *) &parameterValue; 
		parameter->pendingUpdates = updateStatusPendingVisual;
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
		registersGPIO[index].pendingUpdates = updateStatusPendingVisual;
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

void ReceiverCANCapture::ParseParameterTrackerSelectError(AWLCANMessage &inMsg)
{
	// Everything went well when we changed or queried the register. Note the new value.
	boost::mutex::scoped_lock rawLock(GetMutex());
	receiverStatus.bUpdated = true;
	receiverStatus.lastCommandError = inMsg.data[1];
	rawLock.unlock();
	DebugFilePrintf(debugFile, "Control command error.  Type %x", inMsg.data[1]);
}

void ReceiverCANCapture::ParseParameterTrackerParameterError(AWLCANMessage &inMsg)
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
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_DATE_TIME;

	*((uint16_t*)&message.data[2]) = 0x0001; // SET_DATE
	*((uint16_t*)&message.data[4]) = year-yearOffset;
	message.data[6] = (unsigned char) month-monthOffset;
	message.data[7] = (unsigned char) day;

	bMessageOk = bMessageOk && WriteMessage(message);

	// Write date
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_DATE_TIME;

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
		message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

		message.len = AWLCANMSG_LEN;       // Frame size (0.8)
		message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
		message.data[1] = AWLCANMSG_ID_CMD_PARAM_PLAYBACK_FILENAME;

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
		message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

		message.len = AWLCANMSG_LEN;       // Frame size (0.8)
		message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
		message.data[1] = AWLCANMSG_ID_CMD_PARAM_RECORD_FILENAME;

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
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_PLAYBACK_RAW;
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
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_PARAM_RECORD_FILENAME;
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
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_PLAYBACK_RAW;
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
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_PARAM_RECORD_FILENAME;
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
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_RECORD_CALIBRATION;   // Record_Calibration
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
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_ALGO_SELECTED;

	* (int16_t *) &message.data[2] = 0L; // Unused
	* (int32_t *) &message.data[4] = algorithmID;

	// Signal that we are waiting for an update of the register settings.
	
	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentAlgo = algorithmID;
	receiverStatus.currentAlgoPendingUpdates = 1;
	bool bMessageOk = WriteMessage(message);

   return(bMessageOk);
}

bool ReceiverCANCapture::SetTracker(uint16_t trackerID)
{

	AWLCANMessage message;

	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

	message.len = AWLCANMSG_LEN;       // Frame size (0.8)
	message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_TRACKER_SELECTED;

	*(int16_t *)&message.data[2] = 0L; // Unused
	*(int32_t *)&message.data[4] = trackerID;

	// Signal that we are waiting for an update of the register settings.

	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentTracker = trackerID;
	receiverStatus.currentTrackerPendingUpdates = 1;
	bool bMessageOk = WriteMessage(message);

	return(bMessageOk);
}


bool ReceiverCANCapture::SetFPGARegister(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_AWL_REGISTER;

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;
		

	// Signal that we are waiting for an update of thet register settings.
	bool bMessageOk = false;

	int index = FindRegisterByAddress(registersFPGA,registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersFPGA[index].pendingUpdates = updateStatusPendingUpdate;
	}
	
  bMessageOk = WriteMessage(message);

#if 0
	// wait for the reception thread to receive the acknowledge;
	awl::eUpdateStatus status = registersFPGA[index].pendingUpdates;
	
	while (status == updateStatusPendingUpdate)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		boost::mutex::scoped_lock rawLock(GetMutex());
		status = registersFPGA[index].pendingUpdates;
		rawLock.unlock();
	}
#endif
	return(bMessageOk);
}

bool ReceiverCANCapture::SetADCRegister(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_ADC_REGISTER;

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	// Signal that we are waiting for an update of thet register settings.
	bool bMessageOk = false;

	int index = FindRegisterByAddress(registersADC, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersADC[index].pendingUpdates = updateStatusPendingUpdate;
	}

  bMessageOk = WriteMessage(message);

	return(bMessageOk);
}

bool ReceiverCANCapture::SetGPIORegister(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_GPIO_CONTROL;

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;


	// Signal that we are waiting for an update of thet register settings.
	bool bMessageOk = false;
	int index = FindRegisterByAddress(registersGPIO, registerAddress);
	if (index >= 0)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		registersGPIO[index].pendingUpdates = updateStatusPendingUpdate;
		bMessageOk = WriteMessage(message);
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::SetAlgoParameter(int algoID, uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_ALGO_PARAMETER;

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;

	// Signal that we are waiting for an update of thet register settings.
	bool bMessageOk = false;
	AlgorithmParameter *parameter = FindAlgoParamByAddress(algoID, registerAddress);
	if (parameter != NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = updateStatusPendingUpdate;
		bMessageOk = WriteMessage(message);


		// Hack:  Update the SNR Cutoff in status when trying to set in algo parameters. 
		if (!parameter->sDescription.compare("SNR Cutoff (dB)"))
		{
			receiverStatus.signalToNoiseFloor = parameter->floatValue-4.0;
		}
	}

 	return(bMessageOk);
}

bool ReceiverCANCapture::SetGlobalAlgoParameter(uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_GLOBAL_PARAMETER;

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = registerValue;


	// Signal that we are waiting for an update of thet register settings.
	bool bMessageOk = false;
	AlgorithmParameter *parameter = FindAlgoParamByAddress(GLOBAL_PARAMETERS_ID, registerAddress);
	if (parameter!= NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = updateStatusPendingUpdate;
		bMessageOk = WriteMessage(message);
	}


 	return(bMessageOk);
}

bool ReceiverCANCapture::SetTrackerParameter(int trackerID, uint16_t registerAddress, uint32_t registerValue)
{
	AWLCANMessage message;

	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

	message.len = AWLCANMSG_LEN;       // Frame size (0.8)
	message.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_TRACKER_PARAMETER;

	*(int16_t *)&message.data[2] = registerAddress;
	*(int32_t *)&message.data[4] = registerValue;

	// Signal that we are waiting for an update of thet register settings.
	bool bMessageOk = false;
	AlgorithmParameter *parameter = FindTrackerParamByAddress(trackerID, registerAddress);
	if (parameter != NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = updateStatusPendingUpdate;
		bMessageOk = WriteMessage(message);
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask)

{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_TRANSMIT_COOKED;   // Transmit_cooked enable flags

	message.data[1] = channelMask.byteData; // Channel mask
	message.data[2] = 0;  // Reserved
	message.data[3] = frameRate; // New frame rate. oo= use actual.
	message.data[4] = messageMask.byteData; // Message mask
	message.data[5] = 0;  // Reserved
	message.data[6] = 0;  // Reserved
	message.data[7] = 0;  // Reserved

	bool bMessageOk = WriteMessage(message);

    message.data[0] = AWLCANMSG_ID_CMD_TRANSMIT_RAW;   // Transmit_raw enable flags

	message.data[1] = channelMask.byteData; // Channel mask
	message.data[2] = 0xFF;  // Reserved
	message.data[3] = 0;
	message.data[4] = 0;
	message.data[5] = 0;  // Reserved
	message.data[6] = 0;  // Reserved
	message.data[7] = 0;  // Reserved

	bMessageOk = WriteMessage(message);

	// The message has no confirmation built in
   return(bMessageOk);
}

bool ReceiverCANCapture::QueryAlgorithm()
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_ALGO_SELECTED;

	* (int16_t *) &message.data[2] = 0L;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of the register settings.
	
	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentAlgoPendingUpdates = updateStatusPendingUpdate;

    return(bMessageOk);
}

bool ReceiverCANCapture::QueryTracker()
{
	AWLCANMessage message;

	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

	message.len = AWLCANMSG_LEN;       // Frame size (0.8)
	message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_TRACKER_SELECTED;

	*(int16_t *)&message.data[2] = 0L;
	*(int32_t *)&message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of the register settings.

	// We should increment the pointer, but we just reset the 
	// counter to 1.  This makes display more robust in case we 
	// fall out of sync.
	receiverStatus.currentTrackerPendingUpdates = updateStatusPendingUpdate;

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryFPGARegister(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_AWL_REGISTER;

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
		registersFPGA[index].pendingUpdates = updateStatusPendingUpdate;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryADCRegister(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_ADC_REGISTER;

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
		registersADC[index].pendingUpdates = updateStatusPendingUpdate;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryGPIORegister(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_GPIO_CONTROL;

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
		registersGPIO[index].pendingUpdates = updateStatusPendingUpdate;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryAlgoParameter(int algoID, uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_ALGO_PARAMETER;

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
		parameter->pendingUpdates = updateStatusPendingUpdate;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryGlobalAlgoParameter(uint16_t registerAddress)
{
	AWLCANMessage message;
	
	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

    message.len = AWLCANMSG_LEN;       // Frame size (0.8)
    message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_GLOBAL_PARAMETER;

	* (int16_t *) &message.data[2] = registerAddress;
	* (int32_t *) &message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
	AlgorithmParameter *parameter = FindAlgoParamByAddress(GLOBAL_PARAMETERS_ID, registerAddress);
	if (parameter!= NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = updateStatusPendingUpdate;
	}

	return(bMessageOk);
}

bool ReceiverCANCapture::QueryTrackerParameter(int trackerID, uint16_t registerAddress)
{
	AWLCANMessage message;

	message.id = AWLCANMSG_ID_COMMANDMESSAGE;       // Message id: AWLCANMSG_ID_COMMANDMESSAGE- Command message

	message.len = AWLCANMSG_LEN;       // Frame size (0.8)
	message.data[0] = AWLCANMSG_ID_CMD_QUERY_PARAMETER;
	message.data[1] = AWLCANMSG_ID_CMD_PARAM_TRACKER_PARAMETER;

	*(int16_t *)&message.data[2] = registerAddress;
	*(int32_t *)&message.data[4] = 0L;

	bool bMessageOk = WriteMessage(message);

	// Signal that we are waiting for an update of thet register settings.
		AlgorithmParameter *parameter = FindTrackerParamByAddress(trackerID, registerAddress);
	if (parameter != NULL)
	{
		// We should increment the pointer, but we just reset the 
		// counter to 1.  This makes display more robust in case we 
		// fall out of sync.
		parameter->pendingUpdates = updateStatusPendingUpdate;
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

		// Get the parameters here
		canRate = (eReceiverCANRate) receiverNode.get<int>("canBitRate", defaultCANRate);
		return(true);
}

bool ReceiverCANCapture::ReadRegistersFromPropTree( boost::property_tree::ptree &propTree)
{
	using boost::property_tree::ptree;

	registersFPGA.clear();
	registersADC.clear();
	registersGPIO.clear();
	parametersAlgos.algorithms.clear();
	parametersTrackers.algorithms.clear();


	// Read all FPGA Registers default descriptions
	std::string registerDescKey = "config." + sReceiverRegisterSet;

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

	registersFPGALabel = configurationNodePtr->get<std::string>("registersFPGA.<xmlattr>.label", "");

	BOOST_FOREACH(ptree::value_type &registersFPGANode, configurationNodePtr->get_child("registersFPGA"))
	{
		if( registersFPGANode.first == "register" ) {
      int iAdvanced;
			boost::property_tree::ptree &registerNode = registersFPGANode.second;

            RegisterSetting registerFPGA;
            registerFPGA.sIndex = registerNode.get<std::string>("index");
            registerFPGA.address = registerNode.get<uint16_t>("address");
		    registerFPGA.sDescription = registerNode.get<std::string>("description");
        iAdvanced = registerNode.get<int>("advanced", 0);
        registerFPGA.bAdvanced = (iAdvanced != 0);
			registerFPGA.value = 0L;
			registerFPGA.pendingUpdates = updateStatusUpToDate;

			registersFPGA.push_back(registerFPGA);
        }
    }
 
 
	// Read all ADC Registers default descriptions

	registersADCLabel = configurationNodePtr->get<std::string>("registersADC.<xmlattr>.label", "");

	BOOST_FOREACH(ptree::value_type &registersADCNode, configurationNodePtr->get_child("registersADC"))
	{
		if( registersADCNode.first == "register" ) 
		{
			boost::property_tree::ptree &registerNode = registersADCNode.second;

            RegisterSetting registerADC;
            registerADC.sIndex = registerNode.get<std::string>("index");
            registerADC.address  = registerNode.get<uint16_t>("address");
		    registerADC.sDescription = registerNode.get<std::string>("description");
        registerADC.bAdvanced = false;
			registerADC.value = 0L;
			registerADC.pendingUpdates = updateStatusUpToDate;

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
        registerGPIO.bAdvanced = false;
			registerGPIO.value = 0L;
			registerGPIO.pendingUpdates = updateStatusUpToDate;

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

					// Hack: SNR in status is updated on read
					if (!parameter.sDescription.compare("SNR Cutoff (dB)"))
					{
						receiverStatus.signalToNoiseFloor = parameter.floatValue;
					}

					parameter.pendingUpdates = updateStatusUpToDate;
					algoDescription.parameters.push_back(parameter);
				} // if (parametersNode.first)
			} // BOOST_FOREACH (parametersNode)

			parametersAlgos.algorithms.push_back(algoDescription);
		} //		if (algoNode.first == "algo")
	} // BOOST_FOREACH(algosNode)

	// Load all atracker parameters for all Trackers

	parametersTrackers.defaultAlgo = configurationNodePtr->get<uint16_t>("trackers.defaultTracker");
	BOOST_FOREACH(ptree::value_type &trackersNode, configurationNodePtr->get_child("trackers"))
	{
		if (trackersNode.first == "tracker")
		{
			boost::property_tree::ptree &trackerNode = trackersNode.second;
			AlgorithmDescription algoDescription;
			algoDescription.algoID = trackerNode.get<uint16_t>("trackerID");
			algoDescription.sAlgoName = trackerNode.get<std::string>("trackerName");

			// All parameter info for the receiver
			BOOST_FOREACH(ptree::value_type &parametersNode, trackerNode/*.get_child("parameter")*/)
			{
				if (parametersNode.first == "parameter")
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

					parameter.pendingUpdates = updateStatusUpToDate;
					algoDescription.parameters.push_back(parameter);
				} // if (parametersNode.first)
			} // BOOST_FOREACH (parametersNode)

			parametersTrackers.algorithms.push_back(algoDescription);
		} //		if (trackerNode.first == "algo")
	} // BOOST_FOREACH(trackersNode)

	return(true);
}


#ifdef FORCE_FRAME_RESYNC_PATCH
// Big bad patch. AWL-7 does not receive all messages....


void ReceiverCANCapture::ForceFrameResync(AWLCANMessage &inMsg)

{
	// This is a "patch" intended to correct a known bug in AWL-7, where CAN frames may "overwrite" each other,
	// causing the end of frame message not to be received.
	// Happens when a large number of detections are made on multiple channels.
	// The way we detect end of frame is if "channel number" from message 10 decreases (channels are out of order), 
	// then we have skipped end of frame.

		unsigned long msgID = inMsg.id;

		if (msgID == 10)
		{
			uint8_t newChannelMask = *(uint8_t *)&inMsg.data[2];
			uint16_t newChannelID = *(uint16_t *)&inMsg.data[3];
#if 1
			// Compatibility patch: AWL-7 sends byte data only, but only one channel per channelMask.  
			// Other versions send trackMainChannel. For AWL-7 track main channel is 0.
			// Rebuild trackMainChannel from AWL 7 data.
			if (newChannelMask && !newChannelID)
			{
				newChannelID = 0;
				uint8_t channelMask = 0x01;
				for (int channel = 0; channel < 8; channel++)
				{
					if (newChannelMask & channelMask)
					{
						newChannelID = channel;
						break;
					}
					channelMask <<= 1;
				}
			}
#endif

			if (newChannelID < lastChannelID)
			{
				ProcessCompletedFrame();
				lastChannelID = 0;
			}

			lastChannelID = newChannelID;
		}

		if (msgID == 60)
		{
			uint16_t newChannelID = *(uint16_t *)&inMsg.data[3];

			if (newChannelID < lastChannelID)
			{
				ProcessCompletedFrame();
				lastChannelID = 0;
			}

			lastChannelID = newChannelID;
		}
#if 1
		if (msgID == 9)
		{
			ProcessCompletedFrame();
			lastChannelMask.byteData = 0;
			lastChannelID = 0;
		}
#endif
}

#endif // FORCE_FRAME_RESYNC_PATCH

// Channel index in the data cycle returned by the ADI chip
int aChIdxADI[16] = {
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  15,
  14,
  13,
  12,
  11,
  10,
  9,
  8
};

// Channel index of the Guardian 16rcvr Array
int aChIdxArray[16] = {
  15,
  0,
  14,
  1,
  13,
  2,
  12,
  3,
  11,
  4,
  10,
  5,
  9,
  6,
  8,
  7
};

void ReceiverCANCapture::ProcessRaw(RawProvider provider, uint8_t *rawData, size_t size)
{
	int channel = -1;
	int msg_id = -1;
	size_t sampleOffset = 0;
	size_t sampleDrop = 0;
	size_t sampleSize = 1;
	bool sampleSigned = false;
	bool transmit = false;

	uint16_t * rawData16;

	rawData16 = (uint16_t *)rawData;

	++m_nbrRawCumul;

/*
	printf ("ProcessRaw(%d) ", size);
	for (int i = 0; i < size; i++) {
		printf ("%02x ", rawData[i]);
	}
	printf ("\n");
*/

	switch (provider) {
		default:
		case rawFromLibUSB:
      channel = 0;
      sampleOffset = 0;
      sampleSize = 2;
      sampleSigned = true;
      sampleCount = 100;
      sampleDrop = 1;

      for (channel = 0; channel < 16; channel++)
      {
        int chIdxArray = aChIdxArray[channel];

        int chIdx = aChIdxADI[chIdxArray];

        if (!rawBuffers[channel])
          rawBuffers[channel] = new uint8_t[maxRawBufferSize];

        rawBufferCount++;

        memcpy(rawBuffers[channel], rawData + chIdx * (100 * 2), 100 * 2);
      }

      {
        boost::mutex::scoped_lock rawLock(GetMutex());

        for (channel = 0; channel < 16; channel++)
        {
          AScan::Ptr aScan = currentFrame->MakeUniqueAScan(currentFrame->aScans, receiverID, channel);
          aScan->samples = rawBuffers[channel];
          aScan->sampleSize = sampleSize;
          aScan->rawProvider = provider;
          aScan->sampleOffset = sampleOffset;
          aScan->sampleCount = sampleCount - sampleDrop;
          aScan->sampleSigned = sampleSigned;
        }
      }

      break;

		case rawFromPosixTTY:
			msg_id = rawData[0];
			if (msg_id != 0xb0) return;
			channel = rawData16[1];
			channel &= 0xff;
			if (channel >= maxRawBufferCount) break;
			sampleOffset = 12;
			sampleDrop = 0;
			sampleSize = 2;
			sampleSigned = true;
			if (! rawBuffers[channel]) rawBuffers[channel] = new uint8_t[maxRawBufferSize];
			rawBufferCount ++;
			if (size > maxRawBufferSize) size = maxRawBufferSize;
			memcpy (rawBuffers[channel], rawData, size);
			sampleCount = size / 2 - sampleOffset;
			transmit = true;
			if (channel > max_channel) max_channel = channel;
			if (channel == max_channel) transmit = true;
			break;	
		case rawFromPosixUDP:
			msg_id = rawData[0];
			channel = rawData16[1];
			channel &= 0xff;
			if (channel >= maxRawBufferCount) break;
			if (msg_id > 0xbf) return;
			sampleOffset = 16;
			sampleDrop = 100;
			sampleSize = 4;
			sampleSigned = true;
			switch (msg_id) {
			default:
				break;
			case 0x80:
			case 0x81:
				if (! rawBuffers[channel]) rawBuffers[channel] = new uint8_t[maxRawBufferSize];
				rawBufferCount ++;
				if (size > maxRawBufferSize) size = maxRawBufferSize;
				memcpy (rawBuffers[channel], rawData, size);
				sampleCount = size / 4 - sampleOffset;
				break;
			case 0x82:
			case 0x83:
			case 0x84:
				if (channel < 0) return;
				if (size > maxRawBufferSize / 4) size = maxRawBufferSize / 4;
				memcpy (rawBuffers[channel] + size * (msg_id - 0x81), rawData, size);
				sampleCount += size / 4 - sampleOffset;
				break;
			}
			if (msg_id > max_msg_id) max_msg_id = msg_id;
			if (msg_id == 0x80 || msg_id == max_msg_id) transmit = true;
	}	
	//printf("ascan %02x %02x %d %d %d\n", msg_id, max_msg_id, channel, size, sampleCount);

	if (transmit) {

		boost::mutex::scoped_lock rawLock(GetMutex());

		AScan::Ptr aScan = currentFrame->MakeUniqueAScan(currentFrame->aScans, receiverID, channel);
		aScan->samples = rawBuffers[channel];
		aScan->sampleSize = sampleSize;
		aScan->rawProvider = provider;
		aScan->sampleOffset = sampleOffset;
		aScan->sampleCount =  sampleCount - sampleDrop;
		aScan->sampleSigned = sampleSigned;
		//printf("transmit ascan %d %d\n", aScan->channelID, aScan->sampleCount);

		rawLock.unlock();
	}

	// Debug and Log messages
	//DebugFilePrintf(debugFile, "Msg %lu - Val %d %d %d %d", inMsg.id, distancePtr[0], distancePtr[1], distancePtr[2], distancePtr[3]);
}

