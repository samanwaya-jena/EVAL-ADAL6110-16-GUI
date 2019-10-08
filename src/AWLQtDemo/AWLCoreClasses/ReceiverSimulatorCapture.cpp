/* ReceiverSimulatorCapture.cpp */
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

#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "ReceiverSimulatorCapture.h"


using namespace std;
using namespace awl;


ReceiverSimulatorCapture::ReceiverSimulatorCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround,
	uint8_t inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset,
	const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO,
	const AlgorithmSet &inParametersAlgos, const AlgorithmSet &inParametersTrackers) :
ReceiverCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,
					   inRegistersFPGA, inRegistersADC, inRegistersGPIO, inParametersAlgos, inParametersTrackers)

{
}


ReceiverSimulatorCapture::ReceiverSimulatorCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCapture(receiverID, propTree)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);

}

ReceiverSimulatorCapture::~ReceiverSimulatorCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}

int threadCount = 0;
static float trackDistance = 0.0;

void ReceiverSimulatorCapture::DoOneThreadIteration()

{
	if (!WasStopped())
	{
		// Simulate some tracks for debug purposes
		Timestamp elapsed = GetElapsed();

		trackDistance += 0.0001f;
		if (trackDistance > 10.0) trackDistance = 0.0;

		for (uint16_t channel= 0; channel < 16; channel++)
		{
			for (uint16_t detection = 0; detection < 8; detection++)
			{
				// Only display even on even lines and odd on odd lines
				size_t lineIndex = (channel / this->receiverColumnQty);
				size_t lineQty = (GetChannelQty() / this->receiverColumnQty);  
				if (lineIndex == (channel%lineQty)) 
				{
					Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, (channel * 8) + detection);
					track->firstTimeStamp = currentFrame->timeStamp;

					track->timeStamp = currentFrame->timeStamp;
					track->distance = (detection * 2) + trackDistance;
					track->intensity = 1.00;
					track->trackMainChannel = channel;

					track->trackChannels.wordData = 0x01 << (track->trackMainChannel % 8);
					

					track->velocity = 3;
					track->acceleration = 0;
					track->threatLevel = AlertCondition::eThreatLow;
					track->part1Entered = true;
					track->part2Entered = true;
					track->part3Entered = true;
					track->part4Entered = true;

					track->probability = 99;
					track->timeStamp = elapsed;
					track->firstTimeStamp = elapsed;
				}
			}
		}

		threadCount = ++ threadCount % 7;
		ProcessCompletedFrame();
	} // if  (!WasStoppped)
}


bool ReceiverSimulatorCapture::SetMessageFilters(uint8_t /*frameRate*/, ChannelMask /*channelMask*/, MessageMask /*messageMask*/)

{
	return(true);
}


bool ReceiverSimulatorCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		ReceiverCapture::ReadConfigFromPropTree(propTree);
		receiverStatus.signalToNoiseFloor = 18.0;
		return(true);
}

