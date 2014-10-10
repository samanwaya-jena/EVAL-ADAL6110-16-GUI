/* ReceiverSimulatorCapture.cpp */
/*
	Copyright 2014 Aerostar R&D Canada Inc.

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


ReceiverSimulatorCapture::ReceiverSimulatorCapture(int receiverID, int inReceiverChannelQty,  
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos):
ReceiverCapture(receiverID, inReceiverChannelQty, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, inParametersAlgos)

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

void ReceiverSimulatorCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
	// Simulate some tracks for debug purposes
	double elapsed = GetElapsed();
	Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, 0);

	track->firstTimeStamp = currentFrame->timeStamp;
	track->timeStamp = currentFrame->timeStamp;
	track->distance = 2.0;
	track->channels = 0X7f;

	track->velocity = 22;
	track->acceleration = 0;
	track->part1Entered = true;
	track->part2Entered = true;
	track->part3Entered = true;
	track->part4Entered = true;

	track->probability = 99;
	track->timeStamp = elapsed;
	track->firstTimeStamp = elapsed;


	track = acquisitionSequence->MakeUniqueTrack(currentFrame, 1);

	track->firstTimeStamp = currentFrame->timeStamp;
	track->timeStamp = currentFrame->timeStamp;
	track->distance = 4;
	track->channels = 0X7f;

	track->velocity = -3;
	track->acceleration = -3;
	track->part1Entered = true;
	track->part2Entered = true;
	track->part3Entered = true;
	track->part4Entered = true;

	track->probability = 99;
	track->timeStamp = elapsed;
	track->firstTimeStamp = elapsed;
	ProcessCompletedFrame();
	} // if  (!WasStoppped)
}



bool ReceiverSimulatorCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		ReceiverCapture::ReadConfigFromPropTree(propTree);
		return(true);
}

