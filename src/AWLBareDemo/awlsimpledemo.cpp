/* AWLSimpleDemo.cpp */
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
#include <conio.h>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>


#include "AWLSettings.h" // Application specific
#include "AWLCoord.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "ReceiverEasySyncCapture.h"
#include "ReceiverKvaserCapture.h"
#include "ReceiverSimulatorCapture.h"
#include "ReceiverPostProcessor.h"
#include "DebugPrintf.h"
#include "AWLSimpledemo.h"

using namespace std;
using namespace awl;


AWLSimpleDemo::AWLSimpleDemo()
{

	// Read the settigs from the configuration file
	AWLSettings *globalSettings = AWLSettings::InitSettings();
	globalSettings->ReadSettings();

	// Build a reference coodinate system from the settings
	AWLCoordinates *globalCoordinates = AWLCoordinates::InitCoordinates();
	globalCoordinates->BuildCoordinatesFromSettings(globalSettings->GetPropTree());

	// Create the receiver communication objects
	int receiverQty = globalSettings->receiverSettings.size();
	receiverCaptures.resize(receiverQty);
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Create the LIDAR acquisition thread object, depending on the type identified in the config file
		if (boost::iequals(globalSettings->receiverSettings[receiverID].sReceiverType, "EasySyncCAN"))
		{
			// EasySync CAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverEasySyncCapture(receiverID, globalSettings->GetPropTree()));
		}
		else if (boost::iequals(globalSettings->receiverSettings[receiverID].sReceiverType, "KvaserLeaf"))
		{
			// Kvaser Leaf CAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverKvaserCapture(receiverID, globalSettings->GetPropTree()));
		}
		else 
		{
			// If the type is undefined, just use the dumb simulator, not using external device
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverSimulatorCapture(receiverID, globalSettings->GetPropTree()));
		}

		receiverCaptureSubscriberIDs.push_back(receiverCaptures[receiverID]->Subscribe());
	}

}

AWLSimpleDemo::~AWLSimpleDemo()
{
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}
}

int AWLSimpleDemo::exec()

{
	// Start the threads for background  receiver capture objects
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++) 
	{ 
		receiverCaptures[receiverID]->Go();
	}


	// And loop to display all the detections like crazy
	while (!kbhit()) 
	{
		DoOneLoopIteration();
	}

	return(0);
}


bool AWLSimpleDemo::DoOneLoopIteration()
{
	// Update the status information
	Detection::Vector detectionData;
	bool bNewDetections = GetLatestDetections(detectionData);		
	if (bNewDetections) OutputDetections(detectionData);

	return(true);
}

bool AWLSimpleDemo::GetLatestDetections(Detection::Vector &detectionData)
{
	AWLSettings *settings = AWLSettings::GetGlobalSettings();
	bool bNew = false;

	ReceiverPostProcessor postProcessor;

	// Build the list of detections that need to be updated

	// For each of the Receivers:
	///		see if it has any new data available and raise data available flag
	//		Pass the receiver through the ReceiverPostPorocessor to 
	//		acquire the raw data and complete it with complementary information.

	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];
		ReceiverSettings &receiverSettings = settings->receiverSettings[receiverID];

		// Use the frame snapped by the main display timer as the current frame
		Publisher::SubscriberID subscriberID = receiverCaptureSubscriberIDs[receiverID];
		uint32_t lastDisplayedFrame = receiver->GetCurrentIssueID(subscriberID);
		if (receiver->HasNews(subscriberID))
		{
			bNew = true;	
		}

		// Thread safe
		// The UI thread "Snaps" the frame ID for all other interface objects to display
		Detection::Vector detectionBuffer;
		if (postProcessor.GetEnhancedDetectionsFromFrame(receiver, lastDisplayedFrame, subscriberID, detectionBuffer))
		{
			// Copy and filter the detection data to keep only those we need.
			BOOST_FOREACH(const Detection::Ptr &detection, detectionBuffer)
			{
				Detection::Ptr storedDetection = detection;
				detectionData.push_back(storedDetection);
			}
		} // If (receiver...
	}

	return(bNew);
}


bool AWLSimpleDemo::OutputDetections(const Detection::Vector &detectionData)
{
	BOOST_FOREACH(const Detection::Ptr &detection, detectionData)
	{
		// Prepend the current time to output
		boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());

		std::string timeStr(boost::posix_time::to_simple_string(myTime));
		timeStr += ";";

		// For mat the detection information
		char detectionString[255];
		sprintf(detectionString, "Receiver %02d;Channel %02d;Distance from sensor %02.2f;Distance from car %02.2f", detection->receiverID, detection->channelID, detection->relativeToSensorCart.x,  detection->relativeToVehicleCart.x);

		timeStr += detectionString;
		timeStr += "\n";

		// Output this to the display
		printf(timeStr.c_str());
	}

	return(true);
}
