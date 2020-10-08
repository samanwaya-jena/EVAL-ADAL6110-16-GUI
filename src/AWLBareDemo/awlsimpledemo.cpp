/* AWLSimpleDemo.cpp: Minima Interface and sequence management  
   for a console-based LiDAR application */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the BareDemo application of the
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


#include <string>
#include <conio.h>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include "AWLSimpleSettings.h" // Application specific
#include "SensorCoord.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"

#ifdef USE_LIBUSB
#include "ReceiverLibUSBCapture.h"
#endif
#include "ReceiverSimulatorCapture.h"
#include "ReceiverPostProcessor.h"
#include "DebugPrintf.h"
#include "AWLSimpledemo.h"

using namespace awl;
SENSORCORE_USE_NAMESPACE


AWLSimpleDemo::AWLSimpleDemo()
{

	// Read the settigs from the configuration file
	AWLSimpleSettings *globalSettings = AWLSimpleSettings::InitSettings();
	globalSettings->ReadSettings();

	// Build a reference coodinate system from the settings
	SensorCoordinates *globalCoordinates = SensorCoordinates::InitCoordinates();
	globalCoordinates->BuildCoordinatesFromSettings(globalSettings->GetPropTree());

	// Create the receiver communication objects
	int receiverQty = globalSettings->receiverSettings.size();
	receiverCaptures.resize(receiverQty);
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Create the LIDAR acquisition thread object, depending on the type identified in the config file



#ifdef USE_LIBUSB
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string("LibUSB"))
		{
			// LibUSB Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverLibUSBCapture(receiverID, globalSettings->GetPropTree()));
		}
		else
#endif
		{
			// If the type is undefined, just use the dumb simulator, not using external device
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverSimulatorCapture(receiverID, globalSettings->GetPropTree()));
		}

		receiverCaptureSubscriberIDs.push_back(receiverCaptures[receiverID]->Subscribe());
	}


}

AWLSimpleDemo::~AWLSimpleDemo()
{
	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}
}

int AWLSimpleDemo::exec()

{
	// Start the threads for background  receiver capture objects
	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++) 
	{ 
		receiverCaptures[receiverID]->Go();
	}


	// And loop to display all the detections like crazy
	while (!_kbhit()) 
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
	AWLSimpleSettings *settings = AWLSimpleSettings::GetGlobalSettings();
	bool bNew = false;

	ReceiverPostProcessor postProcessor;

	// Build the list of detections that need to be updated

	// For each of the Receivers:
	///		see if it has any new data available and raise data available flag
	//		Pass the receiver through the ReceiverPostPorocessor to 
	//		acquire the raw data and complete it with complementary information.

	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];
		ReceiverSettings &receiverSettings = settings->receiverSettings[receiverID];

		// Use the frame snapped by the main display timer as the current frame
		Publisher::SubscriberID subscriberID = receiverCaptureSubscriberIDs[receiverID];
		FrameID lastDisplayedFrame = receiver->GetCurrentIssueID(subscriberID);
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
		sprintf_s(detectionString, 255, "Receiver %02d;column, row %02d %02d ;Distance from sensor %02.2f;Distance from car %02.2f", detection->receiverID, detection->cellID.column, detection->cellID.row, detection->relativeToSensorCart.cartesian.x,  detection->relativeToVehicleCart.cartesian.x);

		timeStr += detectionString;
		timeStr += "\n";

		// Output this to the display
		printf(timeStr.c_str());
	}

	return(true);
}
