/* AWLQtDemo/AWLSettings.cpp */
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

#include "AWLSettings.h"
#include "DetectionStruct.h"

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>


using namespace awl;
using namespace std;
const std::string sDefaultSettingsFileName("AWLDemoSettings.xml");


AWLSettings *AWLSettings::globalSettings=NULL;

AWLSettings::AWLSettings(const std::string sSettingsFileName) :
sFileName(sSettingsFileName),
sLogoFileName(""),
sIconFileName(""),
bDisplayVideoCrosshair(false),
bDisplayVideoTime(false)

{
	if (sFileName.empty())
	{
		sFileName.assign(sDefaultSettingsFileName);

	}
}

AWLSettings * AWLSettings::InitSettings(const std::string sSettingsFileName)
{
	globalSettings = new AWLSettings(sSettingsFileName);
	return(globalSettings);
}

AWLSettings *AWLSettings::GetGlobalSettings()
{
	return(globalSettings);
}



bool AWLSettings::ReadSettings()
{
	// Create an empty property tree object
    using boost::property_tree::ptree;
	propTree.clear();

	// Load the XML file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
    read_xml(sFileName, propTree);

	int receiverQty = propTree.get<int>("config.receivers.receiverQty");
	receiverSettings.resize(receiverQty);
	for (int receiverIndex = 0; receiverIndex < receiverQty; receiverIndex++)
	{
		char receiverKeyString[32];
		sprintf(receiverKeyString, "config.receivers.receiver%d", receiverIndex);
		std::string receiverKey = receiverKeyString;

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);

		ReceiverSettings *receiverPtr = &receiverSettings[receiverIndex];
		receiverPtr->sReceiverType = receiverNode.get<std::string>("receiverType");
		receiverPtr->sReceiverRegisterSet = receiverNode.get<std::string>("receiverRegisterSet");
		receiverPtr->sReceiverChannelGeometry = receiverNode.get<std::string>("receiverChannelGeometry");

		// Display
		receiverPtr->displayedRangeMin = receiverNode.get<float>("displayedRangeMin");
		receiverPtr->displayedRangeMax = receiverNode.get<float>("displayedRangeMax");

		// Get the Channel configuration Node
		std::string channelGeometryKey = "config." + receiverPtr->sReceiverChannelGeometry;
		boost::property_tree::ptree &channelGeometryNode = propTree.get_child(channelGeometryKey);
		GetChannelGeometry(channelGeometryNode, receiverPtr);
	} // for (int receiverIndex = 0; 


	int cameraQty = propTree.get<int>("config.cameras.cameraQty");
	cameraSettings.resize(cameraQty);
	for (int cameraIndex = 0; cameraIndex < cameraQty; cameraIndex++)
	{
		char cameraKeyString[32];
		sprintf(cameraKeyString, "config.cameras.camera%d", cameraIndex);
		std::string cameraKey = cameraKeyString;

		boost::property_tree::ptree &cameraNode =  propTree.get_child(cameraKey);
		CameraSettings *cameraPtr = &cameraSettings[cameraIndex];

		cameraPtr->sCameraName = cameraNode.get<std::string>("cameraName");
		cameraPtr->cameraFlip = cameraNode.get<bool>("cameraFlip");
		Get2DPoint(cameraNode.get_child("fov"), cameraPtr->cameraFovWidthDegrees, cameraPtr->cameraFovHeightDegrees);
		cameraPtr->barrelK1 = 0.0;
		cameraPtr->barrelK2 = 0.0;

		cameraPtr->barrelK1 = cameraNode.get<float>("barrelCorrectionK1", 0.0);
		cameraPtr->barrelK2 = cameraNode.get<float>("barrelCorrectionK2", 0.0);
	}

	// Debug and log file control
	bWriteDebugFile = propTree.get<bool>("config.debug.enableDebugFile");
	bWriteLogFile = propTree.get<bool>("config.debug.enableLogFile");

	// Other settings
	bDisplaySettingsWindow = propTree.get<bool>("config.layout.displaySettingsWindow");
	bDisplay3DWindow = propTree.get<bool>("config.layout.display3DWindow");
	bDisplay2DWindow = propTree.get<bool>("config.layout.display2DWindow");
	bDisplayTableViewWindow = propTree.get<bool>("config.layout.displayTableViewWindow");;
	bDisplayScopeWindow = propTree.get<bool>("config.layout.displayScopeWindow");
	bDisplayCameraWindow = propTree.get<bool>("config.layout.displayCameraWindow");

	velocityUnits = (VelocityUnits) propTree.get<int>("config.layout.velocityUnits");

	sLogoFileName = propTree.get<std::string>("config.layout.logoFileName");
	sIconFileName = propTree.get<std::string>("config.layout.iconFileName");
	sDisplayShowSize = propTree.get<std::string>("config.layout.displayShowSize", "Normal");

	displayedDetectionsPerChannelInTableView = propTree.get<int>("config.displayTableView.displayedDetectionsPerChannelInTableView");

	carWidth = propTree.get<float>("config.display2D.carWidth");
	carLength = propTree.get<float>("config.display2D.carLength");
	carHeight = propTree.get<float>("config.display2D.carHeight");
	laneWidth = propTree.get<float>("config.display2D.laneWidth");
	shortRangeDistance = propTree.get<float>("config.display2D.shortRangeDistance");
	shortRangeDistanceStartLimited = propTree.get<float>("config.display2D.shortRangeDistanceStartLimited");
	shortRangeAngle = propTree.get<float>("config.display2D.shortRangeAngle");
	shortRangeAngleStartLimited = propTree.get<float>("config.display2D.shortRangeAngleStartLimited");

	longRangeDistance = propTree.get<float>("config.display2D.longRangeDistance");
	longRangeDistanceStartLimited = propTree.get<float>("config.display2D.longRangeDistanceStartLimited");
	longRangeAngle = propTree.get<float>("config.display2D.longRangeAngle");
	longRangeAngleStartLimited = propTree.get<float>("config.display2D.longRangeAngleStartLimited");

	showPalette = propTree.get<int>("config.display2D.showPalette");
	mergeDisplayMode = propTree.get<int>("config.display2D.mergeDisplayMode");
	measureMode = propTree.get<int>("config.display2D.measureMode");
	Get2DPoint(propTree.get_child("config.display2D.mergeAcceptance"), mergeAcceptanceX, mergeAcceptanceY);
	colorCode2D = propTree.get<int>("config.display2D.colorCode");
	maxVelocity2D = propTree.get<float>("config.display2D.maxVelocity");
	zeroVelocity = propTree.get<float>("config.display2D.zeroVelocity");
	displayDistanceMode2D = propTree.get<int>("config.display2D.displayDistances");
	displayZoomMode2D = propTree.get<int>("config.display2D.displayZoom");

	scopeTimerInterval = propTree.get<int>("config.scope.timerInterval");
	bDisplayScopeDistance = propTree.get<bool>("config.scope.displayScopeDistance");
	bDisplayScopeVelocity = propTree.get<bool>("config.scope.displayScopeVelocity");

	brakingDeceleration = propTree.get<float>("config.dynamicTesting.brakingDeceleration");
	travelSpeed = propTree.get<float>("config.dynamicTesting.travelSpeed");

	bDisplayVideoCrosshair = propTree.get<bool>("config.video.displayCrosshair");
	bDisplayVideoTime = propTree.get<bool>("config.video.displayTime");

	// Alert conditions
	int alertConditionQty = propTree.get<int>("config.dynamicTesting.alertQty");
	for (int alertConditionIndex = 0; alertConditionIndex < alertConditionQty; alertConditionIndex++)
	{
		char alertKeyString[32];
		sprintf(alertKeyString, "config.dynamicTesting.alert%d", alertConditionIndex);
		std::string alertKey = alertKeyString;

		boost::property_tree::ptree &alertNode = propTree.get_child(alertKey);
		AlertCondition::Ptr newAlert = AlertCondition::Ptr(new AlertCondition());
		GetAlertConditions(alertNode, *newAlert);
		AlertCondition::Store(newAlert);
	}


	return(true);
}

bool AWLSettings::StoreReceiverCalibration()

{
	// Create an empty property tree object
    using boost::property_tree::ptree;
    ptree propTree;

	// Load the XML file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
    read_xml(sFileName, propTree);


#if 0
	// Loop for all Receiver Configurations
	BOOST_FOREACH(ptree::value_type &receiversNode, propTree.get_child("config.receivers"))
	{
		if( receiversNode.first == "receiver" ) 
		{
			boost::property_tree::ptree &receiverNode = receiversNode.second;
			ReceiverSettings receiver;

			receiverNode.put<std::string>("receiverType", receiver.sReceiverType);
			receiverNode.put<std::string>("receiverRegisterSet", receiver.sReceiverRegisterSet);
			receiverNode.put<std::string>("receiverChannelGeometry", receiver.sReceiverGeometry);


			receiverNode.put<uint8_t>("channelMask", receiver.receiverChannelMask);
			receiverNode.put<uint8_t>("frameRate", receiver.receiverFrameRate);

			// Geometry
			boost::property_tree::ptree &geometryNode = receiverNode.get_child("sensorGeometry");
			PutGeometry(geometryNode, 
				receiver.sensorForward, receiver.sensorLeft, receiver.sensorUp,
				receiver.sensorPitch, receiver.sensorYaw, receiver.sensorRoll);

			// Display
			receiverNode.put<float>("displayedRangeMin", receiver.displayedRangeMin);
			receiverNode.put<float>("displayedRangeMax", receiver.displayedRangeMax);
			receiverNode.put<float>("rangeOffset", receiver.rangeOffset);

			// All channel info for the receiver
			BOOST_FOREACH(ptree::value_type &channelsNode, receiverNode)
			{
				if( channelsNode.first == "channel" ) 
				{
					boost::property_tree::ptree &channelNode = channelsNode.second;
					ChannelConfig channelConfig;

					channelNode.put<int>("index", channelConfig.channelIndex);
					Put2DPoint(channelNode.get_child("fov"), channelConfig.fovWidth, channelConfig.fovHeight);
					float roll = 0.0;
					PutOrientation(channelNode.get_child("orientation"), channelConfig.centerY, channelConfig.centerX, roll);
					channelNode.put<float>("maxRange", channelConfig.maxRange);

					PutColor(channelNode.get_child("displayColor"), 
						channelConfig.displayColorRed, channelConfig.displayColorGreen, channelConfig.displayColorBlue);

					channelsNode->add_child(channelNode);
				}// if( receiversNode.first == "channel"

			} // BOOST_FOREACH(ptree::value_type &channelsNode
			receiverNode.put_child(channelsNode);
		} // If receiversNode.first == "receiver"
		receiversNode.put_child(receiverNode);
	} // BOOST_FOREACH(ptree::value_type &receiversNode, propT
	proptree->put_child(receiversNode);

	// Write the XML file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
	 boost::property_tree::xml_writer_settings<char> set(' ', 4);
	write_xml("test.xml", propTree, std::locale(), set);
#endif
	return (true);
}


void AWLSettings::GetPosition(boost::property_tree::ptree &node, float &forward, float &left, float&up)
{
	forward = node.get<float>("forward");
	left = node.get<float>("left");
	up = node.get<float>("up");
}

void AWLSettings::GetOrientation(boost::property_tree::ptree &node, float &pitch, float &yaw, float &roll)
{
	pitch = node.get<float>("pitch");
	yaw = node.get<float>("yaw");
	roll = node.get<float>("roll");
}

void AWLSettings::Get2DPoint(boost::property_tree::ptree &node, float &x, float &y)
{
	x = node.get<float>("x");
	y = node.get<float>("y");
}

void AWLSettings::GetGeometry(boost::property_tree::ptree &geometryNode, float &forward, float &left, float &up, float &pitch, float &yaw, float &roll)
{
	boost::property_tree::ptree &positionNode = geometryNode.get_child("position");
	boost::property_tree::ptree &orientationNode = geometryNode.get_child("orientation");
	GetPosition(positionNode, forward, left, up);
	GetOrientation(orientationNode, pitch, yaw, roll);
}

void AWLSettings::GetColor(boost::property_tree::ptree &colorNode, uint8_t &red, uint8_t &green, uint8_t &blue)
{
	red = colorNode.get<uint8_t>("red");
	green = colorNode.get<uint8_t>("green");
	blue = colorNode.get<uint8_t>("blue");
}

void AWLSettings::GetAlertConditions(boost::property_tree::ptree &alertNode, AlertCondition &alert)

{
	std::string sAlertType = alertNode.get<std::string>("alertType");
	if (sAlertType.compare("distanceWithin") == 0)
	{
		alert.alertType = AlertCondition::eAlertDistanceWithin;
	}
	else  if (sAlertType.compare("distanceOutside") == 0)
	{
		alert.alertType = AlertCondition::eAlertDistanceOutside;
	}
	else  if (sAlertType.compare("speed") == 0)
	{
		alert.alertType = AlertCondition::eAlertSpeed;
	}
	else  if (sAlertType.compare("acceleration") == 0)
	{
		alert.alertType = AlertCondition::eAlertAcceleration;
	}
	else  if (sAlertType.compare("decelerationToStop") == 0)
	{
		alert.alertType = AlertCondition::eAlertDecelerationToStop;
	}
	else  if (sAlertType.compare("TTC") == 0)
	{
		alert.alertType = AlertCondition::eAlertTTC;
	}
	else
	{
		alert.alertType = AlertCondition::eAlertInvalid;
	}

	alert.receiverID = alertNode.get<int>("alertReceiver", 0);
	alert.channelMask.byteData = alertNode.get<uint8_t>("alertChannels", 255);
	alert.minRange = alertNode.get<float>("alertMin", -std::numeric_limits<float>::max());
	alert.maxRange = alertNode.get<float>("alertMax", std::numeric_limits<float>::max());
	alert.threatLevel = (AlertCondition::ThreatLevel) alertNode.get<int>("alertLevel", AlertCondition::eThreatNone);
}

void AWLSettings::GetChannelGeometry(boost::property_tree::ptree &channelGeometryNode, ReceiverSettings *receiverPtr)
{
	// All channel info for the receiver
	int channelQty = channelGeometryNode.get<int>("channelQty");
	receiverPtr->channelsConfig.resize(channelQty);

	// Range Wraparound trick
	receiverPtr->lineWrapAround = channelGeometryNode.get<float>("lineWrapAround", 32767.0);
	receiverPtr->channelsPerLine = channelGeometryNode.get<int>("channelsPerLine", 1);

	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		char channelKeyString[32];
		sprintf(channelKeyString, "channel%d", channelIndex);
		std::string channelKey = channelKeyString;

		boost::property_tree::ptree &channelNode = channelGeometryNode.get_child(channelKey);

		ChannelConfig *channelConfigPtr = &receiverPtr->channelsConfig[channelIndex];
		channelConfigPtr->channelIndex = channelIndex;
		Get2DPoint(channelNode.get_child("fov"), channelConfigPtr->fovWidth, channelConfigPtr->fovHeight);
		float roll;
		channelConfigPtr->maxRange = channelNode.get<float>("maxRange");

		GetColor(channelNode.get_child("displayColor"),
			channelConfigPtr->displayColorRed, channelConfigPtr->displayColorGreen, channelConfigPtr->displayColorBlue);
	} // for (int channelIndex = 0;

}
void AWLSettings::PutPosition(boost::property_tree::ptree &node, float forward, float left, float up)
{
	node.put<float>("forward", forward);
	node.put<float>("left", left);
	node.put<float>("up", up);
}

void AWLSettings::PutOrientation(boost::property_tree::ptree &node, float pitch, float yaw, float roll)
{
	node.put<float>("pitch", pitch);
	node.put<float>("yaw", yaw);
	node.put<float>("roll", roll);
}

void AWLSettings::Put2DPoint(boost::property_tree::ptree &node, float x, float y)
{
	node.put<float>("x", x);
	node.put<float>("y", y);
}

void AWLSettings::PutGeometry(boost::property_tree::ptree &geometryNode, float forward, float left, float up, float pitch, float yaw, float roll)
{
	boost::property_tree::ptree &positionNode = geometryNode.put_child("position", boost::property_tree::ptree(""));
	boost::property_tree::ptree &orientationNode = geometryNode.put_child("orientation", boost::property_tree::ptree(""));
	PutPosition(positionNode, forward, left, up);
	PutOrientation(orientationNode, pitch, yaw, roll);
}

void AWLSettings::PutColor(boost::property_tree::ptree &colorNode, uint8_t red, uint8_t green, uint8_t blue)
{
	colorNode.get<uint8_t>("red");
	colorNode.get<uint8_t>("green");
	colorNode.get<uint8_t>("blue");
}


void AWLSettings::PutChannelGeometry(boost::property_tree::ptree &channelGeometryNode, ReceiverSettings *receiverPtr)
{
	int channelQty = receiverPtr->channelsConfig.size();
	// All channel info for the receiver
	channelGeometryNode.put<int>("channelQty", channelQty);

	// Range Wraparound trick
	channelGeometryNode.put<float>("lineWrapAround", receiverPtr->lineWrapAround);
	channelGeometryNode.put<int>("channelsPerLine", receiverPtr->channelsPerLine);

	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		char channelKeyString[32];
		sprintf(channelKeyString, "channel%d", channelIndex);
		std::string channelKey = channelKeyString;

		boost::property_tree::ptree &channelNode = channelGeometryNode.put_child(channelKey, boost::property_tree::ptree(""));

		ChannelConfig *channelConfigPtr = &receiverPtr->channelsConfig[channelIndex];
		channelConfigPtr->channelIndex = channelIndex;
		Put2DPoint(channelNode.put_child("fov", boost::property_tree::ptree("")), channelConfigPtr->fovWidth, channelConfigPtr->fovHeight);
		float roll;
		channelConfigPtr->maxRange = channelNode.get<float>("maxRange");

		PutColor(channelNode.put_child("displayColor", boost::property_tree::ptree("")),
			channelConfigPtr->displayColorRed, channelConfigPtr->displayColorGreen, channelConfigPtr->displayColorBlue);
	} // for (int channelIndex = 0;
}