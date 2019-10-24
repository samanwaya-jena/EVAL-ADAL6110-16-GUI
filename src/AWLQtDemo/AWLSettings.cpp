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

	int receiverQty = propTree.get<int>("config.receivers.receiverQty", 0);
	receiverSettings.resize(receiverQty);
	for (int receiverIndex = 0; receiverIndex < receiverQty; receiverIndex++)
	{
		std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverIndex);

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);

		ReceiverSettings *receiverPtr = &receiverSettings[receiverIndex];
		receiverPtr->sReceiverType = receiverNode.get<std::string>("receiverType");
		receiverPtr->sReceiverRegisterSet = receiverNode.get<std::string>("receiverRegisterSet");
		receiverPtr->sReceiverChannelGeometry = receiverNode.get<std::string>("receiverChannelGeometry");

		// Display
		receiverPtr->displayedRangeMin = receiverNode.get<float>("displayedRangeMin", (float)0.0);
		receiverPtr->displayedRangeMax = receiverNode.get<float>("displayedRangeMax", (float) 60.0);

		// Get the Channel configuration Node
		std::string channelGeometryKey = "config." + receiverPtr->sReceiverChannelGeometry;
		boost::property_tree::ptree &channelGeometryNode = propTree.get_child(channelGeometryKey);
		GetChannelGeometry(channelGeometryNode, receiverPtr);
	} // for (int receiverIndex = 0; 


	int cameraQty = propTree.get<int>("config.cameras.cameraQty", 0);
	cameraSettings.resize(cameraQty);
	for (int cameraIndex = 0; cameraIndex < cameraQty; cameraIndex++)
	{
		std::string cameraKey = std::string("config.cameras.camera") + std::to_string(cameraIndex);

		boost::property_tree::ptree &cameraNode =  propTree.get_child(cameraKey);
		CameraSettings *cameraPtr = &cameraSettings[cameraIndex];

		cameraPtr->sCameraName = cameraNode.get<std::string>("cameraName", "0");
		cameraPtr->sCameraAPI = cameraNode.get<std::string>("cameraAPI", "OpenCV");
		cameraPtr->cameraFlip = cameraNode.get<bool>("cameraFlip", false);
		Get2DPoint(cameraNode.get_child("fov"), cameraPtr->cameraFovWidthDegrees, cameraPtr->cameraFovHeightDegrees);
		cameraPtr->barrelK1 = 0.0;
		cameraPtr->barrelK2 = 0.0;

		cameraPtr->barrelK1 = cameraNode.get<float>("barrelCorrectionK1", 0.0);
		cameraPtr->barrelK2 = cameraNode.get<float>("barrelCorrectionK2", 0.0);
	}

	// Debug and log file control
	bWriteDebugFile = propTree.get<bool>("config.debug.enableDebugFile", false);
	bWriteLogFile = propTree.get<bool>("config.debug.enableLogFile", false);

	// Other settings
	bDisplaySettingsWindow = propTree.get<bool>("config.layout.displaySettingsWindow",false);
	bDisplay2DWindow = propTree.get<bool>("config.layout.display2DWindow",false);
	bDisplayTableViewWindow = propTree.get<bool>("config.layout.displayTableViewWindow",false);
	bDisplayAScanViewWindow = propTree.get<bool>("config.layout.displayAScanViewWindow",false);
	bDisplayCameraWindow = propTree.get<bool>("config.layout.displayCameraWindow",false);
	bDisplayAboutWindow = propTree.get<bool>("config.layout.displayAboutWindow",false);

	bTabSettingCalibration = propTree.get<bool>("config.layout.TabSettingCalibration", false);
	bTabSettingControl = propTree.get<bool>("config.layout.TabSettingControl", false);
	bTabSettingStatus = propTree.get<bool>("config.layout.TabSettingStatus", false);
	bTabSettingRegisters = propTree.get<bool>("config.layout.TabSettingRegisters", false);
	bTabSettingGPIOs = propTree.get<bool>("config.layout.TabSettingGPIOs", false);
	bTabSettingAlgoControl = propTree.get<bool>("config.layout.TabSettingAlgoControl", false);
	bTabSettingTrackerControl = propTree.get<bool>("config.layout.TabSettingTrackerControl", false);
	bTabSettingAScan = propTree.get<bool>("config.layout.TabSettingAScan", false);
	bTabSettingMisc = propTree.get<bool>("config.layout.TabSettingMisc", false);

	velocityUnits = (VelocityUnits) propTree.get<int>("config.layout.velocityUnits", 1);

	sLogoFileName = propTree.get<std::string>("config.layout.logoFileName");
	sIconFileName = propTree.get<std::string>("config.layout.iconFileName");
	sDisplayShowSize = propTree.get<std::string>("config.layout.displayShowSize", "Normal");

	displayedDetectionsPerChannelInTableView = propTree.get<int>("config.displayTableView.displayedDetectionsPerChannelInTableView", 2);

	carWidth = propTree.get<float>("config.display2D.carWidth", (float) 1.78);
	carLength = propTree.get<float>("config.display2D.carLength", (float) 4.53 );
	carHeight = propTree.get<float>("config.display2D.carHeight", (float) 1.44);
	laneWidth = propTree.get<float>("config.display2D.laneWidth", (float) -1);

	showPalette = propTree.get<int>("config.display2D.showPalette", 0);
	mergeDisplayMode = propTree.get<int>("config.display2D.mergeDisplayMode", 0);
	measureMode = propTree.get<int>("config.display2D.measureMode", 1);
	Get2DPoint(propTree.get_child("config.display2D.mergeAcceptance"), mergeAcceptanceX, mergeAcceptanceY);
	colorCode2D = propTree.get<int>("config.display2D.colorCode", 4);
	maxVelocity2D = propTree.get<float>("config.display2D.maxVelocity", (float) 30.0);
	zeroVelocity = propTree.get<float>("config.display2D.zeroVelocity", (float) 1.0);
	displayDistanceMode2D = propTree.get<int>("config.display2D.displayDistances", 0);
	displayZoomMode2D = propTree.get<int>("config.display2D.displayZoom", 0);

	brakingDeceleration = propTree.get<float>("config.dynamicTesting.brakingDeceleration", (float) 5.096);
	travelSpeed = propTree.get<float>("config.dynamicTesting.travelSpeed", (float)33.32);

	bDisplayVideoCrosshair = propTree.get<bool>("config.video.displayCrosshair", false);
	bDisplayVideoTime = propTree.get<bool>("config.video.displayTime", true);

	// Alert conditions
	int alertConditionQty = propTree.get<int>("config.dynamicTesting.alertQty", 0);
	for (int alertConditionIndex = 0; alertConditionIndex < alertConditionQty; alertConditionIndex++)
	{
		std::string alertKey = std::string("config.dynamicTesting.alert") + std::to_string(alertConditionIndex);

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
    ptree localPropTree;

	// Load the XML file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
    read_xml(sFileName, localPropTree);


#if 0
	// Loop for all Receiver Configurations
	BOOST_FOREACH(ptree::value_type &receiversNode, localPropTree.get_child("config.receivers"))
	{
		if( receiversNode.first == "receiver" ) 
		{
			boost::property_tree::ptree &receiverNode = receiversNode.second;
			ReceiverSettings receiver;

			receiverNode.put<std::string>("receiverType", receiver.sReceiverType);
			receiverNode.put<std::string>("receiverRegisterSet", receiver.sReceiverRegisterSet);
			receiverNode.put<std::string>("receiverChannelGeometry", receiver.sReceiverGeometry);


			receiverNode.put<uint16_t>("channelMask", receiver.receiverChannelMask);
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
	localProptree->put_child(receiversNode);

	// Write the XML file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
	 boost::property_tree::xml_writer_settings<char> set(' ', 4);
	write_xml("test.xml", localPropTree, std::locale(), set);
#endif
	return (true);
}


void AWLSettings::GetPosition(boost::property_tree::ptree &node, float &forward, float &left, float&up)
{
	forward = node.get<float>("forward", (float)0.0);
	left = node.get<float>("left", (float)0.0);
	up = node.get<float>("up", (float)0.0);
}

void AWLSettings::GetOrientation(boost::property_tree::ptree &node, float &pitch, float &yaw, float &roll)
{
	pitch = node.get<float>("pitch", (float)0.0);
	yaw = node.get<float>("yaw", (float)0.0);
	roll = node.get<float>("roll", (float)0.0);
}

void AWLSettings::Get2DPoint(boost::property_tree::ptree &node, float &x, float &y)
{
	x = node.get<float>("x", (float)0.0);
	y = node.get<float>("y", (float)0.0);
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
	red = colorNode.get<uint8_t>("red", 255);
	green = colorNode.get<uint8_t>("green", 255);
	blue = colorNode.get<uint8_t>("blue", 255);
}

void AWLSettings::GetAlertConditions(boost::property_tree::ptree &alertNode, AlertCondition &alert)

{
	std::string sAlertType = alertNode.get<std::string>("alertType", "Invalid");
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
	alert.alertChannelMask.wordData = alertNode.get<uint16_t>("alertChannels", 255);
	alert.minRange = alertNode.get<float>("alertMin", -std::numeric_limits<float>::max());
	alert.maxRange = alertNode.get<float>("alertMax", std::numeric_limits<float>::max());
	alert.threatLevel = (AlertCondition::ThreatLevel) (alertNode.get<int>("alertLevel", AlertCondition::eThreatNone));
}

void AWLSettings::GetChannelGeometry(boost::property_tree::ptree &channelGeometryNode, ReceiverSettings *receiverPtr)
{
	// All channel info for the receiver
	int channelQty = channelGeometryNode.get<int>("channelQty",-1);
	
	// if no "channelPerChannel" description, try array Description.
	if (channelQty == -1)
	{
		GetChannelGeometryArray(channelGeometryNode, receiverPtr);
		return;
	}

	receiverPtr->channelsConfig.resize(channelQty);

	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{

		std::string channelKey = std::string("channel") + std::to_string(channelIndex);

		boost::property_tree::ptree &channelNode = channelGeometryNode.get_child(channelKey);

		ChannelConfig *channelConfigPtr = &receiverPtr->channelsConfig[channelIndex];
		channelConfigPtr->channelIndex = channelIndex;
		Get2DPoint(channelNode.get_child("fov"), channelConfigPtr->fovWidth, channelConfigPtr->fovHeight);
		channelConfigPtr->maxRange = channelNode.get<float>("maxRange", std::numeric_limits<float>::max());

		GetColor(channelNode.get_child("displayColor"),
			channelConfigPtr->displayColorRed, channelConfigPtr->displayColorGreen, channelConfigPtr->displayColorBlue);
	} // for (int channelIndex = 0;

}

void AWLSettings::GetChannelGeometryArray(boost::property_tree::ptree &channelGeometryNode, ReceiverSettings *receiverPtr)

{
	float columnsFloat;
	float rowsFloat;
	int columns;
	int rows;
	float fovX;
	float fovY;
	float spacingX;
	float spacingY;
	float offsetX;
	float offsetY;

	float maxRange, maxAscanRange;
	uint8_t displayColorRed;
	uint8_t displayColorGreen;
	uint8_t displayColorBlue;


	// Range Wraparound trick
	Get2DPoint(channelGeometryNode.get_child("arraySize"), columnsFloat, rowsFloat);
	columns = (int) columnsFloat;
	rows = (int) rowsFloat;

	Get2DPoint(channelGeometryNode.get_child("arrayFOV"), fovX, fovY);
	Get2DPoint(channelGeometryNode.get_child("pixelSpacing"), spacingX, spacingY);
	Get2DPoint(channelGeometryNode.get_child("arrayOffset"), offsetX, offsetY);

	maxRange = channelGeometryNode.get<float>("maxRange", 60.0);
  maxAscanRange = channelGeometryNode.get<float>("maxAscanRange", 60.0);

	GetColor(channelGeometryNode.get_child("displayColor"), displayColorRed, displayColorGreen, displayColorBlue);

	float pixelWidth = (fovX - ((columns - 1)*spacingX)) / columns;
	float pixelHeight = (fovY - ((rows - 1)*spacingY)) / rows;

	int channelQty = columns * rows;
	receiverPtr->channelsConfig.resize(channelQty);


	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		int row = channelIndex / columns;
		std::string sColorKey = std::string("displayColorLine") + std::to_string(row);
		GetColor(channelGeometryNode.get_child(sColorKey), displayColorRed, displayColorGreen, displayColorBlue);

		ChannelConfig *channelConfigPtr = &receiverPtr->channelsConfig[channelIndex];
		channelConfigPtr->channelIndex = channelIndex;

		channelConfigPtr->fovWidth = pixelWidth;
		channelConfigPtr->fovHeight = pixelHeight;

		channelConfigPtr->maxRange = maxRange;
    channelConfigPtr->maxAscanRange = maxAscanRange;
		channelConfigPtr->displayColorRed = displayColorRed;
		channelConfigPtr->displayColorGreen = displayColorGreen;
		channelConfigPtr->displayColorBlue = displayColorBlue;
	} // for (int channelIndex = ;
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
	colorNode.put<uint8_t>("red", red);
	colorNode.put<uint8_t>("green", green);
	colorNode.put<uint8_t>("blue", blue);
}


void AWLSettings::PutChannelGeometry(boost::property_tree::ptree &channelGeometryNode, ReceiverSettings *receiverPtr)
{
	int channelQty = receiverPtr->channelsConfig.size();
	// All channel info for the receiver
	channelGeometryNode.put<int>("channelQty", channelQty);

	// Range Wraparound trick

	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		std::string channelKey = std::string("channel") + std::to_string(channelIndex);

		boost::property_tree::ptree &channelNode = channelGeometryNode.put_child(channelKey, boost::property_tree::ptree(""));

		ChannelConfig *channelConfigPtr = &receiverPtr->channelsConfig[channelIndex];
		channelConfigPtr->channelIndex = channelIndex;
		Put2DPoint(channelNode.put_child("fov", boost::property_tree::ptree("")), channelConfigPtr->fovWidth, channelConfigPtr->fovHeight);
		channelConfigPtr->maxRange = channelNode.get<float>("maxRange");

		PutColor(channelNode.put_child("displayColor", boost::property_tree::ptree("")),
			channelConfigPtr->displayColorRed, channelConfigPtr->displayColorGreen, channelConfigPtr->displayColorBlue);
	} // for (int channelIndex = 0;
}
