/* SensorSettings.cpp : XML Settings File Management*/
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include "SensorCoreClassesGlobal.h"
#include "SensorSettings.h"
#include "DetectionStruct.h"

SENSORCORE_USE_NAMESPACE

const std::string sDefaultSettingsFileName("SensorDemoSettings.xml");

/*************************************************************************************************************************************************************************/

SensorSettings * SensorSettings::globalSettings = NULL;

SensorSettings::SensorSettings(const std::string sInSettingsFileName)

{
	if (sInSettingsFileName.empty())
	{
		sSettingsFileName.assign(sDefaultSettingsFileName);
	}
	else
	{
		sSettingsFileName.assign(sInSettingsFileName);
	}
}

SensorSettings* SensorSettings::InitSettings(const std::string sInSettingsFileName)
{
	if (globalSettings)
	{
		delete globalSettings;
	}

	globalSettings = new SensorSettings(sInSettingsFileName);
	return(globalSettings);
}

SensorSettings* SensorSettings::GetGlobalSettings()
{
	return(globalSettings);
}



bool SensorSettings::ReadSettings()
{
	// Create an empty property tree object
	using boost::property_tree::ptree;
	propTree.clear();

	// Load the XML file into the property tree. If reading fails
	// (cannot open file, parse error), an exception is thrown.
	read_xml(sSettingsFileName, propTree);

	// Debug and log file control
	bWriteDebugFile = propTree.get<bool>("config.debug.enableDebugFile", false);
	bWriteLogFile = propTree.get<bool>("config.debug.enableLogFile", false);
	sDebugFileName = propTree.get<std::string>("config.debug.debugFileName", "debug.dbg");
	sLogFileName = propTree.get<std::string>("config.debug.logFileName", "distanceLog.csv");
	sDebugAndLogFilePath = propTree.get<std::string>("config.debug.defaultPath", "./");

	// Receivers

	int receiverQty = propTree.get<int>("config.receivers.receiverQty", 0);
	receiverSettings.resize(receiverQty);
	
	for (int receiverIndex = 0; receiverIndex < receiverQty; receiverIndex++)
	{
		std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverIndex);

		boost::property_tree::ptree& receiverNode = propTree.get_child(receiverKey);

		ReceiverSettings* receiverPtr = &receiverSettings[receiverIndex];
		receiverPtr->sReceiverType = receiverNode.get<std::string>("receiverType");
		receiverPtr->sReceiverRegisterSet = receiverNode.get<std::string>("receiverRegisterSet");
		receiverPtr->sReceiverVoxelGeometry = receiverNode.get<std::string>("receiverVoxelGeometry");

		// Display
		receiverPtr->displayedRangeMin = receiverNode.get<float>("displayedRangeMin", (float)0.0);
		receiverPtr->displayedRangeMax = receiverNode.get<float>("displayedRangeMax", (float)60.0);

		// Get the Channel configuration Node
		std::string voxelGeometryKey = "config." + receiverPtr->sReceiverVoxelGeometry;
		boost::property_tree::ptree& voxelGeometryNode = propTree.get_child(voxelGeometryKey);
		GetVoxelGeometry(voxelGeometryNode, receiverPtr);
	} // for (int receiverIndex = 0; 


	int cameraQty = propTree.get<int>("config.cameras.cameraQty", 0);
	cameraSettings.resize(cameraQty);
	for (int cameraIndex = 0; cameraIndex < cameraQty; cameraIndex++)
	{
		std::string cameraKey = std::string("config.cameras.camera") + std::to_string(cameraIndex);

		boost::property_tree::ptree& cameraNode = propTree.get_child(cameraKey);
		CameraSettings* cameraPtr = &cameraSettings[cameraIndex];

		cameraPtr->sCameraName = cameraNode.get<std::string>("cameraName", "0");
		cameraPtr->sCameraAPI = cameraNode.get<std::string>("cameraAPI", "OpenCV");
		cameraPtr->cameraFlip = cameraNode.get<bool>("cameraFlip", false);
		Get2DPoint(cameraNode.get_child("fov"), cameraPtr->cameraFovWidthDegrees, cameraPtr->cameraFovHeightDegrees);
		cameraPtr->barrelK1 = 0.0;
		cameraPtr->barrelK2 = 0.0;

		cameraPtr->barrelK1 = cameraNode.get<float>("barrelCorrectionK1", 0.0);
		cameraPtr->barrelK2 = cameraNode.get<float>("barrelCorrectionK2", 0.0);
	}

	brakingDeceleration = propTree.get<float>("config.dynamicTesting.brakingDeceleration", (float)5.096);
	travelSpeed = propTree.get<float>("config.dynamicTesting.travelSpeed", (float)33.32);

	// Alert conditions
	int alertConditionQty = propTree.get<int>("config.dynamicTesting.alertQty", 0);
	for (int alertConditionIndex = 0; alertConditionIndex < alertConditionQty; alertConditionIndex++)
	{
		std::string alertKey = std::string("config.dynamicTesting.alert") + std::to_string(alertConditionIndex);

		boost::property_tree::ptree& alertNode = propTree.get_child(alertKey);
		AlertCondition::Ptr newAlert = AlertCondition::Ptr(new AlertCondition());
		GetAlertConditions(alertNode, *newAlert);
		AlertCondition::Store(newAlert);
	}

	return(true);
}

bool SensorSettings::StoreReceiverCalibration()

{
	// Create an empty property tree object
	using boost::property_tree::ptree;
	ptree localPropTree;

	// Load the XML file into the property tree. If reading fails
	// (cannot open file, parse error), an exception is thrown.
	read_xml(sSettingsFileName, localPropTree);


#if 0
	// Loop for all Receiver Configurations
	BOOST_FOREACH(ptree::value_type & receiversNode, localPropTree.get_child("config.receivers"))
	{
		if (receiversNode.first == "receiver")
		{
			boost::property_tree::ptree& receiverNode = receiversNode.second;
			ReceiverSettings receiver;

			receiverNode.put<std::string>("receiverType", receiver.sReceiverType);
			receiverNode.put<std::string>("receiverRegisterSet", receiver.sReceiverRegisterSet);
			receiverNode.put<std::string>("receiverVoxelGeometry", receiver.sReceiverGeometry);


			receiverNode.put<uint16_t>("voxelMask", receiver.receiverVoxelMask);
			receiverNode.put<uint16_t>("frameRate", (uint16_t) receiver.receiverStatus.demandedFrameRate);

			// Geometry
			boost::property_tree::ptree& geometryNode = receiverNode.get_child("sensorGeometry");
			PutGeometry(geometryNode,
				receiver.sensorForward, receiver.sensorLeft, receiver.sensorUp,
				receiver.sensorPitch, receiver.sensorYaw, receiver.sensorRoll);

			// Display
			receiverNode.put<float>("displayedRangeMin", receiver.displayedRangeMin);
			receiverNode.put<float>("displayedRangeMax", receiver.displayedRangeMax);
			receiverNode.put<float>("rangeOffset", receiver.rangeOffset);

			// All voxel info for the receiver
			BOOST_FOREACH(ptree::value_type & voxelsNode, receiverNode)
			{
				if (voxelsNode.first == "voxel")
				{
					boost::property_tree::ptree& voxelNode = voxelsNode.second;
					VoxelConfig voxelConfig;

					voxelNode.put<int>("index", voxelConfig.voxelIndex);
					Put2DPoint(voxelNode.get_child("fov"), voxelConfig.fovWidth, voxelConfig.fovHeight);
					float roll = 0.0;
					PutOrientation(voxelNode.get_child("orientation"), voxelConfig.centerY, voxelConfig.centerX, roll);
					voxelNode.put<float>("maxRange", voxelConfig.maxRange);

					PutColor(voxelNode.get_child("displayColor"),
						voxelConfig.displayColorRed, voxelConfig.displayColorGreen, voxelConfig.displayColorBlue);

					voxelsNode->add_child(voxelNode);
				}// if( receiversNode.first == "voxel"

			} // BOOST_FOREACH(ptree::value_type &voxelsNode
			receiverNode.put_child(voxelsNode);
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


void SensorSettings::GetPosition(boost::property_tree::ptree& node, float& forward, float& left, float& up)
{
	forward = node.get<float>("forward", (float)0.0);
	left = node.get<float>("left", (float)0.0);
	up = node.get<float>("up", (float)0.0);
}

void SensorSettings::GetOrientation(boost::property_tree::ptree& node, float& pitch, float& yaw, float& roll)
{
	pitch = node.get<float>("pitch", (float)0.0);
	yaw = node.get<float>("yaw", (float)0.0);
	roll = node.get<float>("roll", (float)0.0);
}

void SensorSettings::Get2DPoint(boost::property_tree::ptree& node, float& x, float& y)
{
	x = node.get<float>("x", (float)0.0);
	y = node.get<float>("y", (float)0.0);
}

void SensorSettings::GetGeometry(boost::property_tree::ptree& geometryNode, float& forward, float& left, float& up, float& pitch, float& yaw, float& roll)
{
	boost::property_tree::ptree& positionNode = geometryNode.get_child("position");
	boost::property_tree::ptree& orientationNode = geometryNode.get_child("orientation");
	GetPosition(positionNode, forward, left, up);
	GetOrientation(orientationNode, pitch, yaw, roll);
}

void SensorSettings::GetColor(boost::property_tree::ptree& colorNode, uint8_t& red, uint8_t& green, uint8_t& blue)
{
	red = colorNode.get<uint8_t>("red", 255);
	green = colorNode.get<uint8_t>("green", 255);
	blue = colorNode.get<uint8_t>("blue", 255);
}

void SensorSettings::GetAlertConditions(boost::property_tree::ptree& alertNode, AlertCondition& alert)

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
	alert.alertVoxelMask.wordData = alertNode.get<uint16_t>("alertVoxel", 255);
	alert.minRange = alertNode.get<float>("alertMin", -std::numeric_limits<float>::max());
	alert.maxRange = alertNode.get<float>("alertMax", std::numeric_limits<float>::max());
	alert.threatLevel = (AlertCondition::ThreatLevel) (alertNode.get<int>("alertLevel", AlertCondition::eThreatNone));
}

void SensorSettings::GetVoxelGeometry(boost::property_tree::ptree& voxelGeometryNode, ReceiverSettings* receiverPtr)
{
	// All voxel info for the receiver
	int voxelQty = voxelGeometryNode.get<int>("voxelQty", -1);

	// if no "voxel Per voxel" description, try array Description.
	if (voxelQty == -1)
	{
		GetVoxelGeometryArray(voxelGeometryNode, receiverPtr);
		return;
	}

	receiverPtr->voxelsConfig.resize(voxelQty);
	receiverPtr->receiverRows = 1;
	receiverPtr->receiverColumns = voxelQty;


	for (int voxelIndex = 0; voxelIndex < voxelQty; voxelIndex++)
	{

		std::string voxelKey = std::string("voxels") + std::to_string(voxelIndex);

		boost::property_tree::ptree& voxelNode = voxelGeometryNode.get_child(voxelKey);

		VoxelConfig* voxelConfigPtr = &receiverPtr->voxelsConfig[voxelIndex];
		voxelConfigPtr->voxelIndex = voxelIndex;
		Get2DPoint(voxelNode.get_child("fov"), voxelConfigPtr->fovWidth, voxelConfigPtr->fovHeight);
		voxelConfigPtr->maxRange = voxelNode.get<float>("maxRange", std::numeric_limits<float>::max());

		GetColor(voxelNode.get_child("displayColor"),
			voxelConfigPtr->displayColorRed, voxelConfigPtr->displayColorGreen, voxelConfigPtr->displayColorBlue);
	} // for (int voxelIndex = 0;

}

void SensorSettings::GetVoxelGeometryArray(boost::property_tree::ptree& voxelGeometryNode, ReceiverSettings* receiverPtr)

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
	Get2DPoint(voxelGeometryNode.get_child("arraySize"), columnsFloat, rowsFloat);
	columns = (int)columnsFloat;
	rows = (int)rowsFloat;

	Get2DPoint(voxelGeometryNode.get_child("arrayFOV"), fovX, fovY);
	Get2DPoint(voxelGeometryNode.get_child("pixelSpacing"), spacingX, spacingY);
	Get2DPoint(voxelGeometryNode.get_child("arrayOffset"), offsetX, offsetY);

	maxRange = voxelGeometryNode.get<float>("maxRange", 60.0);
	maxAscanRange = voxelGeometryNode.get<float>("maxAscanRange", 60.0);

	GetColor(voxelGeometryNode.get_child("displayColor"), displayColorRed, displayColorGreen, displayColorBlue);

	float pixelWidth = (fovX - ((columns - 1) * spacingX)) / columns;
	float pixelHeight = (fovY - ((rows - 1) * spacingY)) / rows;

	int voxelQty = columns * rows;
	receiverPtr->voxelsConfig.resize(voxelQty);
	receiverPtr->receiverColumns = columns;
	receiverPtr->receiverRows = rows;


	for (int voxelIndex = 0; voxelIndex < voxelQty; voxelIndex++)
	{
		int row = voxelIndex / columns;
		std::string sColorKey = std::string("displayColorLine") + std::to_string(row);
		GetColor(voxelGeometryNode.get_child(sColorKey), displayColorRed, displayColorGreen, displayColorBlue);

		VoxelConfig* voxelConfigPtr = &receiverPtr->voxelsConfig[voxelIndex];
		voxelConfigPtr->voxelIndex = voxelIndex;

		voxelConfigPtr->fovWidth = pixelWidth;
		voxelConfigPtr->fovHeight = pixelHeight;

		voxelConfigPtr->maxRange = maxRange;
		voxelConfigPtr->maxAscanRange = maxAscanRange;
		voxelConfigPtr->displayColorRed = displayColorRed;
		voxelConfigPtr->displayColorGreen = displayColorGreen;
		voxelConfigPtr->displayColorBlue = displayColorBlue;
	} // for (int voxelIndex = ;
}

void SensorSettings::PutPosition(boost::property_tree::ptree& node, float forward, float left, float up)
{
	node.put<float>("forward", forward);
	node.put<float>("left", left);
	node.put<float>("up", up);
}

void SensorSettings::PutOrientation(boost::property_tree::ptree& node, float pitch, float yaw, float roll)
{
	node.put<float>("pitch", pitch);
	node.put<float>("yaw", yaw);
	node.put<float>("roll", roll);
}

void SensorSettings::Put2DPoint(boost::property_tree::ptree& node, float x, float y)
{
	node.put<float>("x", x);
	node.put<float>("y", y);
}

void SensorSettings::PutGeometry(boost::property_tree::ptree& geometryNode, float forward, float left, float up, float pitch, float yaw, float roll)
{
	boost::property_tree::ptree& positionNode = geometryNode.put_child("position", boost::property_tree::ptree(""));
	boost::property_tree::ptree& orientationNode = geometryNode.put_child("orientation", boost::property_tree::ptree(""));
	PutPosition(positionNode, forward, left, up);
	PutOrientation(orientationNode, pitch, yaw, roll);
}

void SensorSettings::PutColor(boost::property_tree::ptree& colorNode, uint8_t red, uint8_t green, uint8_t blue)
{
	colorNode.put<uint8_t>("red", red);
	colorNode.put<uint8_t>("green", green);
	colorNode.put<uint8_t>("blue", blue);
}


void SensorSettings::PutChannelGeometry(boost::property_tree::ptree& voxelGeometryNode, ReceiverSettings* receiverPtr)
{
	int voxelQty = receiverPtr->voxelsConfig.size();
	// All voxel info for the receiver
	voxelGeometryNode.put<int>("voxelQty", voxelQty);

	// Range Wraparound trick

	for (int voxelIndex = 0; voxelIndex < voxelQty; voxelIndex++)
	{
		std::string voxelKey = std::string("voxel") + std::to_string(voxelIndex);

		boost::property_tree::ptree& voxelNode = voxelGeometryNode.put_child(voxelKey, boost::property_tree::ptree(""));

		VoxelConfig* voxelConfigPtr = &receiverPtr->voxelsConfig[voxelIndex];
		voxelConfigPtr->voxelIndex = voxelIndex;
		Put2DPoint(voxelNode.put_child("fov", boost::property_tree::ptree("")), voxelConfigPtr->fovWidth, voxelConfigPtr->fovHeight);
		voxelConfigPtr->maxRange = voxelNode.get<float>("maxRange");

		PutColor(voxelNode.put_child("displayColor", boost::property_tree::ptree("")),
			voxelConfigPtr->displayColorRed, voxelConfigPtr->displayColorGreen, voxelConfigPtr->displayColorBlue);
	} // for (int voxelIndex = 0;
}

bool SensorSettings::SetLogAndDebugFilePath(std::string newFilePath)
{
	sDebugAndLogFilePath = newFilePath;
	return(true);
}

std::string SensorSettings::GetLogAndDebugFilePath()
{
	return(sDebugAndLogFilePath);
}

bool SensorSettings::SetLogFileName(std::string newFileName)
{
	sLogFileName = newFileName;
	return(true);
}

std::string SensorSettings::GetLogFileName()
{
	return(sLogFileName);
}

