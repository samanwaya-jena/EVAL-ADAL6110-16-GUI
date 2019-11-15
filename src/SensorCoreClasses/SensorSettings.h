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

#ifndef _SensorSettings__H
#define _SensorSettings__H

#include <string>
#include <stdint.h>
#include <boost/container/vector.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "SensorCoreClassesGlobal.h"
#include "CoordinateSystem.h"
#include "DetectionStruct.h"

#define VelocityToKmH(velocity) (velocity * 3.6)

SENSORCORE_BEGIN_NAMESPACE

	typedef struct ChannelConfig 
	{
		int channelIndex;
		float fovWidth;
		float fovHeight;

		float maxRange;
		float maxAscanRange;
		uint8_t displayColorRed;
		uint8_t displayColorGreen;
		uint8_t displayColorBlue;
	}
	ChannelConfig;

	typedef boost::container::vector<ChannelConfig> ChannelConfigVector;


	typedef enum {
		eVelocityUnitsMS = 0,
		eVelocityUnitsKMH = 1
	}
	VelocityUnits;

	typedef struct ReceiverSettings
	{
	// Channel configuration
	ChannelConfigVector channelsConfig;

	// Receiver
	std::string sReceiverType;
	std::string sReceiverRegisterSet;
	std::string sReceiverChannelGeometry;


	float displayedRangeMin;
	float displayedRangeMax;
	}
	ReceiverSettings;

	typedef boost::container::vector<ReceiverSettings> ReceiverSettingsVector;

typedef struct CameraSettings
	{
	// Camera
	std::string sCameraName;
	std::string sCameraAPI;
	bool cameraFlip;

	CameraCalibration calibration;
	float cameraFovWidthDegrees;
	float cameraFovHeightDegrees;
	float barrelK1;
	float barrelK2;
	}
	CameraSettings;

	typedef boost::container::vector<CameraSettings> CameraSettingsVector;

	class SensorSettings
	{
	public:
		static SensorSettings* InitSettings(const std::string sInSettingsFileName = std::string(""));
		static SensorSettings* GetGlobalSettings();

		// Constructor
		SensorSettings(const std::string sInSettingsFileName = std::string(""));
		bool ReadSettings();
		boost::property_tree::ptree& GetPropTree() { return (propTree); };


		// Get/Set methods

		static void GetPosition(boost::property_tree::ptree& node, float& forward, float& left, float& up);

		static void GetOrientation(boost::property_tree::ptree& node, float& pitch, float& yaw, float& roll);
		static void Get2DPoint(boost::property_tree::ptree& node, float& x, float& y);

		static void GetGeometry(boost::property_tree::ptree& geometryNode, float& forward, float& left, float& up, float& pitch, float& yaw, float& roll);
		static void GetColor(boost::property_tree::ptree& colorNodeNode, uint8_t& red, uint8_t& green, uint8_t& blue);
		static void GetAlertConditions(boost::property_tree::ptree& alertNode, AlertCondition& alert);

		static void GetChannelGeometry(boost::property_tree::ptree& channelGeometryNode, ReceiverSettings* receiverPtr);
		static void GetChannelGeometryArray(boost::property_tree::ptree& channelGeometryNode, ReceiverSettings* receiverPtr);


		static void PutPosition(boost::property_tree::ptree& node, float forward, float left, float up);
		static void PutOrientation(boost::property_tree::ptree& node, float pitch, float yaw, float roll);
		static void Put2DPoint(boost::property_tree::ptree& node, float x, float y);
		static void PutGeometry(boost::property_tree::ptree& geometryNode, float forward, float left, float up, float pitch, float yaw, float roll);
		static void PutColor(boost::property_tree::ptree& colorNode, uint8_t red, uint8_t green, uint8_t blue);

		static void PutChannelGeometry(boost::property_tree::ptree& channelGeometryNode, ReceiverSettings* receiverPtr);

		bool SetLogAndDebugFilePath(std::string newFilePath);
		std::string GetLogAndDebugFilePath();
		bool SetLogFileName(std::string newFileName);
		std::string GetLogFileName();

		/** \brief Stores the current receiver calibration settings
			* \return true if storage processe dwithout error. False in case of a storage error.
		  */
		  //bool SensorSettings::StoreReceiverCalibration();
		  //Linux
		bool StoreReceiverCalibration();



	public:
		// Receiver configuration
		ReceiverSettingsVector receiverSettings;

		// Camera configuration
		CameraSettingsVector cameraSettings;

		// Dynamic testing
		float brakingDeceleration;
		float travelSpeed;

		// Debug
		bool bWriteDebugFile;
		bool bWriteLogFile;
		std::string sDebugFileName;
		std::string sLogFileName;
		std::string sDebugAndLogFilePath;


	protected:
		std::string sSettingsFileName;
		static SensorSettings* globalSettings;
		// Property tree contains all the information from the configuration file
		boost::property_tree::ptree propTree;
	};

SENSORCORE_END_NAMESPACE          

#endif 
