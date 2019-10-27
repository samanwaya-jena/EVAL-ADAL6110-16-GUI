#ifndef _SensorSettings__H
#define _SensorSettings__H

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
		static SensorSettings* InitSettings(const std::string sSettingsFileName = std::string(""));
		static SensorSettings* GetGlobalSettings();

		// Constructor
		SensorSettings(const std::string sSettingsFileName = std::string(""));
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


	protected:
		std::string sFileName;
		static SensorSettings* globalSettings;
		// Property tree contains all the information from the configuration file
		boost::property_tree::ptree propTree;
	};

SENSORCORE_END_NAMESPACE          

#endif 
