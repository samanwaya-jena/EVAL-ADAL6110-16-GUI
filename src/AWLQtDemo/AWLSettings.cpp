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

#include "DetectionStruct.h"
#include "AWLSettings.h"


#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>


using namespace awl;
using namespace std;

const std::string sDefaultAWLSettingsFileName("AWLDemoSettings.xml");

AWLSettings::AWLSettings(const std::string sSettingsFileName) :
SensorSettings(sSettingsFileName),
sLogoFileName(""),
sIconFileName(""),
bDisplayVideoCrosshair(false),
bDisplayVideoTime(false)

{
	if (sSettingsFileName.empty())
	{
		sFileName.assign(sDefaultAWLSettingsFileName);
	}
	else 
	{
		sFileName.assign(sSettingsFileName);
	}
}

AWLSettings * AWLSettings::InitSettings(const std::string sSettingsFileName)
{
	if (globalSettings)
	{
		delete globalSettings;
	}

	globalSettings = (SensorSettings *) new AWLSettings(sDefaultAWLSettingsFileName);
	return( (AWLSettings*) globalSettings);
}

AWLSettings *AWLSettings::GetGlobalSettings()
{
	return((AWLSettings *)globalSettings);
}



bool AWLSettings::ReadSettings()
{
	SensorSettings::ReadSettings();

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

	bDisplayVideoCrosshair = propTree.get<bool>("config.video.displayCrosshair", false);
	bDisplayVideoTime = propTree.get<bool>("config.video.displayTime", true);

	return(true);
}

