/* AWLSettings.cpp: CuteApplication appllication-specific settings */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the CuteApplication of the
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

#include "DetectionStruct.h"
#include "AWLSettings.h"

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>


using namespace awl;
SENSORCORE_USE_NAMESPACE

const std::string sDefaultAWLSettingsFileName("AWLDemoSettings.xml");

AWLSettings::AWLSettings(const std::string sInSettingsFileName) :
SensorSettings(sInSettingsFileName),
sLogoFileName(""),
sIconFileName(""),
bDisplayVideoCrosshair(false),
bDisplayVideoTime(false)

{
	if (sInSettingsFileName.empty())
	{
		sSettingsFileName.assign(sDefaultAWLSettingsFileName);
	}
	else 
	{
		sSettingsFileName.assign(sInSettingsFileName);
	}
}

AWLSettings * AWLSettings::InitSettings(const std::string sInSettingsFileName)
{
	if (globalSettings)
	{
		delete globalSettings;
	}

	globalSettings = (SensorSettings *) new AWLSettings(sInSettingsFileName);
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

	displayedDetectionsPerVoxelInTableView = propTree.get<int>("config.displayTableView.displayedDetectionsPerVoxelInTableView", 2);

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

