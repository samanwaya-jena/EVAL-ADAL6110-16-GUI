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

#ifndef _AWLSettings__H
#define _AWLSettings__H

#include <string>
#include <stdint.h>
#include <boost/container/vector.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "CoordinateSystem.h"
#include "DetectionStruct.h"
#include "SensorSettings.h"

namespace awl
{

class AWLSettings: public SensorCoreScope::SensorSettings
{
public:
	static AWLSettings *InitSettings(const std::string sInSettingsFileName = std::string(""));
	static AWLSettings *GetGlobalSettings();

	// Constructor
	AWLSettings(const std::string sInSettingsFileName = std::string(""));
	bool ReadSettings();
	
public:

	// Layout
	bool bDisplaySettingsWindow;
	bool bDisplay2DWindow;
	bool bDisplayTableViewWindow;
	bool bDisplayAScanViewWindow;
	bool bDisplayAboutWindow;
	bool bDisplayCameraWindow;

	bool bTabSettingCalibration;
	bool bTabSettingControl;
	bool bTabSettingStatus;
	bool bTabSettingRegisters;
	bool bTabSettingGPIOs;
	bool bTabSettingAlgoControl;
	bool bTabSettingTrackerControl;
	bool bTabSettingAScan;


	std::string sDisplayShowSize;

	SensorCoreScope::VelocityUnits velocityUnits;

	std::string sLogoFileName;
	std::string sIconFileName;

	// Table view options
	int displayedDetectionsPerVoxelInTableView;

	// 2D display options
	float carWidth;
	float carLength;
	float carHeight;

	float laneWidth;

	bool showPalette;
	int mergeDisplayMode;
	int measureMode;
	int displayDistanceMode2D;
	int displayZoomMode2D;
	float mergeAcceptanceX;
	float mergeAcceptanceY;
	int colorCode2D;
	float maxVelocity2D;
	float zeroVelocity;

	// Video display options
	bool bDisplayVideoCrosshair;
	bool bDisplayVideoTime;

};

} // namespace AWL          

#endif 
