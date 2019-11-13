/* AWLSettings .cpp */
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

#include "AWLSettings.h"
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

using namespace awl;
SENSORCORE_USE_NAMESPACE

const std::string sDefaultAWLSettingsFileName("AWLSimpleDemoSettings.xml");



AWLSettings::AWLSettings(const std::string sInSettingsFileName) :
	SensorSettings(sSettingsFileName)
	
{
	if (sSettingsFileName.empty())
	{
		sSettingsFileName.assign(sDefaultAWLSettingsFileName);
	}
	else
	{
		sSettingsFileName.assign(sInSettingsFileName);
	}
}

AWLSettings* AWLSettings::InitSettings(const std::string sInSettingsFileName)
{
	if (globalSettings)
	{
		delete globalSettings;
	}

	globalSettings = (SensorSettings*) new AWLSettings(sInSettingsFileName);
	return((AWLSettings*)globalSettings);
}

AWLSettings* AWLSettings::GetGlobalSettings()
{
	return((AWLSettings*)globalSettings);
}



bool AWLSettings::ReadSettings()
{
	return (SensorSettings::ReadSettings());
}

