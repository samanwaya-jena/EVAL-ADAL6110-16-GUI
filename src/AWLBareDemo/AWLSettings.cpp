/* AWLSettings .cpp */
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
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

using namespace awl;
SENSORCORE_USE_NAMESPACE

const std::string sDefaultAWLSettingsFileName("AWLSimpleDemoSettings.xml");



AWLSettings::AWLSettings(const std::string sSettingsFileName) :
	SensorSettings(sSettingsFileName)
	
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

AWLSettings* AWLSettings::InitSettings(const std::string sSettingsFileName)
{
	if (globalSettings)
	{
		delete globalSettings;
	}

	globalSettings = (SensorSettings*) new AWLSettings(sDefaultAWLSettingsFileName);
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

