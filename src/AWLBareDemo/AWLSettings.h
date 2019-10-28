#ifndef _AWLSettings__H
#define _AWLSettings__H

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

#include "CoordinateSystem.h"
#include "DetectionStruct.h"
#include "SensorSettings.h"

#define VelocityToKmH(velocity) (velocity * 3.6)

namespace awl
{


	class AWLSettings : public SensorCoreScope::SensorSettings
	{
	public:
		static AWLSettings * InitSettings(const std::string sSettingsFileName = std::string(""));
		static AWLSettings* GetGlobalSettings();

		// Constructor
		AWLSettings(const std::string sSettingsFileName = std::string(""));
		bool ReadSettings();
	};

} // namespace AWL          

#endif 
