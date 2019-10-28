#ifndef SENSORCORE_SENSORCLASSESESGLOBAL_H
#define SENSORCORE_SENSORCLASSESESGLOBAL_H

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

#ifdef __cplusplus


#if !defined(SENSORCORE_NAMESPACE) /* user namespace */

# define SENSORCORE_USE_NAMESPACE
# define SENSORCORE_BEGIN_NAMESPACE
# define SENSORCORE_END_NAMESPACE
# define SENSORCORE_NAMESPACE_PREFIX 
# define SensorCoreScope

#else /* user namespace */

# define SENSORCORE_USE_NAMESPACE using namespace ::SENSORCORE_NAMESPACE;
# define SENSORCORE_BEGIN_NAMESPACE namespace SENSORCORE_NAMESPACE {
# define SENSORCORE_END_NAMESPACE }
# define SENSORCORE_NAMESPACE_PREFIX SENSORCORE_NAMESPACE 
# define SensorCoreScope SENSORCORE_NAMESPACE

namespace SENSORCORE_NAMESPACE {}

#endif /* user namespace */

#else /* __cplusplus */

# define SENSORCORE_USE_NAMESPACE
# define SENSORCORE_BEGIN_NAMESPACE
# define SENSORCORE_END_NAMESPACE
	
# define SENSORCORE_BEGIN_INCLUDE_NAMESPACE
# define SENSORCORE_END_INCLUDE_NAMESPACE
# define SENSORCORE_NAMESPACE_PREFIX 
# define SensorCoreScope

#endif /* __cplusplus */

#endif //SENSORCORE_SENSORCLASSESESGLOBAL_H
