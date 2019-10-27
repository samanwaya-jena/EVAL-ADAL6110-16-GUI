/* LoopedWorker.cpp */
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

#include "SensorCoreClassesGlobal.h"
#include "LoopedWorker.h"

SENSORCORE_USE_NAMESPACE

LoopedWorker::LoopedWorker():
mWorkerRunning(false)
{

}

LoopedWorker::~LoopedWorker()
{
	if (mWorkerRunning) Stop();
}

void LoopedWorker::Go() 
	
{
    mWorkerRunning = true;
}
 

void  LoopedWorker::Stop() 
{
	if (!mWorkerRunning) return;
    mWorkerRunning = false;
}


bool  LoopedWorker::WasStopped()
{
	if (!mWorkerRunning) return(true);
	return(false);
}

#if 0
// Sample code for a LoopedWorker loop iteration
bool LoopedWorker::SpinOnce()

{

	if (WasStopped()) return(false);
	...
	...
	if (someStopCondition)
	{
		Stop();
		return(false);
	}

	return(true);
}
#endif
