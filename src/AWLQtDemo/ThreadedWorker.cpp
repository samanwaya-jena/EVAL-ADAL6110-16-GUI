/*
	Copyright 2014 Aerostar R&D Canada Inc.

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

#include "ThreadedWorker.h"

using namespace std;
using namespace awl;

ThreadedWorker::ThreadedWorker():
mWorkerRunning(false)

{

}

ThreadedWorker::~ThreadedWorker()
{
	if (mWorkerRunning) Stop();
}

void ThreadedWorker::Go() 
	
{
	assert(!mThread);
    mWorkerRunning = true;

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ThreadedWorker::DoThreadLoop, this)));
}
 

void  ThreadedWorker::Stop() 
{
	StopThread();
}


bool  ThreadedWorker::WasStopped()
{
	if (!mWorkerRunning) return(true);
	return(false);
}

void  ThreadedWorker::StopThread(bool bCalledFromThread)
{
	if (!mWorkerRunning) return;
    mWorkerRunning = false;

	assert(mThread);
	mThread->join();
}


#if 0
// Sample code for a threadedworker loop
void ThreadedWorker::DoThreadLoop()

{

	while (!WasStopped())
    {
		DoOneThreadIteration();
		if (someCondition)
		{
			StopThread(true);
		}

	} // while (!WasStoppped)
}
#endif
