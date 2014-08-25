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
