#include "LoopedWorker.h"

using namespace std;
using namespace awl;



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
