#define CV_NO_BACKWARD_COMPATIBILITY

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <cstdio>

#include "tracker.h"
#include "ReceiverCapture.h"
#include "ReceiverFileCapture.h"

using namespace std;
using namespace pcl;
using namespace awl;

ReceiverFileCapture::ReceiverFileCapture(int inReceiverID, int inSequenceID, int inReceiverChannelQty, std::string inFileName):
ReceiverCapture(inReceiverID, inSequenceID, inReceiverChannelQty)

{
	acquisitionSequence->ReadFile(inFileName, inReceiverChannelQty, inDetectionsPerChannel);
}


ReceiverFileCapture::~ReceiverFileCapture()
{
	Stop();
}


void  ReceiverFileCapture::Go(bool inIsThreaded) 
{
	bIsThreaded = inIsThreaded;
	assert(!mThread);
    mStopRequested = false;

	if (bIsThreaded) 
		{
		mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverFileCapture::DoThreadLoop, this)));
#if 0
		// Set the priority under windows.  This is the most critical display thread 
		// for user interaction
	

		 HANDLE th = mThread->native_handle();
		//   SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
		//   SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif
	}
}
 

void  ReceiverFileCapture::Stop() 
{
	if (mStopRequested) return;
    mStopRequested = true;
	if (bIsThreaded) {
		bIsThreaded = false;
		assert(mThread);
		mThread->join();
	}
}

bool  ReceiverFileCapture::WasStopped()
{
	if (mStopRequested) return(true);
	return(false);
}

void ReceiverFileCapture::DoThreadLoop()

{
	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverFileCapture::DoThreadIteration()

{
	if (!bIsThreaded)
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverFileCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

		
		// Take the frame at the front and roll over at the end of the queue
		SensorFrame::Ptr frontFrame(acquisitionSequence->sensorFrames.front());
		acquisitionSequence->sensorFrames.push(frontFrame);
		frontFrame->frameID = acquisitionSequence->AllocateFrameID();
		acquisitionSequence->sensorFrames.pop();

		currentReceiverCaptureSubscriptions->PutNews();

		rawLock.unlock();

//		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		boost::this_thread::sleep(boost::posix_time::milliseconds(300));
	} // while (!WasStoppped)
}



