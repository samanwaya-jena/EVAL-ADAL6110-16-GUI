#include <fstream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "awlcoord.h"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#if 1 //defined(HAVE_XIMEA)
#include "xiapi.h"
#endif

using namespace std;
using namespace awl;

// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

const int ximeaDefaultBinningMode  = 4; // Binning mode on the ximea camera for 648x486 resolution


VideoCapture::VideoCapture(int argc, char** argv):
currentFrameSubscriptions(new(Subscription))

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	mStopRequested = false;
	mThreadExited = false;

	// Initialize HighGUI
	cvInitSystem(argc, argv);

	std::string inputName = globalSettings->sCameraName;
	int inputID = 0;
	if (!inputName.empty()) inputID = atoi(inputName.c_str());

	// Determine capture source:  Camera, Single Frame or AVI
    if( inputName.empty() || isdigit(inputName.c_str()[0]) )
	{
		cam.open(inputID);
	}
	else 
	{
		cam.open(inputName);
	}

		
	if (cam.isOpened()) 
	{
		// interpret preferred interface (0 = autodetect). This tells us what type of marea capabilities to expect
		int pref = (inputID / 100) * 100;

		// If we are using the Ximea driver, set the downsampling for a 640x480 image
		if (pref == CV_CAP_XIAPI)
		{
			cam.set(CV_CAP_PROP_XI_DATA_FORMAT, XI_RGB24 );
//			cam.set(CV_CAP_PROP_XI_DOWNSAMPLING_TYPE, XI_SKIPPING );
			cam.set(CV_CAP_PROP_XI_DOWNSAMPLING, ximeaDefaultBinningMode);
		}

		frameWidth = (int) cam.get(CV_CAP_PROP_FRAME_WIDTH);
		frameHeight = (int) cam.get(CV_CAP_PROP_FRAME_HEIGHT);
		double framesPerSecond = cam.get(CV_CAP_PROP_FPS);

		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // CV_CAP_PROP_FPS may reurn 0;
		frameRate = (double) 1.0/framesPerSecond;
	}
	else 
	{
		// Set default values not to be zeroes
		frameWidth = 640;
		frameHeight = 480;
		frameRate = (double) 1.0/30.0;
	}

	// Field of view of the camera are in application seetings. 
	// They are in degrees, so need to be converted in radians.

	cameraFovWidth = DEG2RAD(globalSettings->cameraFovWidthDegrees);
	cameraFovHeight = DEG2RAD(globalSettings->cameraFovHeightDegrees);
}

VideoCapture:: ~VideoCapture()

{
	Stop();
	boost::mutex::scoped_lock threadLock(GetMutex());
	threadLock.unlock();
}

void  VideoCapture::Go() 
{
	assert(!mThread);
    mStopRequested = false;

	mThreadExited = false;

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&VideoCapture::DoThreadLoop, this)));

}
 

void  VideoCapture::Stop() 
{
	if (mStopRequested) return;
	mStopRequested = true;

	if (mStopRequested || mThreadExited) 
	{
		mThreadExited=false;
		assert(mThread);
		mThread->join();
	}	
}


bool  VideoCapture::WasStopped()
{
	if (mStopRequested || mThreadExited) return(true);

	return(false);
}

void VideoCapture::CopyCurrentFrame(VideoCapture::FramePtr targetFrame,  Subscription::SubscriberID inSubscriberID) 
{
	boost::mutex::scoped_lock updateLock(currentFrameSubscriptions->GetMutex());
 	*targetFrame = currentFrame.clone();
	currentFrameSubscriptions->GetNews(inSubscriberID);
	updateLock.unlock();
};


void VideoCapture::DoThreadLoop()

{
	double delay = (frameRate * 1000) / 2;
	if (delay < 1.0) delay = 1;

	while (!WasStopped())
    {

		// Acquire from camera source or AVI
   		boost::mutex::scoped_lock threadLock(GetMutex());
		DoThreadIteration();
		threadLock.unlock();
	}

	if (cam.isOpened()) 
	{
		cam.release();
	}

	mThreadExited = true;
}

void VideoCapture::DoThreadIteration()

{
	double delay = (frameRate * 1000) / 2;
	if (delay < 1.0) delay = 1;

	// Acquire from camera source or AVI
	if( cam.isOpened() )
	{
		cam.read(bufferFrame);
		// cvQuery frame is blocking, while waaiting for the frame.  
		//Check again if we were stopped in the meantime 
		if (WasStopped()) 
		{
			return;
		}

		if (!bufferFrame.empty()) 
		{
#if 1
			// Reset the iplImg dimensions.  This corrects an OpenCV reporting bug with the XIMEA Camera.
			bufferFrame.cols = frameWidth;
			bufferFrame.rows = frameHeight;
			bufferFrame.step = bufferFrame.cols*bufferFrame.channels();
			// End of the Ximea patch
#endif
			boost::mutex::scoped_lock currentLock(currentFrameSubscriptions->GetMutex());
			bufferFrame.copyTo(currentFrame);
			currentFrameSubscriptions->PutNews();
			currentLock.unlock();
		} // if (!bufferFrame.empty()) 
	} // if( cam.isOpened() )
}


