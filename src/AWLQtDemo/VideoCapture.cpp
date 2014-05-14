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

using namespace std;
using namespace awl;

// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

// Define camera resolution = 640x360
VideoCapture::VideoCapture(int argc, char** argv):
currentFrame(new (cv::Mat)), 
bufferFrame(new (cv::Mat)), 
currentFrameSubscriptions(new(Subscription))

{
	mStopRequested = false;
	mThreadExited = false;
	capture = 0;

	// Initialize HighGUI
	cvInitSystem(argc, argv);

   std::string inputName;

	// Determine capture source:  Camera, Single Frame or AVI
    if( inputName.empty() || (isdigit(inputName.c_str()[0]) && inputName.c_str()[1] == '\0') )
	{
        capture = cvCaptureFromCAM( inputName.empty() ? 0 : inputName.c_str()[0] - '0' );
	}
	else if( inputName.size() )
    {
        image = cv::imread( inputName, 1 );
        if( image.empty() )
		{
            capture = cvCaptureFromFile( inputName.c_str() );
#if 0
			if (capture == 0)
			{
			cerr << "Error: invalid input file" << inputName << endl;
			}
#endif
		}
		else 
		{
			(*currentFrame) = image;
		}

    }

	if (capture) 
	{
		frameWidth = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
		frameHeight = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
		double framesPerSecond = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // CV_CAP_PROP_FPS may reurn 0;
		frameRate = (double) 1.0/framesPerSecond;
	}
	else 
	{
		if (!image.empty()) 
		{
			frameWidth = (int) image.cols;
			frameHeight = (int) image.rows;
			frameRate = (double) 1/FRAME_RATE;
		}
	}

	// Field of view of the camera are in application seetings. 
	// They are in degrees, so need to be converted in radians.
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	cameraFovX = DEG2RAD(globalSettings->cameraFovXDegrees);
	cameraFovY = DEG2RAD(globalSettings->cameraFovYDegrees);
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

VideoCapture::FramePtr VideoCapture::GetCurrentFrame() 
{
	return(currentFrame);
};


void VideoCapture::CopyCurrentFrame(VideoCapture::FramePtr targetFrame,  Subscription::SubscriberID inSubscriberID) 
{
	boost::mutex::scoped_lock updateLock(currentFrameSubscriptions->GetMutex());
 	currentFrame->copyTo(*targetFrame);
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
		if( capture )
		{
			IplImage* iplImg = cvQueryFrame( capture );

			// cvQuery frame is blocking, while waaiting for the frame.  
			//Check again if we were stopped in the meantime 
			if (WasStopped()) 
			{
			threadLock.unlock();
			break;
			}

			if (iplImg) 
			{
				(*bufferFrame) = iplImg;

				boost::mutex::scoped_lock currentLock(currentFrameSubscriptions->GetMutex());
				bufferFrame->copyTo(*currentFrame);
				currentFrameSubscriptions->PutNews();
				currentLock.unlock();
			}

            if( currentFrame->empty() )
			{
			threadLock.unlock();
                break;
			}
		}

		// Acquire a still video frame
		else if  (!image.empty() )
		{
			boost::mutex::scoped_lock currentLock(currentFrameSubscriptions->GetMutex());
			currentFrameSubscriptions->PutNews();
			currentLock.unlock();
		}
		else 
		{
		}

		threadLock.unlock();
	} // for ;;



	if (capture) 
	{
		cvReleaseCapture( &capture );
		capture = NULL;
	}
	mThreadExited = true;
}

void VideoCapture::DoThreadIteration()

{
	double delay = (frameRate * 1000) / 2;
	if (delay < 1.0) delay = 1;

	if (!WasStopped())
    {
		// Acquire from camera source or AVI
		if( capture )
		{
            IplImage* iplImg = cvQueryFrame( capture );
			{
            (*bufferFrame) = iplImg;

			boost::mutex::scoped_lock currentLock(currentFrameSubscriptions->GetMutex());
			bufferFrame->copyTo(*currentFrame);
			currentFrameSubscriptions->PutNews();
			currentLock.unlock();
			}
		}
		// Acquire a still video frame
		else if  (!image.empty() )
		{
			boost::mutex::scoped_lock currentLock(currentFrameSubscriptions->GetMutex());
			currentFrameSubscriptions->PutNews();
			currentLock.unlock();
		}
	} // for ;;
}

