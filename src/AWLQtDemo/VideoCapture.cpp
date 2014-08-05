#include <fstream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "awlcoord.h"
#include "DebugPrintf.h"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include "xiapi.h"

using namespace std;
using namespace awl;

// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

const int ximeaDefaultBinningMode  = 4; // Binning mode on the ximea camera for 648x486 resolution

class CvCaptureCAM_XIMEA
{
public:
    CvCaptureCAM_XIMEA() { init(); }
    virtual ~CvCaptureCAM_XIMEA() { close(); }

    virtual bool open( int index );
    virtual void close();
    virtual double getProperty(int);
    virtual bool setProperty(int, double);
    virtual bool grabFrame();
    virtual IplImage* retrieveFrame(int);
    virtual int getCaptureDomain() { return CV_CAP_XIAPI; } // Return the type of the capture object: CV_CAP_VFW, etc...

public:
    void init();
    void errMsg(const char* msg, int errNum);
    void resetCvImage();
    int  getBpp();
    IplImage* frame;

    HANDLE    hmv;
    DWORD     numDevices;
    int       timeout;
    XI_IMG    image;
};

// VideoCaptureDummy is just a descriptive class to allow us to recuperate the private cap member from the 
// openCV CVCapture device.

class VideoCaptureDummy
{
public:
    CV_WRAP VideoCaptureDummy();
    CV_WRAP VideoCaptureDummy(const std::string& filename);
    CV_WRAP VideoCaptureDummy(int device);

    virtual ~VideoCaptureDummy();
    CV_WRAP virtual bool open(const std::string& filename);
    CV_WRAP virtual bool open(int device);
    CV_WRAP virtual bool isOpened() const;
    CV_WRAP virtual void release();

    CV_WRAP virtual bool grab();
    CV_WRAP virtual bool retrieve(CV_OUT cv::Mat& image, int channel=0);
    virtual VideoCapture& operator >> (CV_OUT cv::Mat& image);
    CV_WRAP virtual bool read(CV_OUT cv::Mat& image);

    CV_WRAP virtual bool set(int propId, double value);
    CV_WRAP virtual double get(int propId);

public:
    cv::Ptr<CvCaptureCAM_XIMEA> cap;
};

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

			// Capture format for Ximea is RGB32.  Preferable over RGB24 for performance reasons, according to Ximea documentation.
			cam.set(CV_CAP_PROP_XI_DATA_FORMAT, XI_RGB32 );
			// Downsampling: Prefer XI_SKIPPING over XI_BINNING for performance reasons.
			xiSetParamInt(((VideoCaptureDummy *)&cam)->cap, XI_PRM_DOWNSAMPLING_TYPE, XI_SKIPPING);
//			xiSetParamInt(((VideoCaptureDummy *)&cam)->cap, XI_PRM_SHUTTER_TYPE, XI_SHUTTER_GLOBAL);

			// Set the amount of downsampling to get decent frame rate.
			cam.set(CV_CAP_PROP_XI_DOWNSAMPLING, ximeaDefaultBinningMode);
			// Always get the most recent frame.
			xiSetParamInt(((VideoCaptureDummy *)&cam)->cap, XI_PRM_RECENT_FRAME, XI_ON);
		}

		frameWidth = (int) cam.get(CV_CAP_PROP_FRAME_WIDTH);
		frameHeight = (int) cam.get(CV_CAP_PROP_FRAME_HEIGHT);
		double framesPerSecond = cam.get(CV_CAP_PROP_FPS);

 		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // CV_CAP_PROP_FPS may reurn 0;
		frameRate = (double) 1.0/framesPerSecond;
	}
	else // No camera
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
	bCameraFlip = globalSettings->cameraFlip; 
}

VideoCapture:: ~VideoCapture()

{

	boost::mutex::scoped_lock threadLock(GetMutex());
	Stop();
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
	while (!WasStopped())
    {

		// Acquire one frame from camera source or AVI

		// Do not lock the thread. We should not be blocking
		//		boost::mutex::scoped_lock threadLock(GetMutex());

		DoThreadIteration();
		
		//		threadLock.unlock();

		// Messages must be at leat 1ms apart.
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));	
	}

  	boost::mutex::scoped_lock threadLock(GetMutex());
	if (cam.isOpened()) 
	{
		cam.release();
	}
	threadLock.unlock();

	mThreadExited = true;
}



void VideoCapture::DoThreadIteration()

{
	// Acquire from camera source or AVI
	if( cam.isOpened() )
	{
		cam.read(bufferFrame);

		// cvQuery frame is blocking, while waiting for the frame.  
		//Check again if we were stopped in the meantime 
		if (WasStopped()) 
		{
			return;
		}

		if (!bufferFrame.empty()) 
		{
#if 1
			// Force-set the bufferFrame dimensions.  This corrects an OpenCV reporting bug with the XIMEA Camera, after downsampling
			bufferFrame.cols = frameWidth;
			bufferFrame.rows = frameHeight;
			bufferFrame.step = bufferFrame.cols*(bufferFrame.channels());
			// End of the Ximea patch
#endif


			boost::mutex::scoped_lock currentLock(currentFrameSubscriptions->GetMutex());
			if (bCameraFlip) 
			{
				cv::flip(bufferFrame, currentFrame, -1);
			}
			else 
			{
				bufferFrame.copyTo(currentFrame);
			}
			currentFrameSubscriptions->PutNews();
			currentLock.unlock();
		} // if (!bufferFrame.empty()) 


	} // if( cam.isOpened() )
}


