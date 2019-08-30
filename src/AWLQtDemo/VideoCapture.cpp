/* VideoCapture.cpp */
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
// define XIFORWINDOWS
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
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include "xiapi.h"

using namespace std;
using namespace awl;

// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

const int ximeaDefaultBinningMode  = 4; // Binning mode on the ximea camera for 648x486 resolution
const int reopenCameraDelaylMillisec = 5000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

boost::posix_time::ptime reconnectTime;

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
    virtual int getCaptureDomain() { return cv::CAP_XIAPI; } // Return the type of the capture object: cv::CAP_VFW, etc...

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


VideoCapture::VideoCapture(int inCameraID, int argc, char** argv, boost::property_tree::ptree &propTree):
ThreadedWorker(),
Publisher(), 
calibration(),
cameraID(inCameraID)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	mStopRequested = false;

	// Initialize HighGUI
	cvInitSystem(argc, argv);

	ReadConfigFromPropTree(propTree);

	// Create the current frame blank
	currentFrame.create(calibration.frameHeightInPixels, calibration.frameWidthInPixels, CV_8UC3);
	currentFrame.setTo(cv::Scalar(0, 0, 0));


#if 0
	ListCameras();
#endif
	OpenCamera();
}

VideoCapture:: ~VideoCapture()

{
	if (!WasStopped()) Stop();
}

void  VideoCapture::Go() 
{
	assert(!mThread);
    mWorkerRunning = true;

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&VideoCapture::DoThreadLoop, this)));
}
 
void VideoCapture::CopyCurrentFrame(VideoCapture::FramePtr targetFrame, Publisher::SubscriberID inSubscriberID) 
{
	if (LockNews(inSubscriberID)) 
	{
		//Instead of simply cloning, make sure the target comes out as a 3 channel BGR image.
		targetFrame->create(currentFrame.rows, currentFrame.cols, CV_8UC3);
		*targetFrame = currentFrame.clone();

		UnlockNews(inSubscriberID);
	}
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
		boost::this_thread::sleep(boost::posix_time::milliseconds(3));	
	}

  	boost::mutex::scoped_lock threadLock(GetMutex());
	if (cam.isOpened()) 
	{
		cam.release();
	}
	threadLock.unlock();
}


void VideoCapture::DoThreadIteration()

{

	// If camera is not opened, check to see if we must try to reopen
	if (!cam.isOpened() && boost::posix_time::microsec_clock::local_time() > reconnectTime)
	{
		OpenCamera();
		reconnectTime = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(reopenCameraDelaylMillisec);
		return;
	}

	// Acquire from camera source or AVI
	if( cam.isOpened() )
	{
		// Set to all zeros.  This will be used to see if we have captured something, since normal openCV validations
		// don't seem to work.
		bufferFrame.setTo(0);

		// cam.read returns true, even when camera is unplugged. But, in this last case, the bufferFrame is 
		// all zeros so we need to test for both
		if ((!cam.read(bufferFrame)) || (norm(bufferFrame)==0))
		{
			cam.release();
			currentFrame.create(calibration.frameHeightInPixels, calibration.frameWidthInPixels, CV_8UC3);
			currentFrame.setTo(cv::Scalar(0, 0, 0));;
			return;
		}

		// cvQueryFrame (cam.read) is blocking, while waiting for the frame.  
		//Check again if we were stopped in the meantime 
		if (WasStopped()) 
		{
			return;
		}

		if (!bufferFrame.empty()) 
		{
#if 1  // Patch: Do not remove JYD 2014-07-07
			// Force-set the bufferFrame dimensions.  This corrects an OpenCV reporting bug with the XIMEA Camera, after downsampling
			bufferFrame.cols = calibration.frameWidthInPixels;
			bufferFrame.rows = calibration.frameHeightInPixels;
			bufferFrame.step = bufferFrame.cols*(bufferFrame.channels());
			// End of the Ximea patch
#endif


			boost::mutex::scoped_lock currentLock(GetMutex());
			if (bCameraFlip) 
			{
				cv::flip(bufferFrame, currentFrame, -1);
			}
			else 
			{
				bufferFrame.copyTo(currentFrame);
			}
			currentLock.unlock();
			PutNews();

		} // if (!bufferFrame.empty()) 

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);
	} // if( cam.isOpened() )
	else  
	{
		// No cam opened.  Show blank image
		
		boost::mutex::scoped_lock currentLock(GetMutex());

			if (currentFrame.empty()) 
			{
				currentFrame.create(calibration.frameHeightInPixels, calibration.frameWidthInPixels, CV_8UC3);
				//currentFrame.ones(calibration.frameHeightInPixels, calibration.frameWidthInPixels,CV_8UC3);
				currentFrame.setTo(cv::Scalar(0, 0, 0));

			} 
			currentLock.unlock();
			PutNews();
	}
}

void VideoCapture::ListCameras()

{
	HANDLE hMV;

	AWLSettings::GetGlobalSettings()->bWriteDebugFile = true;

	DebugFilePrintf("Requested Camera: %s", sCameraName.c_str());
	DWORD dwNumDevices;												
	//num of connected SHT devices	
#ifdef XIFORWINDOWS	
	if(xiGetNumberDevices(&dwNumDevices) == XI_OK)
	{
		int dwSerial = 0;

		if(xiOpenDevice( 0, &hMV) != XI_OK)
		{ 
			DebugFilePrintf("Cannot Open Ximea Cam");
		}

		xiGetParamInt( hMV, XI_PRM_DEVICE_SN, &dwSerial);
	
		char camName[512];
	
		xiGetParamString( hMV, XI_PRM_DEVICE_NAME, camName, 256);
				
		DebugFilePrintf("%s %08X", camName, dwSerial);
	}
#endif


	for (int i = 0; i < 2000; i++) 
	{
		DebugFilePrintf("Open %d", i);
		cam.open(i);
		if (cam.isOpened())
		{
			DebugFilePrintf("Found %d", i);
			cam.release();
		}
	}
}

bool VideoCapture::OpenCamera()

{
	int inputID = 0;


	if (!sCameraName.empty()) inputID = atoi(sCameraName.c_str());

		// Determine capture source:  Camera, Single Frame or AVI
    if( sCameraName.empty() || isdigit(sCameraName.c_str()[0]) )
	{
		cam.open(inputID);
	}
	else 
	{
		cam.open(sCameraName);
	}

		
	if (cam.isOpened()) 
	{
		// interpret preferred interface (0 = autodetect). This tells us what type of marea capabilities to expect
		int pref = (inputID / 100) * 100;

		// If we are using the Ximea driver, set the downsampling for a 640x480 image
		if (pref == cv::CAP_XIAPI)
		{
			// Set the amount of downsampling to get decent frame rate.
			cam.set(cv::CAP_PROP_XI_DOWNSAMPLING, ximeaDefaultBinningMode);
		}

		calibration.frameWidthInPixels = (int) cam.get(cv::CAP_PROP_FRAME_WIDTH);
		calibration.frameHeightInPixels = (int) cam.get(cv::CAP_PROP_FRAME_HEIGHT);
		double framesPerSecond = cam.get(cv::CAP_PROP_FPS);

 		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // cv::CAP_PROP_FPS may reurn 0;
		frameRate = (double) 1.0/framesPerSecond;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);

		return(true);
	}
	else // No camera
	{
		// Set default values not to be zeroes
		calibration.frameWidthInPixels = 640;
		calibration.frameHeightInPixels = 480;

		frameRate = (double) 1.0/30.0;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);

		return(false);
	}
}

bool VideoCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	char cameraKeyString[32];
	sprintf(cameraKeyString, "config.cameras.camera%d", cameraID);
	std::string cameraKey = cameraKeyString;

	boost::property_tree::ptree &cameraNode =  propTree.get_child(cameraKey);

	sCameraName = cameraNode.get<std::string>("cameraName");
	bCameraFlip = cameraNode.get<bool>("cameraFlip");


	/// Calibration parameters are optional
	float cameraFovWidthDegrees;
	float cameraFovHeightDegrees;
	AWLSettings::Get2DPoint(cameraNode.get_child("fov"), cameraFovWidthDegrees, cameraFovHeightDegrees);
	
	calibration.fovWidth = DEG2RAD(cameraFovWidthDegrees);
	calibration.fovHeight = DEG2RAD(cameraFovHeightDegrees);
	calibration.CalculateFocalLengthsFromFOVs();

	calibration.centerX = cameraNode.get<float>("centerCorrectionX", 0.0);
	calibration.centerY = cameraNode.get<float>("centerCorrectionY", 0.0);
	calibration.radialK1 = cameraNode.get<float>("radialCorrectionK1", 0.0);
	calibration.radialK1 = cameraNode.get<float>("radialCorrectionK1", 0.0);
	calibration.radialK2 = cameraNode.get<float>("radialCorrectionK2", 0.0);
	calibration.radialK3 = cameraNode.get<float>("radialCorrectionK3", 0.0);
	calibration.tangentialP1 = cameraNode.get<float>("tangentialCorrectionP1", 0.0);
	calibration.tangentialP1 = cameraNode.get<float>("tangentialCorrectionP2", 0.0);

	return(true);
}


