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
		if (pref == CV_CAP_XIAPI)
		{
			HANDLE ximeaHandle = ((VideoCaptureDummy *)&cam)->cap->hmv; 
			XI_RETURN xiRet = 0;

			// Capture format for Ximea is RGB32.  Preferable over RGB24 for performance reasons, according to Ximea documentation.
			cam.set(CV_CAP_PROP_XI_DATA_FORMAT, XI_RGB32 );
			// Downsampling: Prefer XI_SKIPPING over XI_BINNING for performance reasons.
//			xiRet = xiSetParamInt(ximeaHandle, XI_PRM_DOWNSAMPLING_TYPE, XI_SKIPPING);
//			xiSetParamInt(ximeaHandle, XI_PRM_SHUTTER_TYPE, XI_SHUTTER_GLOBAL);

			// Set color stuff
#if 0
			xiRet = xiSetParamInt(ximeaHandle, XI_PRM_AEAG, 1);  // Automatic exposure/gain enable
			xiRet = xiSetParamInt(ximeaHandle, XI_PRM_AE_MAX_LIMIT, 80000); // Maximum exposure time, in microsec

			xiRet = xiSetParamInt(ximeaHandle, XI_PRM_AG_MAX_LIMIT, 10.0); // Maximum limit of gain in AEAG procedure(dB).
			xiRet = xiSetParamInt(ximeaHandle, XI_PRM_AEAG_LEVEL, 10);  //Average intensity of output signal AEAG should achieve(in %).
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_EXP_PRIORITY, 0.8);  // Priority Gain VS exposure. 0.0: Gain <------> 1.0 Exposure
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_GAMMAY, 0.5); // Luminosity gamma. Range: 0.3 (highest correction); 1 (no correction)
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_GAMMAC, 80.0/100.0); // Chromaticity gamma. Default: 0.8
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_SHARPNESS, 0.0);  //Sharpness Strength. The range is -4 (less sharp) to +4 (more sharp). Default: 0.0 (neutral)
#endif
#if 0
			xiRet = xiSetParamInt(ximeaHandle, XI_PRM_AUTO_WB, 0);  // Auto white balance
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_WB_KR, 1.0);
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_WB_KG, 1.20);
			xiRet = xiSetParamFloat(ximeaHandle, XI_PRM_WB_KB, 1.0);
#else
					xiRet = xiSetParamInt(ximeaHandle, XI_PRM_AUTO_WB, 1);  // Auto white balance
#endif

#if 0
			float saturation  = (float)0.5;

			float colorCorrectionMatrix[4][4];
			colorCorrectionMatrix[0][0] = (float)(1.0+2*saturation);
			colorCorrectionMatrix[0][1] = -saturation;
			colorCorrectionMatrix[0][2] = -saturation;
			colorCorrectionMatrix[0][3] = 0.0;

			colorCorrectionMatrix[1][0] = -saturation;
			colorCorrectionMatrix[1][1] = (float)(1.0+2*saturation);
			colorCorrectionMatrix[1][2] = -saturation;
			colorCorrectionMatrix[1][3] = 0.0;

			colorCorrectionMatrix[2][0] = -saturation;
			colorCorrectionMatrix[2][1] = -saturation;
			colorCorrectionMatrix[2][2] = (float)(1.0+2*saturation);
			colorCorrectionMatrix[2][3] = 0.0;

			colorCorrectionMatrix[3][0] = 0.0;
			colorCorrectionMatrix[3][1] = 0.0;
			colorCorrectionMatrix[3][2] = 1.0;
			colorCorrectionMatrix[3][3] = 0.0;


			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_00, colorCorrectionMatrix[0][0]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_01, colorCorrectionMatrix[0][1]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_02, colorCorrectionMatrix[0][2]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_03, colorCorrectionMatrix[0][3]);

			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_10, colorCorrectionMatrix[1][0]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_11, colorCorrectionMatrix[1][1]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_12, colorCorrectionMatrix[1][2]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_03, colorCorrectionMatrix[1][3]);

			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_20, colorCorrectionMatrix[2][0]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_21, colorCorrectionMatrix[2][1]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_22, colorCorrectionMatrix[2][2]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_03, colorCorrectionMatrix[2][3]);

			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_30, colorCorrectionMatrix[3][0]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_31, colorCorrectionMatrix[3][1]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_32, colorCorrectionMatrix[3][2]);
			xiSetParamFloat(ximeaHandle, XI_PRM_CC_MATRIX_33, colorCorrectionMatrix[3][3]);
#endif
#if 0
			// Disable color management for performance reasons
			xiSetParamInt(ximeaHandle, XI_PRM_CMS, XI_CMS_DIS);
#endif

			// Set the amount of downsampling to get decent frame rate.
			cam.set(CV_CAP_PROP_XI_DOWNSAMPLING, ximeaDefaultBinningMode);
			// Always get the most recent frame.
			xiSetParamInt(ximeaHandle, XI_PRM_RECENT_FRAME, XI_ON);

		    // use minimum possible transport buffer size to optimize frame rate
			int transportBufferSize;
		    XI_RETURN stat = xiGetParamInt(ximeaHandle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_MIN, &transportBufferSize);
			stat = xiSetParamInt(ximeaHandle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, transportBufferSize);

			// set maximum number of queue
			int numberOfFieldBuffers = 0;
			stat = xiGetParamInt(ximeaHandle, XI_PRM_BUFFERS_QUEUE_SIZE XI_PRM_INFO_MAX, &numberOfFieldBuffers);
			stat = xiSetParamInt(ximeaHandle, XI_PRM_BUFFERS_QUEUE_SIZE, numberOfFieldBuffers);
		}

		calibration.frameWidthInPixels = (int) cam.get(CV_CAP_PROP_FRAME_WIDTH);
		calibration.frameHeightInPixels = (int) cam.get(CV_CAP_PROP_FRAME_HEIGHT);
		double framesPerSecond = cam.get(CV_CAP_PROP_FPS);

 		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // CV_CAP_PROP_FPS may reurn 0;
		frameRate = (double) 1.0/framesPerSecond;
	}
	else // No camera
	{
		// Set default values not to be zeroes
		calibration.frameWidthInPixels = 640;
		calibration.frameHeightInPixels = 480;

		frameRate = (double) 1.0/30.0;
	}
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
		cv::cvtColor(currentFrame, *targetFrame, CV_BGRA2BGR);
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
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));	
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


	} // if( cam.isOpened() )
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


