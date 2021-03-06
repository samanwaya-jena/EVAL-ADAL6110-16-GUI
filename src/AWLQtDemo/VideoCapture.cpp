/* VideoCapture.cpp: Capture from an OpenCV Camera */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the CuteApplication of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/

#include <fstream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "SensorCoord.h"
#include "DebugPrintf.h"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"


using namespace std;
using namespace awl;
SENSORCORE_USE_NAMESPACE

// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

const int reopenCameraDelaylMillisec = 5000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

boost::posix_time::ptime reconnectTime;



VideoCapture::VideoCapture(int inCameraID, int argc, char** argv, boost::property_tree::ptree &propTree):
ThreadedWorker(),
Publisher(), 
calibration(),
cameraID(inCameraID)

{
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

#ifdef _WINDOWS_
	// Set the priority under windows.  
	// Camera thread has to be lower priority than sensor communications threads
	HANDLE th = mThread->native_handle();

	SetThreadPriority(th, THREAD_PRIORITY_LOWEST);
#endif

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
	AWLSettings::GetGlobalSettings()->bWriteDebugFile = true;

	DebugFilePrintf("Requested Camera: %s", sCameraName.c_str());
	
	//max num of conected devices is 2000	

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
		calibration.frameWidthInPixels = (int) cam.get(cv::CAP_PROP_FRAME_WIDTH);
		calibration.frameHeightInPixels = (int) cam.get(cv::CAP_PROP_FRAME_HEIGHT);
		float framesPerSecond = (float) cam.get(cv::CAP_PROP_FPS);

 		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // cv::CAP_PROP_FPS may reurn 0;
		frameRate = (float) 1.0/framesPerSecond;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);

		return(true);
	}
	else // No camera
	{
		// Set default values not to be zeroes
		calibration.frameWidthInPixels = 640;
		calibration.frameHeightInPixels = 480;

		frameRate =  1.0f/30.0f;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);

		return(false);
	}
}

bool VideoCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	std::string cameraKey = std::string("config.cameras.camera") + std::to_string(cameraID);

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


