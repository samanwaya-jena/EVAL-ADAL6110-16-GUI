/* APVideoCapture.cpp */
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

#if QT_VERSION > 0x050000
#include <QMessageBox>
#else
#include <QtGui/QMessageBox>
#endif


#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "AWLSettings.h"
#include "APVideoCapture.h"
#include "awlcoord.h"
#include "DebugPrintf.h"

using namespace std;
using namespace awl;

// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

const int ximeaDefaultBinningMode  = 4; // Binning mode on the ximea camera for 648x486 resolution
const int reopenCameraDelaylMillisec = 5000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

boost::posix_time::ptime apReconnectTime;

APVideoCapture::APVideoCapture(int inCameraID, int argc, char** argv, boost::property_tree::ptree &propTree):
ThreadedWorker(),
Publisher(), 
calibration(),
cameraID(inCameraID)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	mStopRequested = false;

	// Initialize HighGUI
	//cvInitSystem(argc, argv);

	ReadConfigFromPropTree(propTree);

	// Create the current frame blank
	//currentFrame.create(calibration.frameHeightInPixels, calibration.frameWidthInPixels, CV_8UC3);
	//currentFrame.setTo(cv::Scalar(0, 0, 0));
	//
	currentFrame.data = NULL;
	bufferFrame.data = NULL;
	apbase = NULL;
	rgbBuffer = NULL;


#if 0
	ListCameras();
#endif
	OpenCamera();
}

APVideoCapture:: ~APVideoCapture()

{
	if (!WasStopped()) Stop();
}

void  APVideoCapture::Go() 
{
	assert(!mThread);
    mWorkerRunning = true;

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&APVideoCapture::DoThreadLoop, this)));
}
 
void APVideoCapture::CopyCurrentFrame(APVideoCapture::FramePtr targetFrame, Publisher::SubscriberID inSubscriberID) 
{
	if (LockNews(inSubscriberID)) 
	{
		//Instead of simply cloning, make sure the target comes out as a 3 channel BGR image.
		//targetFrame->create(currentFrame.rows, currentFrame.cols, CV_8UC3);
		//*targetFrame = currentFrame.clone();
		
		ap_u32 width, height, depth;
		depth = 24;
		rgbBuffer = ap_ColorPipe(apbase, currentFrame.data, currentFrame.size, &width, &height, &depth);
		//printf ("ZZ %d %d %d\n", width, height, depth);

		targetFrame->size = width * height * depth / 8;
		//targetFrame->size = currentFrame.size;
		targetFrame->cols = width;
		targetFrame->rows = height;
		if (targetFrame->data) delete targetFrame->data;
		targetFrame->data = new unsigned char [targetFrame->size];
		//memcpy(targetFrame->data, currentFrame.data, currentFrame.size);
		memcpy(targetFrame->data, rgbBuffer, targetFrame->size);
		//memset(targetFrame->data, 255, currentFrame.size);

		UnlockNews(inSubscriberID);
	}
};


void APVideoCapture::DoThreadLoop()

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

	ap_Destroy(apbase);
	ap_Finalize();

	/*
	if (ap_CheckSensorState(apbase, 0)) 
	{
		cam.release();
	}
	*/
	threadLock.unlock();
}


void APVideoCapture::DoThreadIteration()

{

	if (apbase && ap_GrabFrame(apbase, currentFrame.data, currentFrame.size)) PutNews();
	return;

	ap_u32 width;
	ap_u32 height;
	char imageType[64];

	ap_s32 ret;
	
	ret = ap_GetImageFormat(apbase, &width, &height, imageType, 64);
	ap_CheckSensorState(apbase, 0);
	ret = ap_SetImageFormat(apbase, width/2, height/2, imageType);
	ap_CheckSensorState(apbase, 0);
	ret = ap_GetImageFormat(apbase, &width, &height, imageType, 64);
	ap_CheckSensorState(apbase, 0);

	if (0)
	{
		char s[1024];
		QMessageBox msgBox;
		snprintf(s, 1024, "%dx%d %s", width, height, imageType);
		msgBox.setText(s);
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
	}
	// If camera is not opened, check to see if we must try to reopen
	if (0 && !ap_CheckSensorState(apbase, 0) && boost::posix_time::microsec_clock::local_time() > apReconnectTime)
	{
		OpenCamera();
		apReconnectTime = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(reopenCameraDelaylMillisec);
		return;
	}

	// Acquire from camera source or AVI
	if( ap_CheckSensorState(apbase, 0) )
	{
		// Set to all zeros.  This will be used to see if we have captured something, since normal openCV validations
		// don't seem to work.
		//bufferFrame.setTo(0);

		// cam.read returns true, even when camera is unplugged. But, in this last case, the bufferFrame is 
		// all zeros so we need to test for both
		//
		/*
		if ((!ap_GrabFrame(apbase, bufferFrame, (ap_u32)nBufferSize)) || (norm(bufferFrame)==0))
		{
			cam.release();
			currentFrame.create(calibration.frameHeightInPixels, calibration.frameWidthInPixels, CV_8UC3);
			currentFrame.setTo(cv::Scalar(0, 0, 0));;
			return;
		}
		*/
		// cvQueryFrame (cam.read) is blocking, while waiting for the frame.  
		//Check again if we were stopped in the meantime 
		if (WasStopped()) 
		{
			return;
		}
/*
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
*/
	apReconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);
	} // if( ap_CheckSensorState(apbase, 0) )
	else  
	{
		// No cam opened.  Show blank image
		
		boost::mutex::scoped_lock currentLock(GetMutex());
		/*

			if (currentFrame.empty()) 
			{
				currentFrame.create(calibration.frameHeightInPixels, calibration.frameWidthInPixels, CV_8UC3);
				//currentFrame.ones(calibration.frameHeightInPixels, calibration.frameWidthInPixels,CV_8UC3);
				currentFrame.setTo(cv::Scalar(0, 0, 0));

			} 
		*/
			currentLock.unlock();
			PutNews();
	}
}

void APVideoCapture::ListCameras()

{
	/*
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
		if (ap_CheckSensorState(apbase, 0))
		{
			DebugFilePrintf("Found %d", i);
			cam.release();
		}
	}
	*/
}

bool APVideoCapture::OpenCamera()

{
	int inputID = 0;
	int retVal;
	apbase = NULL;
	printf( "\n\nOpenCamera\n\n");

	retVal = ap_DeviceProbe(NULL);
	//retVal = ap_DeviceProbe("../AP/sensor_data");

	if (retVal != AP_CAMERA_SUCCESS)
	{
		QMessageBox msgBox;
		msgBox.setText("Unable to either detect a sensor or find a matching SDAT file.");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return false;
	}

	apbase = ap_Create(0);
	if (apbase == NULL)
	{
		QMessageBox msgBox;
		msgBox.setText("Camera initialization error.");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return false;
	}

	//ap_LoadIniPreset(apbase, NULL, NULL);
	ap_SetState(apbase, "Sensor Reset", 1);
	ap_SetState(apbase, "Sensor Reset", 0);
	ap_LoadIniPreset(apbase, NULL, "Python:");
	//ap_RunPython(apbase, "loadXML(apbase.home + '/apps_data/AR0231AT-REV7.ini')");
	//ap_LoadIniPreset(apbase, "../AP/apps_data/AR0231AT-REV7.ini", "Linear Parallel Full Resolution");
	ap_LoadIniPreset(apbase, "../AP/apps_data/AR0231AT-REV7.ini", "Linear Full Resolution Mode Parallel 12-bit 30FPS PLL Enable");
	ap_CheckSensorState(apbase, 0);

	ap_u32 bufSize;

	bufSize = ap_GrabFrame(apbase, NULL, 0);

	ap_u32 width;
	ap_u32 height;
	char imageType[64];

	ap_s32 ret;
	
	ret = ap_GetImageFormat(apbase, &width, &height, imageType, 64);
	//ret = ap_SetImageFormat(apbase, width, height, "RGB-24");
	ret = ap_GetImageFormat(apbase, &width, &height, imageType, 64);

	bufSize = ap_GrabFrame(apbase, NULL, 0);

	if (0)
	{
		char s[1024];
		QMessageBox msgBox;
		snprintf(s, 1024, "%dx%d %s %d", width, height, imageType, bufSize);
		msgBox.setText(s);
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
	}

	if (rgbBuffer) delete rgbBuffer;
	rgbBuffer = new unsigned char[bufSize];

	if (bufSize < (width * height * 3)) bufSize = (width * height * 3);
	if (currentFrame.data) delete currentFrame.data;
	currentFrame.size = bufSize;
	currentFrame.cols = width;
	currentFrame.rows = height;
	currentFrame.data = new unsigned char[bufSize];

	if (bufferFrame.data) delete bufferFrame.data;
	bufferFrame.size = bufSize;
	bufferFrame.cols = width;
	bufferFrame.rows = height;
	bufferFrame.data = new unsigned char [bufSize];

/*

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
*/
		
	if (ap_CheckSensorState(apbase, 0)) 
	{
		/*
		// interpret preferred interface (0 = autodetect). This tells us what type of marea capabilities to expect
		int pref = (inputID / 100) * 100;

		// If we are using the Ximea driver, set the downsampling for a 640x480 image
		if (pref == CV_CAP_XIAPI)
		{
			// Set the amount of downsampling to get decent frame rate.
			cam.set(CV_CAP_PROP_XI_DOWNSAMPLING, ximeaDefaultBinningMode);
		}

		calibration.frameWidthInPixels = (int) cam.get(CV_CAP_PROP_FRAME_WIDTH);
		calibration.frameHeightInPixels = (int) cam.get(CV_CAP_PROP_FRAME_HEIGHT);
		double framesPerSecond = cam.get(CV_CAP_PROP_FPS);

 		if (framesPerSecond < 1) framesPerSecond = FRAME_RATE;  // CV_CAP_PROP_FPS may reurn 0;
		frameRate = (double) 1.0/framesPerSecond;
		*/
		calibration.frameWidthInPixels = width;
		calibration.frameHeightInPixels = height;

		frameRate = (double) 1.0/30.0;
		apReconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);

		return(true);
	}
	else // No camera
	{
		// Set default values not to be zeroes
		calibration.frameWidthInPixels = 640;
		calibration.frameHeightInPixels = 480;

		frameRate = (double) 1.0/30.0;
		apReconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenCameraDelaylMillisec);

		return(false);
	}
    return true;
}

bool APVideoCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	char cameraKeyString[32];
	sprintf(cameraKeyString, "config.cameras.camera%d", cameraID);
	std::string cameraKey = cameraKeyString;

	boost::property_tree::ptree &cameraNode =  propTree.get_child(cameraKey);

	sCameraName = cameraNode.get<std::string>("cameraName");
	//sCameraAPI  = cameraNode.get<std::string>("cameraAPI");
	bCameraFlip = cameraNode.get<bool>("cameraFlip");

printf ("%d %s", cameraID, sCameraName.c_str());
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


