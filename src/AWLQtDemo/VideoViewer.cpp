
#include <iostream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#endif

#include "VideoCapture.h"
#include "ReceiverCapture.h"
#include "Sensor.h"
#include "VideoViewer.h"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace awl;

#define THREADED_VIEWER

VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture, ReceiverCapture::Ptr inReceiverCapture, ReceiverProjector::Ptr inProjector):
workFrame(new (cv::Mat)),
displayFrame(new (cv::Mat)),
cameraName(inCameraName),
videoCapture(inVideoCapture),
receiverCapture(inReceiverCapture),
projector(inProjector),
bWindowCreated(false),
mThread()

{

	SetVideoCapture(videoCapture);
	SetReceiverCapture(receiverCapture);
	mStopRequested = false;
	mThreadExited = false;


}

VideoViewer::~VideoViewer()
{
	Stop();
}

void VideoViewer::SetVideoCapture( VideoCapture::Ptr inVideoCapture)
{
	boost::mutex::scoped_lock updateLock(mMutex);
 
	videoCapture = inVideoCapture;

	frameWidth = videoCapture->GetFrameWidth();
	frameHeight = videoCapture->GetFrameHeight();
	frameRate = videoCapture->GetFrameRate();
	scale = videoCapture->GetScale();
	cameraFovX = videoCapture->GetCameraFovX();
	cameraFovY = videoCapture->GetCameraFovY();

	currentVideoSubscriberID = videoCapture->currentFrameSubscriptions->Subscribe();

	updateLock.unlock();
}

void VideoViewer::SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture)
{
	boost::mutex::scoped_lock updateLock(mMutex);
 
	receiverCapture = inReceiverCapture;

	currentReceiverSubscriberID = receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();

	updateLock.unlock();
}

void VideoViewer::SetReceiverProjector(ReceiverProjector::Ptr inProjector)
{
	boost::mutex::scoped_lock updateLock(mMutex);
	projector = inProjector;
	updateLock.unlock();
}

void  VideoViewer::Go() 
{
	if (!(mThread && mThread->joinable()))
	{

		// Create output window
		if (!bWindowCreated) 
		{
			cvNamedWindow(cameraName.c_str(), 1 );
			bWindowCreated = true;
#if 0
		// Place the window in the top right corner
		top = 0;
		left = 
		cvMoveWindow(cameraName.c_str(),top, left)
#endif
		}

		mStopRequested = false;
		mThreadExited = false;

#ifdef THREADED_VIEWER
		mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&VideoViewer::DoThreadLoop, this)));
#endif
	}
}
 

void  VideoViewer::Stop() 
{
	if (mStopRequested) return;
	mStopRequested = true;

	if (mStopRequested || mThreadExited) 
	{
		mThreadExited=false;
#ifdef THREADED_VIEWER
		if (mThread.get() &&
			mThread->joinable())
		{
			mThread->join();
			mThread.reset();
		}

#endif
	}

	if (bWindowCreated) 
	{
		bWindowCreated = false;
		cvDestroyWindow(cameraName.c_str());

	}
}

bool  VideoViewer::WasStopped()
{
	if (mStopRequested || mThreadExited) return(true);

	return(false);
}

void VideoViewer::CopyWorkFrame(VideoCapture::FramePtr targetFrame) 
{
//	boost::mutex::scoped_lock updateLock(mMutex);
 	workFrame->copyTo(*targetFrame);
//	updateLock.unlock();
}

void VideoViewer::DoThreadLoop()

{
	while (!WasStopped())
	{
		// Update the video frame
		if (videoCapture != NULL && videoCapture->currentFrameSubscriptions->HasNews(currentVideoSubscriberID)) 
		{
			// Copy the contents of the cv::Mat
			videoCapture->CopyCurrentFrame(workFrame, currentVideoSubscriberID);

			// Add the lidar range decorations to the video frame
			DisplayReceiverValues(workFrame);

			// Copy to the display (we are duouble-buffering)
			workFrame->copyTo(*displayFrame);

			HWND window = ::FindWindowA(NULL, cameraName.c_str());
			if (window == NULL) bWindowCreated = false;

			if (bWindowCreated) cv::imshow(cameraName, *displayFrame);
		}

		//Give a break to other threads and wait for next frame
		if((!bWindowCreated) || (cv::waitKey(1) >= 0) )
		{
			break;
		}
	} // while (!WasStoppped)

	mThreadExited = true;
}


void VideoViewer::DoThreadIteration()

{
	if (!WasStopped())
	{
		// Update the video frame
		if (videoCapture != NULL && videoCapture->currentFrameSubscriptions->HasNews(currentVideoSubscriberID)) 
		{
			// Copy the contents of the cv::Mat
			videoCapture->CopyCurrentFrame(workFrame, currentVideoSubscriberID);

			// Add the lidar range decorations to the video frame
			DisplayReceiverValues(workFrame);

			// Copy to the display (we are duouble-buffering)
			workFrame->copyTo(*displayFrame);
			HWND window = ::FindWindowA(NULL, cameraName.c_str());
			if (window == NULL) bWindowCreated = false;

			if (bWindowCreated) cv::imshow(cameraName, *displayFrame);
		}

		//Give a break to other threads and wait for next frame
		if((!bWindowCreated) || (cv::waitKey(1) >= 0) )
		{
			Stop();
		}
	} // if (!WasStoppped)
}

void VideoViewer::DisplayReceiverValues(VideoCapture::FramePtr &targetFrame)

{
	for (int channelID = 0; channelID < 7; channelID++) 
	{
		if (channelID < receiverCapture->GetChannelQty())
		{
			if (receiverCapture->GetFrameQty()) 
			{

				ChannelFrame::Ptr channelFrame(new ChannelFrame(channelID));

				// Thread safe
				uint32_t lastDisplayedFrame = receiverCapture->GetSnapshotFrameID();
				if (receiverCapture->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, currentReceiverSubscriberID)) 
				{
					float minDistance = receiverCapture->GetMinDistance();
					float maxDistance = receiverCapture->GetMaxDistance();

					int detectionQty = channelFrame->detections.size();
					if (detectionQty > 4) detectionQty = 4;

					for (int i = 0; i < detectionQty; i++)
					{	
						Detection::Ptr detection = channelFrame->detections.at(i);
						float distance = detection->distance;
						if ((distance >= minDistance) && (distance <= maxDistance))
						{
							DisplayTarget(targetFrame, channelID, detection);
							break;  // We display only the first target in the list
						}
					}
				}
			}
		}
	}
}

void VideoViewer::DisplayTarget(VideoCapture::FramePtr &targetFrame, int channelID,  Detection::Ptr &detection)
{
	int top;
	int left;
	int bottom;
	int right;

	CvRect rect;

	cv::Vec3b color;
	cv::Vec3b colorEnhance;

	Detection::ThreatLevel threatLevel = detection->threatLevel;
	switch (threatLevel) 
	{
	case Detection::eThreatNone: 
		{
			colorEnhance = cv::Vec3b(0, 128, 0);
		}
		break;
	case Detection::eThreatLow:
		{
			colorEnhance = cv::Vec3b(0, 128, 0);
		}
		break;

	case Detection::eThreatWarn:
		{
			colorEnhance = cv::Vec3b(0, 128, 128);
		}
		break;

	case Detection::eThreatCritical:
		{
			colorEnhance = cv::Vec3b(128, 0, 0);
		}
		break;

	default:
		{
			colorEnhance = cv::Vec3b(0, 0, 0);
		}

	} // case


	// Paint a square that corresponds to the receiver FOV
	projector->GetChannelRect(channelID, top, left, bottom, right);

	for (int row = top; row <= bottom; row++) 
	{
		for (int column = left; column <= right; column++) 
		{
			color = targetFrame->at<cv::Vec3b>(row, column);
			int r = color[0]+ colorEnhance[0];
			int g = color[1] + colorEnhance[1];
			int b = color[2] + colorEnhance[2];

			if (r > 255) color[0] = 255;
			else		 color[0] = r;
			
			if (g > 255) color[1] = 255;
			else		 color[1] = g;
			
			if (b > 255) color[2] = 255;
			else		 color[2] = b;

			targetFrame->at<cv::Vec3b>(row, column) = color;
		}
	}
}





