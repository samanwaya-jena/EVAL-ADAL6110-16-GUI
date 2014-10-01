#include <fstream>

#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>


#include "Sensor.h"
#include "VideoCapture.h"
#include "ReceiverCapture.h"
#include "AWLSettings.h"
#include "awlcoord.h"

using namespace std;
using namespace pcl;
using namespace awl;

const int threadSleepDelay = 10;  // Thread sleep time between iterations.

const cv::Vec3b cvBlack(0, 0, 0);

ReceiverChannel::ReceiverChannel(	const int inReceiverID, const int inChannelID, const float inFovWidth, const float inFovHeight, 
						const float inCenterX, const float inCenterY, const float inRangeMax, 
						bool inDisplayUnderZero, 
						double inDisplayColorR, double inDisplayColorG, double inDisplayColorB):
	receiverID(inReceiverID),
	channelID(inChannelID),
	fovWidthX(inFovWidth),
	fovWidthY(inFovHeight),
	fovCenterX(inCenterX),
	fovCenterY(-inCenterY),  // In our world, pitch is clowckwise, whereas elsewere is counterclockwise.
	rangeMax(inRangeMax), 
	displayColorR(inDisplayColorR),
	displayColorG(inDisplayColorG),
	displayColorB(inDisplayColorB),
	displayUnderZero(inDisplayUnderZero),
	decimationX(1),
	decimationY(1),
	backgroundPtr(),
	colorPtr(),
	currentCloud()

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	sensorUp = globalSettings->receiverSettings[receiverID].sensorUp;
	sensorForward = globalSettings->receiverSettings[receiverID].sensorForward;
	rangeMax = globalSettings->receiverSettings[receiverID].displayedRangeMax;
} 

ReceiverChannel::~ReceiverChannel()
{

}

void ReceiverChannel::AddDistancesToCloud()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	float minDistance = globalSettings->receiverSettings[receiverID].displayedRangeMin;
	float maxDistance = globalSettings->receiverSettings[receiverID].channelsConfig[channelID].maxRange;

	if (channelID < receiverCapture->GetChannelQty())
	{
		if (receiverCapture->GetFrameQty()) 
		{
	
			ChannelFrame::Ptr channelFrame(new ChannelFrame(receiverCapture->receiverID, channelID));

			// Thread safe

			uint32_t lastDisplayedFrame = receiverCapture->GetCurrentIssueID(receiverCaptureSubscriberID);
			if (receiverCapture->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, receiverCaptureSubscriberID))
			{

				int detectionQty = channelFrame->detections.size();
				for (int i = 0; i < detectionQty; i++)
				{
					Detection::Ptr detection = channelFrame->detections.at(i);

					if (detection->distance >= minDistance && detection->distance <= maxDistance) 
					{
							AddDistanceToCloud(detection->distance, 255);
					}
				}
			}
		}
	}
}

void ReceiverChannel::AddDistanceToCloud(float inDistance, uint8_t inIntensity)

{
	PointXYZRGB newCloudPoint;
	PointWithRange pointWithRange;
	cv::Vec3b mask;

	assert(backgroundPtr!= NULL);
	assert(colorPtr != NULL);

	int size = maskPoints.size();
	CvPoint *point = (CvPoint *) maskPoints.data();

	mask.val[0] = 1; mask.val[1] = 1; mask.val[2] = 1;

	for (int i = 0; i < size; i++, point++) 
	{
		int x = point->x;
		int y = point->y;

		// Check to see if X and Y are non-decimated points
		bool bIsOnY = (y / decimationY)*(decimationY) == y;
		bool bIsOnX = (x/ decimationX) * decimationX == x;

		if (bIsOnY && bIsOnX)// Display only points that are an even multimple of decimation Y
		{
			if (y < colorPtr->rows && y < backgroundPtr->rows &&
				x < colorPtr->cols && x < backgroundPtr->cols) 
			{
				AddDistanceToCloud((const cv::Vec3b &) mask, 
										  (const cv::Vec3b &) colorPtr->at<cv::Vec3b>(y, x), 
										  (cv::Vec3b &) backgroundPtr->at<cv::Vec3b>(y, x), 
										   x, y, inDistance, inIntensity);
			}
		}
	}
}


void ReceiverChannel::AddDistanceToCloud(const cv::Vec3b &mask,
										const cv::Vec3b &color,
										cv::Vec3b &background,
										int xPos,
										int yPos, 
										float inDistance, 
										uint8_t inIntensity)
{
	PointXYZRGB newCloudPoint;

	if (mask.val[1]  | mask.val[1] | mask.val[2]) 
	{
		// Define point location
		float pointX = ((float) xPos);
		float pointY = imageHeight - (float)yPos;
		viewerCoordinatesPtr->GetXYZFromRange(pointX, pointY, inDistance, newCloudPoint);

		if ((newCloudPoint.y > (-sensorUp)) || displayUnderZero) {
			newCloudPoint.b = color[0];
			newCloudPoint.g = color[1];
			newCloudPoint.r = color[2];

			newCloudPoint.a = inIntensity;

			//	Add the point to the array;
			currentCloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);

			// mask the background
			background = cvBlack;
		}
	}
}

void ReceiverChannel::AddPointToCloud(float x, float y, float z,
									  uint8_t r,
									  uint8_t g,
									  uint8_t b,
								      uint8_t inIntensity)

{
	PointXYZRGB newCloudPoint;

	// Define point location
	newCloudPoint.x = x;
	newCloudPoint.y = y;
	newCloudPoint.z = z;

	newCloudPoint.b = b;
	newCloudPoint.g = g;
	newCloudPoint.r = r;

	newCloudPoint.a = inIntensity;

	//	Add the point to the array;
	currentCloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);
}

void ReceiverChannel::UpdateViewerCoordinates(ViewerCoordinates::Ptr &inViewerCoordinatesPtr)
{
	viewerCoordinatesPtr = inViewerCoordinatesPtr;
	imageWidth = viewerCoordinatesPtr->GetWidth();
	imageHeight = viewerCoordinatesPtr->GetHeight();

	float x, y;
	viewerCoordinatesPtr->getImagePointFromAngles(-(fovCenterX - (fovWidthX/2)), 
											 (fovCenterY) - (fovWidthY/2),
											 x, y);
	topLeftX = (int) x;
	bottomRightY = imageHeight - (int) y;  // Y is reversed between video and cloudView!

			
	viewerCoordinatesPtr->getImagePointFromAngles(-(fovCenterX + (fovWidthX/2)), 
											 (fovCenterY) + (fovWidthY/2),
											 x, y);
	bottomRightX = (int) x;
	topLeftY = imageHeight - (int) y; // Y is reversed between video and cloudView!

	if (topLeftX < 0) topLeftX = 0;
	if (topLeftY < 0) topLeftY = 0;
	if (bottomRightX >= imageWidth) bottomRightX = imageWidth - 1;
	if (bottomRightY >= imageHeight) bottomRightY = imageHeight - 1;

	BuildPixelMask();
}

bool IsPtInCircle( CvPoint2D32f pt, CvPoint2D32f center, float radius )
{
    double dx = pt.x - center.x;
    double dy = pt.y - center.y;
    if (((double)radius*radius - dx*dx - dy*dy) > 0) 
		return (true);
	else
		return(false);
}

void ReceiverChannel::BuildPixelMask()

{
	PointXYZRGB newCloudPoint;
	PointWithRange pointWithRange;
	cv::Vec3b mask;
	cv::Vec3b color;

	CvPoint2D32f center;
	center.x = (topLeftX + bottomRightX) /2;
	center.y = (topLeftY + bottomRightY) / 2;
	int radius = (bottomRightY - topLeftY) / 2;
	if (radius < 0) radius = 0;


	for (int y = 0; y < imageHeight; y++)
	{
		for (int x = 0; x < imageWidth; x++) 
		{
			CvPoint2D32f thePoint;
			thePoint.x = x;
			thePoint.y = y;

			if (IsPtInCircle(thePoint, center, radius))
			{
				CvPoint maskPoint;
				maskPoint.x = x;
				maskPoint.y = y;
				maskPoints.push_back(maskPoint);
			}
		}
	}
}

void ReceiverChannel::GetChannelLimits(ViewerCoordinates::Ptr &inViewerCoordinates, double &minX, double &minY, double &minZ, 
		double &maxX, double &maxY, double &maxZ, double &originX, double &originY, double &originZ)
{
	imageWidth = inViewerCoordinates->GetWidth();
	imageHeight = inViewerCoordinates->GetHeight();

	float x, y;

	inViewerCoordinates->getImagePointFromAngles(-(fovCenterX - (fovWidthX/2)), 
											 (fovCenterY) - (fovWidthY/2),
											 x, y);
	int topLeftX = (int) x;
	int bottomRightY = imageHeight - (int) y;  // Y is reversed between video and cloudView!

			
	inViewerCoordinates->getImagePointFromAngles(-(fovCenterX + (fovWidthX/2)), 
											 (fovCenterY) + (fovWidthY/2),
											 x, y);
	int bottomRightX = (int) x;
	int topLeftY = imageHeight - (int) y; // Y is reversed between video and cloudView!

	if (topLeftX < 0) topLeftX = 0;
	if (topLeftY < 0) topLeftY = 0;
	if (bottomRightX >= imageWidth) bottomRightX = imageWidth - 1;
	if (bottomRightY >= imageHeight) bottomRightY = imageHeight - 1;

	PointXYZRGB topLeftPoint;
	inViewerCoordinates->GetXYZFromRange((float)topLeftX, (float)topLeftY, rangeMax, topLeftPoint);

	PointXYZRGB bottomRightPoint;
	inViewerCoordinates->GetXYZFromRange((float)bottomRightX, (float)bottomRightY, rangeMax, bottomRightPoint);
	
	minX = topLeftPoint.x;
	minY = topLeftPoint.y;
	minZ = topLeftPoint.z;
	maxX = bottomRightPoint.x;
	maxY = bottomRightPoint.y;
	maxZ = bottomRightPoint.z;

	originX = 0;
	originY = sensorUp;
	originZ = sensorForward;
}

void ReceiverChannel::GetChannelRect(ViewerCoordinates::Ptr &inViewerCoordinates, int &top, int &left, int &bottom, int &right)
{
	imageWidth = inViewerCoordinates->GetWidth();
	imageHeight = inViewerCoordinates->GetHeight();
	float x, y;

	inViewerCoordinates->getImagePointFromAngles(-(fovCenterX - (fovWidthX/2)), 
											 (fovCenterY) - (fovWidthY/2),
											 x, y);
	left = (int) x;
	bottom = imageHeight - (int) y;  // Y is reversed between video and cloudView!

			
	inViewerCoordinates->getImagePointFromAngles(-(fovCenterX + (fovWidthX/2)), 
											 (fovCenterY) + (fovWidthY/2),
											 x, y);
	right = (int) x;
	top = imageHeight - (int) y; // Y is reversed between video and cloudView!

	// Make sure we are in order
	
	if (left > right) 
	{
		int temp = right;
		right = left;
		left = temp;
	}
	if (top > bottom)
	{
		int temp = bottom;
		bottom = top;
		top = temp;
	}

	// Make sure we are within display bounds
	if (left < 0) left = 0;
	if (top < 0) top = 0;
	if (right >= imageWidth) right = imageWidth - 1;
	if (bottom >= imageHeight) bottom = imageHeight - 1;

}

void ReceiverChannel::GetDisplayColor(double &outR, double &outG, double&outB)
{
	outR = displayColorR;
	outG = displayColorG;
	outB = displayColorB;
}

bool ReceiverChannel::SetDisplayUnderZero(bool inDisplayUnderZero)
{
	displayUnderZero = inDisplayUnderZero;
	return (displayUnderZero);
}


bool ReceiverChannel::GetDisplayUnderZero()
{
	return (displayUnderZero);
}


void ReceiverChannel::SetDecimation(int inDecimationX, int inDecimationY)
{
	decimationX = inDecimationX;
	decimationY = inDecimationY;
}


void ReceiverChannel::GetDecimation(int &outDecimationX, int &outDecimationY)
{
	outDecimationX = decimationX;
	outDecimationY = decimationY;
}

void ReceiverChannel::SetSensorUp(double inSensorUp)
{
	sensorUp = inSensorUp;
}

void ReceiverChannel::GetSensorUp(double &outSensorUp)
{
	outSensorUp = sensorUp;
}

void ReceiverChannel::SetSensorForward(double inSensorForward)
{
	sensorForward = inSensorForward;
}

void ReceiverChannel::GetSensorForward(double &outSensorForward)
{
	outSensorForward = sensorForward;
}

void ReceiverChannel::SetRangeMax(double inRangeMax)
{
	rangeMax = inRangeMax;
}

void ReceiverChannel::GetRangeMax(double &outRangeMax)
{
	outRangeMax = rangeMax;
}

void ReceiverChannel::SetBackgroundPtr(ReceiverChannel::FramePtr inBackgroundPtr) 

{
	backgroundPtr = inBackgroundPtr;
}

void ReceiverChannel::SetColorPtr(ReceiverChannel::FramePtr inColorPtr) 

{
	colorPtr = inColorPtr;
}

void ReceiverChannel::SetCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inCurrentCloud) 

{
	currentCloud = inCurrentCloud;
}

void ReceiverChannel::SetReceiver(ReceiverCapture::Ptr inReceiver, 
	Publisher::SubscriberID inCurrentReceiverCaptureSubscriberID)
{
	receiverCapture = inReceiver;
	receiverCaptureSubscriberID = inCurrentReceiverCaptureSubscriberID;
}


//*********************************************************************

ReceiverProjector::ReceiverProjector(VideoCapture::Ptr videoCapture, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inCloud, 
	ReceiverCapture::Ptr receiverCapture):
LoopedWorker(),
Publisher(),
displayUnderZero(true),
currentFrame(new (cv::Mat)),
backgroundFrame(new (cv::Mat))

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
		AWLCoordinates *globalCoord = AWLCoordinates::GetGlobalCoordinates();
		int receiverID = receiverCapture->GetReceiverID();
	CartesianCoord relativeCoord(0, 0, 0);
	CartesianCoord worldCoord(0,0,0);
	worldCoord = globalCoord->GetReceiver(receiverID)->ToReferenceCoord(eSensorToWorldCoord, relativeCoord);
	
	up = worldCoord.up;
	forward = worldCoord.forward;
	rangeMax = globalSettings->viewerMaxRange;
	decimationX = globalSettings->decimation;
	decimationY = globalSettings->decimation;


	SetVideoCapture(videoCapture);
	SetReceiverCapture(receiverCapture);
	SetCloud(inCloud);
}

ReceiverProjector::~ReceiverProjector()
{
	Stop();
}

bool  ReceiverProjector::WasStopped()
{
	if (LoopedWorker::WasStopped()) return(true);
	if (videoCapture->WasStopped()) {
		Stop();
		return(true);
	}
	if (receiverCapture->WasStopped()) {
		Stop();
		return(true);
	}

	return(false);
}


void ReceiverProjector::SetVideoCapture( VideoCapture::Ptr inVideoCapture)
{
	videoCapture = inVideoCapture;

	frameWidth = videoCapture->GetFrameWidth();
	frameHeight = videoCapture->GetFrameHeight();
	frameRate = videoCapture->GetFrameRate();
	scale = videoCapture->GetScale();
	cameraFovWidth = videoCapture->GetCameraFovWidth();
	cameraFovHeight = videoCapture->GetCameraFovHeight();

	mViewerCoordinatesPtr = ViewerCoordinates::Ptr(new ViewerCoordinates(frameWidth, frameHeight, 
                                                cameraFovWidth, cameraFovHeight, up, forward, rangeMax));

	currentVideoSubscriberID = videoCapture->Subscribe();
}

void ReceiverProjector::SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture)
{
	receiverCapture = inReceiverCapture;

	currentReceiverCaptureSubscriberID = receiverCapture->Subscribe();
}

void ReceiverProjector::SetCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inCloud)
{
 	cloud = inCloud;
}

void ReceiverProjector::CopyCurrentCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outCloud, Publisher::SubscriberID inSubscriberID)
{
	if (LockNews(inSubscriberID))
	{
	 	pcl::copyPointCloud(*cloud, *outCloud);
		UnlockNews(inSubscriberID);
	}
}


ReceiverChannel::Ptr & ReceiverProjector::GetChannel(int inChannelID) 
{
	return receiverChannels.at(inChannelID);
}

ReceiverChannel::Ptr &ReceiverProjector::AddChannel(ReceiverChannel::Ptr &inChannelPtr) 
{
	inChannelPtr->UpdateViewerCoordinates(mViewerCoordinatesPtr);
	receiverChannels.push_back(inChannelPtr);

	return inChannelPtr;
}

void ReceiverProjector::SpinOnce()

{
	if (!WasStopped())
    {
		// Update the video frame before we build the point-cloud
		// if there is a video frame available.

		if (videoCapture != NULL && (videoCapture->HasNews(currentVideoSubscriberID))) 
		{
			// Copy the contents of the working frame, if is is updated
			videoCapture->CopyCurrentFrame(currentFrame, currentVideoSubscriberID);
			
			// Update the point-cloud data.
			// Right now, we pace ourselves with the video frames
			AddDistancesToCloud();
			PutNews();
		}
	} // while (!WasStoppped)

}

void ReceiverProjector::AddDistancesToCloud()
{

	// Prepare a new background that will be drawn upon from the current frame 
	SetBackgroundFrame(GetCurrentFrame());

	// Reset the cloud.  Clear all current points
	ResetCloud();

	int channelQty = receiverChannels.size();
	// For each of the channels, update local information
	for (int channelID = 0; channelID < channelQty; channelID++) 
	{
		GetChannel(channelID)->SetBackgroundPtr(backgroundFrame);
		GetChannel(channelID)->SetColorPtr(currentFrame);
		GetChannel(channelID)->SetCurrentCloud(cloud);
		GetChannel(channelID)->SetDisplayUnderZero(displayUnderZero);
		GetChannel(channelID)->SetDecimation(decimationX, decimationY);
		GetChannel(channelID)->SetReceiver(receiverCapture, currentReceiverCaptureSubscriberID );
	}

	for (int i = 0; i < channelQty; i++) 
	{
		GetChannel(i)->AddDistancesToCloud();
	}

	AddBackgroundToCloud();
}


void ReceiverProjector::GetChannelLimits(int channel, double &minX, double &minY, double &minZ, 
	double &maxX, double &maxY, double &maxZ, double &originX, double &originY, double &originZ)
{
	GetChannel(channel)->GetChannelLimits(mViewerCoordinatesPtr, minX, minY, minZ, maxX, maxY, maxZ, originX, originY, originZ);
}

void ReceiverProjector::GetChannelRect(int channel, int &top, int &left, int &bottom, int &right)
{
	GetChannel(channel)->GetChannelRect(mViewerCoordinatesPtr, top, left, bottom, right);
}


void ReceiverProjector::GetDisplayColor(int channel, double &outR, double &outG, double&outB)
{
	GetChannel(channel)->GetDisplayColor(outR, outG, outB);
}

void ReceiverProjector::SetBackgroundFrame(ReceiverProjector::FramePtr &inFrame) 
{
	 inFrame->copyTo(*backgroundFrame);
}

void ReceiverProjector::SetCurrentFrame(ReceiverProjector::FramePtr &inFrame) 
{
	inFrame->copyTo(*currentFrame);
}

void ReceiverProjector::AddBackgroundToCloud()

{
	PointXYZRGB newCloudPoint;
	PointWithRange pointWithRange;
	cv::Vec3b cvColor;

	int width = mViewerCoordinatesPtr->GetWidth();
	int height = mViewerCoordinatesPtr->GetHeight();

	if (height > backgroundFrame->rows || width > backgroundFrame->cols)
		return;

	for (int i = 0; i < width; i+=decimationX) 
	{
		for (int j = 0; j < height; j+=decimationY)
		{
			// Define point location
			float pointX = ((float) i);
			float pointY = height - (float)j;

			mViewerCoordinatesPtr->GetXYZFromRange(pointX, pointY, 50.0, newCloudPoint);

			cvColor = backgroundFrame->at<cv::Vec3b>(j, i); // extract color from Mat

			newCloudPoint.b = cvColor[0];
			newCloudPoint.g = cvColor[1];
			newCloudPoint.r = cvColor[2];

			newCloudPoint.a = 255;

			//	Add the point to the array;

			cloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);
		}
	}

	
	// Add a reference point at 0, 0, 0.
	// This is used as baseline for color scheme of the range display.
	mViewerCoordinatesPtr->GetXYZFromRange(0.0, 0.0, 0.0,newCloudPoint);
	newCloudPoint.b = newCloudPoint.g = newCloudPoint.b = 0;
	newCloudPoint.a = 255;

	//	Add the point to the array;
	cloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);
}

bool ReceiverProjector::SetDisplayUnderZero(bool inDisplayUnderZero)
{
	// Update the displayStyle.

	displayUnderZero = inDisplayUnderZero;
	return (displayUnderZero);
}


bool ReceiverProjector::GetDisplayUnderZero()
{
	return (displayUnderZero);
}


void ReceiverProjector::SetDecimation(int inDecimationX, int inDecimationY)
{
	// Update the decimation values.

	decimationX = inDecimationX;
	decimationY = inDecimationY;
}


void ReceiverProjector::GetDecimation(int &outDecimationX, int &outDecimationY)
{
	outDecimationX = decimationX;
	outDecimationY = decimationY;
}

void ReceiverProjector::SetPositionUp(double inUp)
{

	up = inUp;
	mViewerCoordinatesPtr->SetPositionUp(up);
	int channelQty = receiverChannels.size();
	// For each of the channels, update local information
	for (int channelID = 0; channelID < channelQty; channelID++) 
	{
		GetChannel(channelID)->SetSensorUp(up);
	}
}

void ReceiverProjector::GetPositionUp(double &outUp)
{
	outUp = up;
}

void ReceiverProjector::SetPositionForward(double inForward)
{
	forward = inForward;
	mViewerCoordinatesPtr->SetPositionForward(inForward);
	int channelQty = receiverChannels.size();
	// For each of the channels, update local information
	for (int channelID = 0; channelID < channelQty; channelID++) 
	{
		GetChannel(channelID)->SetSensorForward(forward);
	}
}


void ReceiverProjector::GetPositionForward(double &outForward)
{
	outForward = forward;
}

void ReceiverProjector::GetRangeMax(double &outRangeMax)
{
	outRangeMax = rangeMax;
}

void ReceiverProjector::SetRangeMax(double inRangeMax)
{
	rangeMax = inRangeMax;
	int channelQty = receiverChannels.size();

#if 0
	// There is no management of the range max value in the coordinates ptr
	mViewerCoordinatesPtr->SetSensorUp(inSensorUp);
#endif
}

void ReceiverProjector::SetCameraFovWidth(double inFovWidth)
{
	cameraFovWidth = inFovWidth;
	mViewerCoordinatesPtr->SetCameraFovWidth(inFovWidth);
}

void ReceiverProjector::GetCameraFovHeight(double &outFovHeight)
{
	outFovHeight = cameraFovHeight;
}

void ReceiverProjector::SetCameraFovHeight(double inFovHeight)
{
	cameraFovHeight = inFovHeight;
	mViewerCoordinatesPtr->SetCameraFovHeight(inFovHeight);
}

void ReceiverProjector::GetCameraFovWidth(double &outFovWidth)
{
	outFovWidth = cameraFovWidth;
}

void ReceiverProjector::ResetCloud()
{
	size_t cloudSize = frameWidth*frameHeight*2;

	// Reset point cloud
	cloud->points.clear();
	cloud->points.reserve(cloudSize);

	// Define point cloud parameters
	cloud->width = static_cast<uint32_t>(frameWidth);
	cloud->height = static_cast<uint32_t>(frameHeight);
	cloud->is_dense = true;
}


ViewerCoordinates::ViewerCoordinates(const int inWidth, const int inHeight, const double inFovWidth, const double inFovHeight, 
	const double inUp, double inForward, double inRangeMax):
pcl::RangeImage(),
width(inWidth),
height(inHeight),
fovWidth(inFovWidth),
fovHeight(inFovHeight),
up(inUp),
forward(inForward),
rangeMax(inRangeMax)
{
   // We now want to create a range image from the above point cloud, with an angular resolution
   // that corresponds to the video resolution
  float angularResolutionX = float (inFovWidth / width);
  float angularResolutionY = float (inFovHeight / height);
  float maxAngleWidth     = (float) inFovWidth;  // 180.0 degree in radians
  float maxAngleHeight    = (float) inFovHeight;  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;

  pcl::RangeImage::createEmpty(angularResolutionX, angularResolutionY, sensorPose, coordinateFrame, maxAngleWidth,  maxAngleHeight);
}

void ViewerCoordinates::GetXYZFromRange(float inPointX, float inPointY, float inPointZ, 
									PointXYZRGB &ioCloudPoint)
{
	float angle_x, angle_y;
	getAnglesFromImagePoint(inPointX, inPointY, angle_x, angle_y);
	float cosY = cosf (angle_y);

	// We reverse the X coordinate to make this a left-handed projection
	// the PCL viewer uses a right-handed coordinate system.

	ioCloudPoint.x = -(inPointZ * sinf (angle_x) * cosY);
	ioCloudPoint.y = inPointZ * sinf (angle_y);
	ioCloudPoint.z = inPointZ * cosf (angle_x)*cosY;


	// We offset the values to the sensor offset position
	// depth is compensated for at receiver level, so we undo the offset here. 
	ioCloudPoint.z -= forward;
}

	/** \brief Sets   horizontal camera FOV.
      * \param[in] cameraFovWidth horizontal FOV of camera in radians.
	  */
void ViewerCoordinates::SetCameraFovWidth(double inCameraFovWidth)
{
	fovWidth = inCameraFovWidth;
}

	/** \brief Sets   verticsl camera FOV.
      * \param[in] cameraFovHeight vertical FOV of camera in radians.
	      */
void ViewerCoordinates::SetCameraFovHeight(double inCameraFovHeight)
{
	fovHeight = inCameraFovHeight;
}

void ViewerCoordinates::SetPositionUp(double inUp)
{
	up = inUp;

}

void ViewerCoordinates::GetPositionUp(double &outUp)
{
	outUp = up;
}

void ViewerCoordinates::SetPositionForward(double inForward)
{
	forward = inForward;
}

void ViewerCoordinates::GetPositionForward(double &outForward)
{
	outForward = forward;
}


