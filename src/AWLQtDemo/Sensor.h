#ifndef _SENSOR_H
#define _SENSOR_H

/*
	Copyright 2014 Aerostar R&D Canada Inc.

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

#ifndef Q_MOC_RUN
#include <boost/container/vector.hpp>
#include <pcl/range_image/range_image.h>
#endif

#include "Publisher.h"
#include "LoopedWorker.h"
#include "VideoCapture.h"
#include "ReceiverCapture.h"

using namespace std;
using namespace pcl;

namespace awl
{
class ViewerCoordinates;
class Receiver;
class ReceiverChannel;

class ViewerCoordinates: public pcl::RangeImage
{
// Public types
public:
	typedef boost::shared_ptr<ViewerCoordinates> Ptr;
    typedef boost::shared_ptr<ViewerCoordinates > ConstPtr;

// protected variables
protected:

	/** \brief  width in pixels */
	int width;
	/** \brief  height in pixels */
	int height;

	/** \brief  FOV Width in radians */
	double fovWidth;
	/** \brief  FOV Height in radians */
	double fovHeight;

	/** \brief  position from ground */
	double  up;
	/** \brief  forward position from bumper (ideally, should be negative)*/
	double  forward;
	/** \brief  diplay plane for max range*/
	double rangeMax;

	// public variables
public:


// public functions
public:
	ViewerCoordinates(const int inWidth, const int inHeight, const double inFovWidth, const double inFovHeight, const double inUp, double inForward, double iRangeMax);

	void ViewerCoordinates::GetXYZFromRange(float inPointX, float inPointY, float inPointZ, 
									PointXYZRGB &ioCloudPoint);


	int GetWidth() {return (width);};
	int	GetHeight() {return(height);};

	/** \brief Modify the viewer's sensor upwards position parameter.
      * \param[in] inUp sensor upwards position, in meters
       */
	void SetPositionUp(double inUp);

	/** \brief Get the viewer's sensor upwards position in meters.
      * \param[out] outSensorHeight sensor height.
      */
	void GetPositionUp(double &outUp);

	/** \brief Modify the viewer's sensor forward position.
      * \param[in] inForward sensor forward position, in meters
      */

	void SetPositionForward(double inForward);

	/** \brief Get the viewer's sensor forward position, in meters.
      * \param[out] outSensorForward sensor depth.
      */
	void GetPositionForward(double &outForward);

	/** \brief Sets   horizontal camera FOV.
      * \param[in] cameraFovWidth horizontal FOV of camera in radians.
	      */
	void  SetCameraFovWidth(double inCameraFovWidth);

	/** \brief Return the  horizontal camera FOV.
      * \return horizontal camera FOV in radians.
      */
	double GetCameraFovWidth() {return(fovWidth);}

	/** \brief Sets   verticsl camera FOV.
      * \param[in] cameraFovHeight vertical FOV of camera in radians.
	      */
	void  SetCameraFovHeight(double inCameraFovHeight);

	/** \brief Return the  vertical camera FOV.
      * \return vertical camera FOV in radians.
      */
	double GetCameraFovHeight() {return(fovHeight);}
};


class ReceiverChannel
{
		friend class ReceiverProjector;

// Public types
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;
	typedef boost::shared_ptr<ReceiverChannel> Ptr;
    typedef boost::shared_ptr<ReceiverChannel > ConstPtr;

// Protected variables
protected:

	// Receiver channel descriptor
	int		receiverID;
	int		channelID;
	float	fovWidthX;
	float	fovWidthY;
	float	fovCenterX;
	float	fovCenterY;
	float   rangeMax;

	// Display colors
	double displayColorR;
	double displayColorG;
	double displayColorB;

	// Display options
	bool   displayUnderZero;
	int		decimationX;
	int		decimationY;

	// Current image ROI and correspondence parameters
	// Updated to reflect current cam with a call to UpdateViewerCoordinates()
	
	ViewerCoordinates::Ptr viewerCoordinatesPtr;

	int		imageWidth;
	int		imageHeight;
	int		topLeftX;
	int		topLeftY;
	int		bottomRightX;
	int		bottomRightY;

	/** \brief  sensor height to ground */
	double  sensorUp;
	/** \brief  sensor depth from bumber (ideally, should be negative)*/
	double  sensorForward;

	// Pointer to background during frame reconstruction
	ReceiverChannel::FramePtr backgroundPtr;
	// Color of current frame during frame reconstruction;
	ReceiverChannel::FramePtr colorPtr;
	// point-Cloud used for frame reconstructio
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud;

	// public variables
public:
	// List of Receiver points in RGB image covered by FOV
	boost::container::vector<CvPoint> maskPoints;

// protected methods
protected:

	// Adds a distance to the cloud, scanning the mask image only for the topLeft, bottomRight scope
	void AddDistanceToCloud(float inDistance, uint8_t inIntensity);

	void AddDistanceToCloud(const cv::Vec3b &mask,
						    const cv::Vec3b &video,
							cv::Vec3b &background,
							int xPos,
							int yPos, 
							float inDistance, 
							uint8_t inIntensity);

	void AddPointToCloud(float x, float y, float z,
						 uint8_t r,
						 uint8_t g,
						 uint8_t b,
						 uint8_t inIntensity);

	void ReceiverChannel::BuildPixelMask();

	// public methods
public:

	ReceiverChannel(const int inReceiverID,
					const int inChannelID, 
					const float inFovWidth, const float inFovHeight, 
					const float inCenterX, const float inCenterY, 
					const float inRangeMax, 
					bool inDisplayUnderZero = false, 
					double inDisplayColorR = 0, double inDisplayColorG = 255, double inDisplayColorB = 0);
	~ReceiverChannel();


	void AddDistancesToCloud(const Detection::Vector &detectionBuffer);

	/** \brief Obtain the  channel's bounding rectange at maximum range, projected in 3D space.
	           In current implementation, the value pairs may be un-ordered (minX may be greater than maxX).
	  * \param[out] minX value of the x coordinate of bottom left corner.
	  * \param[out] miny value of the y coordinate of bottom left corner.
	  * \param[out] minz value of the z coordinate of bottom left corner.
	  * \param[out] maxX value of the x coordinate of top right corner.
	  * \param[out] maxy value of the y coordinate of top right corner.
	  * \param[out] maxz value of the z coordinate of top right corner.
 	  * \param[out] originX value of the x coordinate of the sensor's origin.
	  * \param[out] originY value of the y coordinate of the sensor's origin.
	  * \param[out] originZ value of the z coordinate of the sensor's origin.
     */
	void GetChannelLimits(ViewerCoordinates::Ptr &inViewerCoordinates, double &minX, double &minY, double &minZ, 
		double &maxX, double &maxY, double &maxZ, double &originX, double &originY, double &originZ);

/** \brief Obtain the  channel's bounding rectange on the video frame.
	           In current implementation, the value pairs may be un-ordered (minX may be greater than maxX).
	  * \param[out] top y coordinate of the bounding rectangle.
	  * \param[out] left  left X coordinate of teh bounding rectable.
	  * \param[out] bottom y coordinate of the bottom of the bounding rectangle.
	  * \param[out] right value of the X the right extreme of bounding rectangle.
     */	
	
	void GetChannelRect(ViewerCoordinates::Ptr &inViewerCoordinates, int &top, int &left, int &bottom, int &right);

	void GetDisplayColor(double &outR, double &outG, double&outB);
	void UpdateViewerCoordinates(ViewerCoordinates::Ptr &inViewerCoordinates);

	/** \brief Modify the viewer display so as to hide/show all voxels under ground.
      * \param[in] inDisplayUnderZero If false, values under "-sensorUp"  not be displayed.  Displayed if true.
      */
	bool SetDisplayUnderZero(bool inDisplayUnderZero);
	
	/** \brief Get the value of the displayUnderZero display mode.
      * \return If false, values under "-sensorUp"  are not be displayed.  Displayed if true.
      */

	bool GetDisplayUnderZero();

	/** \brief Modify the channel's pixel decimation on the viewer.
	  *        A smaller decimation results in a higer resolution image, but less performance.
      * \param[in] inDecimation  Should be a positive number >= 1. Any number < 1 will default to 1.
      */
	void SetDecimation(int inDecimationX, int inDecimationY);

	/** \brief Get channel's pixel decimation on the viewer.
      * \param[out] outDecimation  Should be a positive number >= 1. Any number < 1 will default to 1.
      */
	void GetDecimation(int &outDecimationX, int &outDecimationY);

		/** \brief Modify the viewer's sensor height parameter.
      * \param[in] inSensorUp sensor height, in meters
      */
	void SetSensorUp(double inSensorUp);

	/** \brief Get the viewer's sensor height in meters.
      * \param[out] outSensorUp sensor height.
      */
	void GetSensorUp(double &outSensorUp);

	/** \brief Modify the viewer's sensor forrward position parameter (depth from bumper).
      * \param[in] inSensorForward sensor position (positive forward), in meters (normally negative)
      */
	void SetSensorForward(double inSensorForward);

	/** \brief Get the viewer's sensor forward position in meters.
      * \param[out] outSensorForward sensor depth.
      */
	void GetSensorForward(double &outSensorForward);

	/** \brief Modify the viewer's maximum display range.
      * \param[in] inRangeMax maximum range of the sensor, in meters
      */
	void SetRangeMax(double inRangeMax);

	/** \brief Get the viewer's maximum display range.
      * \param[out] outRangeMax maximum range of the sensor, in meters
      */
	void GetRangeMax(double &outRangeMax);

	void SetBackgroundPtr(ReceiverChannel::FramePtr inBackgroundPtr);
	void SetColorPtr(ReceiverChannel::FramePtr inColorPtr);
	void SetCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inCurrentCloud); 
}; // class ReceiverChannel

/** \brief ReceiverProjector class is used to control the projection of 3D data acquired from ReceiverChannels
  *        onto a point cloud, after adequate coordinate tranformation.  The projection also maps video RGB data to the
  *        individual points of the point-cloud.
  *		   The ReceiverProjector uses a VideoCaptureDevice as input.
  *		   The ReceiverProjector aslo used a PointCloud<XYZRGB>  as input
  * \author Jean-Yves Deschênes
  */
class ReceiverProjector: public LoopedWorker, public Publisher 
{
// Public types
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;

	typedef boost::shared_ptr<ReceiverProjector> Ptr;
    typedef boost::shared_ptr<ReceiverProjector> ConstPtr;

// public Methods
public:

	/** \brief ReceiverProjector constructor.
      * \param[in] inVideoCapture videoCaptureDevice we feed image from
      * \param[in] inCloud pointer to the cloud that is going t be projected upon.
      */

	ReceiverProjector(VideoCapture::Ptr videoCapture, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inCloud,
		ReceiverCapture::Ptr inReceiverCapture);

	/** \brief ReceiverProjector Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverProjector();

	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	virtual bool  WasStopped();

	/** \brief Update the display from the application's main loop or timer.
      */
	virtual void SpinOnce();


	/** \brief Return the number of receiver channels used for video projection
      * \return int indicating the number of channels.
      */
	int GetChannelQty() {return(receiverChannels.size());};

	/** \brief Return an individual Receiver Channel object
     * \param[in] inChannelID index of the required channel
     * \return Reference the channel that corresponds to the index.
     */
	ReceiverChannel::Ptr & GetChannel(int inChannelID);

	/** \brief Return an individual Receiver Channel object.
	  * \param[in] inChannelPtr reference to a ReceiverChannelObject that was created externally.
      * \return Reference the channel that corresponds to the index.
      */
	ReceiverChannel::Ptr &AddChannel(ReceiverChannel::Ptr &inChannelPtr);


	/** \brief Update the cloud with the current LIDAR data from each of the ReceiverChannels.
	         The cloud will first be cleared.
      */
	void AddDistancesToCloud();


	/** \brief Obtain the specified channel's bounding rectange at maximum range.
	           In current implementation, the value pairs may be un-ordered (minX may be greater than maxX).
	  * \param[in] inChannel channel index.
	  * \param[out] minX value of the x coordinate of bottom left corner.
	  * \param[out] miny value of the y coordinate of bottom left corner.
	  * \param[out] minz value of the z coordinate of bottom left corner.
	  * \param[out] maxX value of the x coordinate of top right corner.
	  * \param[out] maxy value of the y coordinate of top right corner.
	  * \param[out] maxz value of the z coordinate of top right corner.
 	  * \param[out] originX value of the x coordinate of the sensor's origin.
	  * \param[out] originY value of the y coordinate of the sensor's origin.
	  * \param[out] originZ value of the z coordinate of the sensor's origin.
     */
	void GetChannelLimits(int inChannel, double &minX, double &minY, double &minZ, double &maxX, double &maxY, double &maxZ, double &originX, double &originY, double &originZ);

	void GetChannelRect(int channel, int &top, int &left, int &bottom, int &right);

	/** \brief Obtain the color used to display the specified channel's bounding rectange at maximum range.
	  * \param[in] inChannel channel index.
	  * \param[out] r double value (0-1.0) of the red componnent.
	  * \param[out] g double value (0-1.0) of the red componnent.
	  * \param[out] b double value (0-1.0) of the red componnent.
      */
	void GetDisplayColor(int channel, double &outR, double &outG, double &outB);

	ReceiverProjector::FramePtr &GetCurrentFrame() {return(currentFrame);};
	void SetCurrentFrame(ReceiverProjector::FramePtr &inFrame);

	ReceiverProjector::FramePtr &GetBackgroundFrame() {return(backgroundFrame);};
	void SetBackgroundFrame(ReceiverProjector::FramePtr &inFrame);

	/** \brief Modify the viewer display so as to hide/show all voxels under ground.
      * \param[in] bDisplayUnderZero If false, values under "-sensorUp"  not be displayed.  Displayed if true.
      */
	bool SetDisplayUnderZero(bool inDisplayUnderZero);

	/** \brief Get the value of the displayUnderZero display mode.
      * \param[out] bDisplayUnderZero If false, values under "-sensorUp"  are not be displayed.  Displayed if true.
      */
	bool GetDisplayUnderZero();

	void SetDecimation(int inDecimationX, int inDecimationY);
	void GetDecimation(int &outDecimationX, int &outDecimationY);

	/** \brief Modify the viewer's sensor upwards position parameter.
      * \param[in] inUp sensor upwards position, in meters
      */
	void SetPositionUp(double inUp);

	/** \brief Get the viewer's sensor upwards position in meters.
      * \param[out] outSensorHeight sensor height.
      */
	void GetPositionUp(double &outUp);

	/** \brief Modify the viewer's sensor forward position.
      * \param[in] inForward sensor forward position, in meters
      */

	void SetPositionForward(double inForward);

	/** \brief Get the viewer's sensor forward position, in meters.
      * \param[out] outSensorForward sensor depth.
      */
	void GetPositionForward(double &outForward);

	/** \brief Modify the viewer's maximum display range.
      * \param[in] inRangeMax maximum range of the sensor, in meters
      */
	void SetRangeMax(double inRangeMax);

	/** \brief Get the viewer's maximum display range.
      * \param[out] outRangeMax maximum range of the sensor, in meters
      */
	void GetRangeMax(double &outRangeMax);

	/** \brief Modify the videoCapture source.  Thread safe
               Update the internal video format description variables to reflect the new source.
 	  * \param[in] inVideoCapture safe pointer to the video capture device.
     */
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);

	/** \brief Modify the receiver capture source.  Thread safe
               	  * \param[in] inVideoCapture safe pointer to the video capture device.
     */
	void SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture);

	/** \brief Modify the target point-cloud.  Thread safe
  	  * \param[in] inCloud safe pointer to the point cloud.
     */
	void SetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inCloud);

	/** \brief Copy the current point-cloud to the target point-cloud.  
	  *        The current point-cloud  is thread-locked during transfer
       * \param[out] outCloud pointer to the target point-cloud that will get copied to.
       * \param[in] inSubscriberID identification iof the subscriber getting the cloud.  Default -1 (none).
	   * \note Locking of the target point-cloud is under the responsibility of the calling thread.
      */
	void CopyCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outCloud, Publisher::SubscriberID inSubscriberID = -1);

	/** \brief Return the video frame width.
      * \return videoframe width in pixels.
      */
	int	   GetFrameWidth() {return(frameWidth);}

	/** \brief Return the video frame width.
      * \return videoframe height in pixels.
      */
	int	   GetFrameHeight() {return(frameHeight);}


	/** \brief Return the image scale.
      * \return image scaling factor.
      */
	double GetScale() { return(scale);}

	/** \brief Sets   horizontal camera FOV.
      * \param[in] cameraFovWidth horizontal FOV of camera in radians.
	      */
	void  SetCameraFovWidth(double cameraFovWidth);

	/** \brief Return the  horizontal camera FOV.
      * \param[out] cameraFovWidth horizontal FOV of camera in radians.
      */
	void GetCameraFovWidth(double &outCameraFov);

	/** \brief Sets   verticsl camera FOV.
      * \param[in] cameraFovHeight vertical FOV of camera in radians.
	      */
	void  SetCameraFovHeight(double cameraFovHeight);

	/** \brief Return the  vertical camera FOV.
      * \param[out] cameraFovWidth vertical FOV of camera in radians.
      */
	void GetCameraFovHeight(double &outCameraFov);

	// public variables
public:
	friend class ReceiverChannel;

	/** \brief Vector holding each of the individual lidar channels. */
	boost::container::vector<ReceiverChannel::Ptr> receiverChannels;

// Protected methods
protected:
	/** \brief Place the background image at a preset distance in the cloud.
     */
	void AddBackgroundToCloud();

	/** \briefClear the cloud points anbd adjust the coordinates
     */
	void ResetCloud();

// Protected variables
protected:
	protected:
	
    /** \brief Local flag indicating the termination of thread. */
	volatile bool mStopRequested;

	/** \brief Object used to support coordinate conversion */
	ViewerCoordinates::Ptr mViewerCoordinatesPtr;

	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief receiever capture device that supplies the lidar data */
	ReceiverCapture::Ptr receiverCapture; 


	/** \brief Pointer to the cloud that is being written onto */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	/** \brief Captured image. Should be for reference purposes. */
    ReceiverProjector::FramePtr currentFrame; 

	/** \brief Copy of the current frame that is messed up by projectors. */
    ReceiverProjector::FramePtr backgroundFrame; 

	
//	cv::Mat	mBackground;


	/** \brief Decimation of the columns to accelerate point-cloud display. */
	int		decimationX;
	/** \brief Decimation of the rows to accelerate point-cloud display. */
	int		decimationY;
	/** \brief Boolean indicates if we display lidar points that are below the ground line. */
	bool	displayUnderZero;
	/** \brief  position from ground */
	double  up;
	/** \brief  forward position from bumper (ideally, should be negative)*/
	double  forward;
	/** \brief  maximum range (which is also distance at whick we project image place)*/
	double  rangeMax;


	/** \brief Current video frame width. */
	int frameWidth;

    /** \brief Current video frame height. */
	int frameHeight;

	/** \brief Current video stream frame rate in FPS.  For still video, it defaults to 33FPS. */
	double frameRate;

	/** \brief Video scaling factor, that can be set on the command line. */
	double scale;

	/** \brief Horizontal field of view of the camera. */
	float cameraFovWidth;
	/** \brief Vertical field of view of the camera. */
	float cameraFovHeight;

	/** \brief Our subscription identifier to access to video frame. */
	Publisher::SubscriberID currentVideoSubscriberID;
	
	/** \brief Our subscription identifier to access to lidar data. */
	Publisher::SubscriberID currentReceiverCaptureSubscriberID;
};


} // namespace AWL

#endif



