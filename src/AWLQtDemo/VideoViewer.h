#ifndef _VIDEOVIEWER_H
#define _VIDEOVIEWER_H

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>



using namespace std;

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"

#include "LoopedWorker.h"
#include "VideoCapture.h"
#include "Tracker.h"

namespace awl
{
/** \brief The VideoViewer class displays an image taken from a VideoCapture
           and overlays "decorations" based on the Detections provided.
  *        The video display class is a non-threaded class based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class VideoViewer: public LoopedWorker 
{
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;
	typedef boost::container::vector<FramePtr> FrameList;
	
	typedef boost::shared_ptr<VideoViewer> Ptr;
	typedef boost::shared_ptr<VideoViewer const> ConstPtr;
	typedef boost::container::vector<VideoViewer::Ptr> List;
	typedef VideoViewer::List *ListPtr;

	/** \brief Video Viewer constructor.
      * \param[in] inCameraName string used to identify camera and used as window title
      * \param[in] inVideoCapture videoCaptureDevice we feed image from.
	  * \param[in] inProjector receiverProjector that supplies us with range info
      */
	VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture);

	/** \brief Video Viewer destructor. Insures that the viewer is Stopped()
      */
	virtual VideoViewer::~VideoViewer();

	/** \brief Prepare the video display for the display loop.  Creates the display window.
      */
	void  Go(); 

	/** \brief Stop the video display. Destroys the display window.
      */
	void  Stop(); 

	/** \brief Return the video display status
      * \return true if the video display is stoppped (the display Window is not available).
      */
	bool  WasStopped();

	/** \brief Perform the video display update
      */
	void SpinOnce();

	/** \brief Return the video frame width.
      * \return videoframe width in pixels.
      */
	int	   GetFrameWidth() {return(frameWidth);}

	/** \brief Return the video frame width.
      * \return videoframe height in pixels.
      */
	int	   GetFrameHeight() {return(frameHeight);}

	/** \brief Return the  horizontal camera FOV.
      * \return horizontal camera FOV in radians.
      */
	double GetCameraFovWidth() {return(cameraFovWidth);}

	/** \brief Return the  vertical camera FOV.
      * \return vertical camera FOV in radians.
      */
	double GetCameraFovHeight() {return(cameraFovHeight);}
			

	/** \brief Move the window at position left, top.
	  * \remarks implemented for compatibility  with Qt (hence name convention).
      */
	void move(int left, int top); 

	/** \brief Update the detection positions.
      */
	void slotDetectionDataChanged(const Detection::Vector & data);

protected:
	/** \brief Modify the videoCapture source. 
			   Update the internal video format description variables to reflect the new source.
      * \param[in] inVideoCapture videoCaptureDevice we feed the image from
      */
	
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);


	/** \brief Resize the window to fit the screen.
	  * \remarks implemented for compatibility  with Qt.
      */
	void SizeWindow();

	void DisplayReceiverValues(VideoCapture::FramePtr &sourceFame, VideoCapture::FramePtr &targetFrame, const Detection::Vector & data);


	void DisplayTarget(VideoCapture::FramePtr &sourceFame, VideoCapture::FramePtr &targetFrame, const Detection::Ptr &detection);

	void DisplayCrossHairs(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame);

protected:
	void GetDetectionColors(const Detection::Ptr &detection, cv::Vec3b &colorEnhance, cv::Vec3b &colorDehance, int &iThickness);
	void GetChannelRect(const Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight);

	/** \brief Draw an "enhanced" detection line over the target frame.
	           We take the original background from the source frame.  
			   This way, we avoid a "pile-up" of enhancements that can be confusing.
			   Since the detections are sorted in order of threatLevel, this insures that the most menacing threats 
			   are always displayed correctly.
      */
    void DrawDetectionLine(VideoCapture::FramePtr &sourceFame, VideoCapture::FramePtr &targetFrame, 
						 const CvPoint &startPoint, const CvPoint &endPoint,  
						 const cv::Vec3b &colorEnhance, const cv::Vec3b &colorDehance, 
						 int iWidth, int iHeight);

	/** \brief Draw a contrasting lineline over the target frame.
	           We take the original background from the source frame. And paint the line gray: ligther on dark pixel, darker on light pixels;

			   This way, we avoid a "pile-up" of enhancements that can be confusing.
			   Since the detections are sorted in order of threatLevel, this insures that the most menacing threats 
			   are always displayed correctly.
      */

	void DrawContrastingLine(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, const CvPoint &startPoint, const CvPoint &endPoint,  int iWidth, int iHeight);

protected:
     /** \brief Current video frame width. */
	int frameWidth;

    /** \brief Current video frame height. */
	int frameHeight;

	/** \brief Horizontal field of view of the camera. */
	float cameraFovWidth;

	/** \brief Vertical field of view of the camera. */
	float cameraFovHeight;

	/** \brief Copy of the captured image. */
    VideoViewer::FramePtr cameraFrame;

	/** \brief Frames used for painting and display. 
	           As we are double-buffering, the displayed frame is
			   never the one that is being painted on.
	*/
	FrameList workFrames;

	/** \brief Index of frame currently being used for work, previous to display
	*/
	int currentWorkFrameIndex;

	/** \brief Camera name, also used as the window title */
	std::string cameraName;

	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief Sunscriber identification to the video feed. */
	Publisher::SubscriberID currentVideoSubscriberID;

	/** \brief Time the object was created.  Used to calculate flashing rates */
	boost::posix_time::ptime startTime;

	/** \brief Vector containing the detections to be displayed */
   Detection::Vector detectionData;
}; // VideoViewer

} // namespace awl


#endif //_CAPTUREVIEWER_H