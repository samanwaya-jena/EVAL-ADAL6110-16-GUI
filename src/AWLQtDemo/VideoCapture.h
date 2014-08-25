#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

#define CV_NO_BACKWARD_COMPATIBILITY

#include <fstream>

using namespace std;

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include "publisher.h"
#include "ThreadedWorker.h"


namespace awl
{


/** \brief Threaded Video Capture class.
  *        The video capture thread is based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class VideoCapture: public ThreadedWorker, public Publisher
{
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;
	
	typedef boost::shared_ptr<VideoCapture> Ptr;
	typedef boost::shared_ptr<VideoCapture const> ConstPtr;
	typedef boost::container::vector<VideoCapture::Ptr> List;
	typedef VideoCapture::List *ListPtr;

	/** \brief Video Capture constructor.
      * \param[in] inCameraID index of the camera in the configuration parameters.
      * \param[in] argc command-line argument count.  Used in OpenCV cvInit() call.
      * \param[in] argv command line argument strings. Used in OpenCV cvInit() call.
      */
	VideoCapture(int inCameraID, int argc, char** argv);

	/** \brief Video Capture destructor.  Insures that the thread is stopped.

      */
	virtual ~VideoCapture();

	/** \brief Start the video acquisition thread
      */
	void  Go(); 

	/** \brief Return the camera ID.
      * \return cameraID (also is the index of the camera in our structures).
      */
	double GetCameraID() {return(cameraID);};


	/** \brief Return the video frame rate.
      * \return video acquisition frame rate in FPS.
      */
	double GetFrameRate() {return(frameRate);};

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


	/** \brief Return the  horizontal camera FOV.
      * \return horizontal camera FOV in radians.
      */
	double GetCameraFovWidth() {return(cameraFovWidth);}

	/** \brief Return the  vertical camera FOV.
      * \return vertical camera FOV in radians.
      */
	double GetCameraFovHeight() {return(cameraFovHeight);}

	/** \brief Copy the current frame to the targetFrame.  The current frame is thread-locked during transfer
      * \param[out] targetFrame pointer to the target frame that will get copied to.
      * \note Locking of the target frame is under the responsibility of the calling thread.
      */
	void CopyCurrentFrame(VideoCapture::FramePtr targetFrame, Publisher::SubscriberID subscriberID = -1);


protected:
	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Do a single iteration of the video capture loop.
	           (for calls from within the main event loop)
      */
	void  DoThreadIteration();

protected:
	/** \brief Index of the camera in the configuration parameter list */
	int cameraID;
	
    /** \brief Local flag indicating a request for termination */
	volatile bool mStopRequested;

    /** \brief Current video frame width. */
	int frameWidth;

    /** \brief Current video frame height. */
	int frameHeight;

	/** \brief Current video stram frame rate in FPS.  For still video, it defaults to 33FPS. */
	double frameRate;

	/** \brief Video scaling factor, that can be set on the command line. */
	double scale;

	/** \brief Horizontal field of view of the camera. */
	float cameraFovWidth;
	/** \brief Vertical field of view of the camera. */
	float cameraFovHeight;

	/** \brief Set cameraFlip to true for 180 degree rotation of the camera image. */
	bool bCameraFlip;

	/** \brief Video capture device. */
	cv::VideoCapture cam;

	/** \brief Captured image. Should be for reference purposes. */
    cv::Mat currentFrame; 

	/** \brief Captured image-directly from CVCapture. Should be for reference purposes. */
    cv::Mat bufferFrame; 
}; // VideoCapture

} // namespace awl

#endif //_VIDEOCAPTURE_H