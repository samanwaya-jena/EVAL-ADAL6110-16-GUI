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

#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

#define CV_NO_BACKWARD_COMPATIBILITY

#include <fstream>

#include <boost/property_tree/ptree.hpp>

#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>

#include "Publisher.h"
#include "ThreadedWorker.h"
#include "CoordinateSystem.h"


namespace awl
{


/** \brief Threaded Video Capture class acquires and publishes image data from the Cameras.
  *        The video capture thread is based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class VideoCapture: public SensorCoreScope::ThreadedWorker, public SensorCoreScope::Publisher
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
	  * \param[in] propTree configuration property tree. Corresponds to configuration file contents.
      */
	VideoCapture(int inCameraID, int argc, char** argv, boost::property_tree::ptree &propTree);

	/** \brief Video Capture destructor.  Insures that the thread is stopped.

      */
	virtual ~VideoCapture();

	/** \brief Start the video acquisition thread
      */
	void  Go(); 

	/** \brief Return the camera ID.
      * \return cameraID (also is the index of the camera in our structures).
      */
	int GetCameraID() {return(cameraID);};


	/** \brief Return the video frame rate.
      * \return video acquisition frame rate in FPS.
      */
	float GetFrameRate() {return(frameRate);};

	/** \brief Copy the current frame to the targetFrame.  The current frame is thread-locked during transfer
      * \param[out] targetFrame pointer to the target frame that will get copied to.
      * \note Locking of the target frame is under the responsibility of the calling thread.
      */
	void CopyCurrentFrame(VideoCapture::FramePtr targetFrame,SensorCoreScope::Publisher::SubscriberID subscriberID = -1);

	/** \brief calibration information. */
	SensorCoreScope::CameraCalibration calibration;

protected:
	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Do a single iteration of the video capture loop.
	           (for calls from within the main event loop)
      */
	void  DoThreadIteration();

	/** \brief List all of the available cameras in file "CameraList.txt".
	           For debug purposes onlyle, the image is assumed to be 640x480 @ 30FPS.
  */
	void ListCameras();


	/** \brief With the configuration variable initialized, try to open the camera channel.
	           Set the calibration frameWidthInPixels, frameHeightInPixels and frameRate according 
			   to the information returned by the camera.
			   If no camera is available, the image is assumed to be 640x480 @ 30FPS.
		\return True if the camera is found and opened.  False otherwise.
	  */
	bool OpenCamera();

	/** \brief Read the configuration from configuration file
      */
	bool ReadConfigFromPropTree(boost::property_tree::ptree &propTree);

protected:
	
	/** \brief camera name */
	std::string sCameraName;

	/** \brief Index of the camera in the configuration parameter list */
	int cameraID;
	
    /** \brief Local flag indicating a request for termination */
	volatile bool mStopRequested;

	/** \brief Current video stram frame rate in FPS.  For still video, it defaults to 33FPS. */
	float frameRate;

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
