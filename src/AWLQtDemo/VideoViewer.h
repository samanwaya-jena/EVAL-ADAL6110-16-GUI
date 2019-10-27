#ifndef _VIDEOVIEWER_H
#define _VIDEOVIEWER_H

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

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>


#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>

#include "VideoCapture.h"
#include "DetectionStruct.h"

#include <QWidget>
#include <QFrame>
#include <QImage>
#include <QAction>
#include <QActionGroup>

namespace awl
{
/** \brief The VideoViewer class displays an image taken from a VideoCapture
           and overlays "decorations" based on the Detections provided.
  *        The video display class is a non-threaded class based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class VideoViewer: public QFrame

{
	Q_OBJECT
public:
	/** \brief Video Viewer constructor.
      * \param[in] inCameraName string used to identify camera and used as window title

	  * \param[in] inVideoCapture videoCaptureDevice we feed image from.
	  * \param[in] inProjector receiverProjector that supplies us with range info
      */
	explicit VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture, QWidget *parentWidget=0);

	typedef boost::shared_ptr<cv::Mat> FramePtr;
	typedef boost::container::vector<FramePtr> FrameList;
	
	typedef boost::shared_ptr<VideoViewer> Ptr;
	typedef boost::shared_ptr<VideoViewer const> ConstPtr;
	typedef boost::container::vector<VideoViewer::Ptr> List;
	typedef VideoViewer::List *ListPtr;


	/** \brief Video Viewer destructor. Insures that the viewer is Stopped()
      */
	virtual ~VideoViewer();

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

	QSize sizeHint() const;
	QSize minimumSizeHint() const;
	QSize maximumSizeHint() const;

public slots:
	void ShowContextMenu(const QPoint& pos);
	/** \brief Update the detection positions.
      */
	void slotDetectionDataChanged(const SENSORCORE_NAMESPACE_PREFIX::Detection::Vector & data);
	void slotImageChanged();
	void slotVideoOptionsChangedAction();


protected:
	/** \brief Modify the videoCapture source. 
			   Update the internal video format description variables to reflect the new source.
      * \param[in] inVideoCapture videoCaptureDevice we feed the image from
      */
	
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);


	/** \brief Resize the window to fit the screen.
      */
	void resizeEvent(QResizeEvent * /*event*/);

	void DisplayReceiverValues(QImage &sourceFame, QPainter& painter, const SENSORCORE_NAMESPACE_PREFIX::Detection::Vector & data);

protected:
	void paintEvent(QPaintEvent* /*event*/); 

	void GetDetectionColors(const SENSORCORE_NAMESPACE_PREFIX::Detection::Ptr &detection, QColor &colorEnhance,  int &iThickness);
	/** \brief get the four corners of the specified Detection's channel FOV, as projected at the Detection distance
	  *         in the camera plane.
	  *\return Returns false if all the points in the projection are behind the camera plane. Returns true
	  *        if at least one of the points is in front of the camera.
      */

	bool GetChannelRect(const SENSORCORE_NAMESPACE_PREFIX::Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight);

	

	void DisplayTarget(QImage &sourceFame, QPainter& painter, const SENSORCORE_NAMESPACE_PREFIX::Detection::Ptr &detection);

	void DisplayCrossHairs(QImage &sourceFame, QPainter& painter);

	/** \brief Draw an "enhanced" detection line over the target frame.
	           We take the original background from the source frame.  
			   This way, we avoid a "pile-up" of enhancements that can be confusing.
			   Since the detections are sorted in order of threatLevel, this insures that the most menacing threats 
			   are always displayed correctly.
      */
    void DrawDetectionLine(QImage &sourceFame, QPainter& painter,
						 const CvPoint &startPoint, const CvPoint &endPoint,  
						 QColor &colorEnhance,
						 int penWidth);

	void DrawHorizontalTicks(QImage &sourceFrame, QPainter& painter, float tickAngle, float tickLength, int thickness);
	void DrawVerticalTicks(QImage &sourceFrame,  QPainter& painter, float tickAngle, float tickLength, int thickness);
	void DrawChannelOutlines(QImage &sourceFrame, QPainter &painter);
	void DrawVideoText(QImage &sourceFrame, QPainter &painter, const QRect &textRect, const QString &text);
protected:
	/** \brief Copy of the last Camera image in OpenCV */
	cv::Mat lastValidFrame;

	/** \brief Copy of the captured image. */
    QImage qtCameraFrame;

	/** \brief Copy of the captured image, but with the image enhancements */
    QImage qtEnhancedFrame;


	/** \brief Camera name, also used as the window title */
	std::string cameraName;

	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief Sunscriber identification to the video feed. */
	SENSORCORE_NAMESPACE_PREFIX::Publisher::SubscriberID currentVideoSubscriberID;

	/** \brief Time the object was created.  Used to calculate flashing rates */
	boost::posix_time::ptime startTime;

	/** \brief Vector containing the detections to be displayed */
	SENSORCORE_NAMESPACE_PREFIX::Detection::Vector detectionData;

   /** \brief Screen scale factor in Qt Widget */
   float displayScaleFactor;

   /** displayCrosshair, as initially spaecified in configuration file */
   bool bDisplayCrosshair;

   /** displayTime, as initially spaecified in configuration file */
   bool bDisplayTime;

private:
	void createAction();

	QActionGroup* groupVideoOptions;
	QAction* crosshairOptionAction;
	QAction *timeOptionAction;
}; // VideoViewer

} // namespace awl


#endif //_CAPTUREVIEWER_H
