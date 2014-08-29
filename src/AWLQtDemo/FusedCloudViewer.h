#ifndef _FUSEDCLOUDVIEWER_H
#define _FUSEDCLOUDVIEWER_H

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>

#ifndef Q_MOC_RUN
#include <boost/container/vector.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/histogram_visualizer.h>
#endif

#include "sensor.h"
#include "Publisher.h"
#include "LoopedWorker.h"

using namespace std;
using namespace pcl;


namespace awl
{

/** \brief Point-Cloud display class.
  *        The VieweDesciptor creates a local copy of the point-cloud and a 
  *        pcl::visualization::PCLVisualizer viewing windo to display it.
  * \author Jean-Yves Deschênes
  */
class  CloudViewerWin: public LoopedWorker
{
	
// Public types
public:
	typedef boost::shared_ptr<CloudViewerWin> Ptr;
    typedef boost::shared_ptr<CloudViewerWin > ConstPtr;
	typedef boost::container::vector<CloudViewerWin::Ptr> List;

	typedef enum ColorHandlerType 
	{
		eHandlerRGB=0, 
		eHandlerZ=1,
	} ColorHandlerType;

	typedef enum CameraView 
	{
		eCameraFront=0, 
		eCameraTop=1,
		eCameraSide=2, 
		eCameraIsometric=3,
		eCameraZoom=4
	} CameraView;


// Public methods
public:
	/** \brief Constructor for the CloudViewerWin object
	           The construction instantiates the viewer window.
	  * \param[in] inSourceProjector ReceiverProjector used as a point-cloud source
      * \param[in] inWindowName title of the viewer display  window (should be unique).
	  *
      */
	 CloudViewerWin(VideoCapture::Ptr inVideoCapture, ReceiverCapture::Ptr inReceiverCapture,
					const std::string &inWindowName);

	/** \brief Destructor for the CloudViewerWin object
    */
	virtual ~CloudViewerWin();

	/** \brief Initialize the variables needed by the display loop
	    \note The current implementation is not threaded.
		      Installs the callvbacks in the viewer window.
      */
	virtual void  Go(); 

	/** \brief Stop worker.
	  *        Clear the callbacks on the viewer window (avoids crashing the app).
      */
	virtual void  Stop(); 

	/** \brief Return theviewer status
      * \return true if the viewer thread is stoppped.
      */
	bool  WasStopped();

	/** \brief Go once through the display loop, to be called by main thread or timer event
      */
	void  SpinOnce();

	/** \brief Modify the color handker thread for the viewer
      * \param[in] inColorHandlerType  enumerated value indicating the color handler used for rendering.
	  * \return a pointer to the color handler that was just set.
      */
	pcl::visualization::PCLVisualizer::ColorHandler *  
	SetCurrentColorHandlerType(CloudViewerWin::ColorHandlerType inColorHandlerType);
	
	/** \brief Get the current color handler type identifier.
      * \return The enumerated value that identifies the colorHandler type.
      */
	CloudViewerWin::ColorHandlerType GetCurrentColorHandlerType();
	
	/** \brief Return a pointer to the current handler
      * \return a pointer to the color handler that was just set.
      */
	pcl::visualization::PCLVisualizer::ColorHandler * GetCurrentColorHandler();

	/** \brief Change the source projector that will be used as a point-cloud rendering source.  Thread safe.
    * \param[in] inSourceProjector  Pointer to the ReceiverProjector that was originally used.

      */

	/** \brief Change the pixel size used for rendering
    * \param[in] inpPixelSize new pixelSize

      */
	void SetPixelSize(int inPixelSize);

	/** \brief Get the pixel size used for rendering
    * \param[out] outPixelSize current pixelSize

      */	
	void GetPixelSize(int &outPixelSize);

	void SetSourceProjector(ReceiverProjector::Ptr inSourceProjector);
	/** \brief Modify the camera view angle on the viewer.
      * \param[in] inAngle  enumerated identifier of the camera angle.
      */
	void SetCameraView(CloudViewerWin::CameraView inAngle);

	/** \brief Modify the viewer's pixel decimation on the viewer.
	  *        A smaller decimation results in a higer resolution image, but less performance.
      * \param[in] inDecimation  Should be a positive number >= 1. Any number < 1 will default to 1.
      */
	void SetDecimation(int inDecimation);

	/** \brief Get viewer's pixel decimation on the viewer.
      * \param[out] outDecimation  Should be a positive number >= 1. Any number < 1 will default to 1.
      */
	void GetDecimation(int &outDecimation);


	/** \brief Modify the viewer display so as to hide/show all voxels under ground.
      * \param[in] bInDisplayUnderZero If false, values under "-sensorHeight"  not be displayed.  Displayed if true.
      */
	void SetDisplayUnderZero(bool bInDisplayUnderZero);

	/** \brief Get the value of the displayUnderZero display mode.
      * \param[out] bOutDisplayUnderZero If false, values under "-sensorHeight"  are not be displayed.  Displayed if true.
      */
	void GetDisplayUnderZero(bool &bOutDisplayUnderZero);

	/** \brief Modify the viewer's sensor upwards position parameter.
      * \param[in] inUp sensor upwards position, in meters
      * \param[in] bRedraw true to force redraw of the Window's grid and decorations
      */
	void SetPositionUp(double inUp, bool bRedraw = true);

	/** \brief Get the viewer's sensor upwards position in meters.
      * \param[out] outSensorHeight sensor height.
      */
	void GetPositionUp(double &outUp);

	/** \brief Modify the viewer's sensor forward position.
      * \param[in] inForward sensor forward position, in meters
      * \param[in] bRedraw true to force redraw of the Window's grid and decorations
      */

	void SetPositionForward(double inForward, bool bRedraw = true);

	/** \brief Get the viewer's sensor forward position, in meters.
      * \param[out] outSensorForward sensor depth.
      */
	void GetPositionForward(double &outForward);

	/** \brief Modify the viewer's maximum display range.
      * \param[in] inRangeMax maximum range of the sensor, in meters
      * \param[in] bRedraw true to force redraw of the Window's grid and decorations
      */
	void SetRangeMax(double inRangeMax , bool bRedraw = true);

	/** \brief Get the viewer's maximum display range.
      * \param[out] outRangeMax maximum range of the sensor, in meters
      */
	void GetRangeMax(double &outRangeMax);

	/** \brief Upfate the ranges and positions from the global configuration and 
	  * coordinate classes.
      */
	void UpdateFromGlobalConfig();

// Public variables
public:
	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief receiever capture device that supplies the lidar data */
	ReceiverCapture::Ptr receiverCapture; 

    /** \brief PCLVisualizer object that creates the viewing window. */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    /** \brief Point-Cloud object that is used to project the camera data prior to display. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloud;

    /** \brief Point-Cloud object that is used to display the cloud. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    /** \brief Unique identifier for the point-cloud object. */
	std::string cloudName;

    /** \brief Title of the window used to display the cloud. */
	std::string windowName;

    /** \brief ColorHandlerType enumerated value identifying the current colorHandler. */

	ColorHandlerType currentHandlerType;

	/** \brief Pointer to the colorHandler currently used for display. 
      */
	pcl::visualization::PCLVisualizer::ColorHandler *currentColorHandlerPtr;

   /** \brief RGB colorHandler. 
     *        These are created at viewer instantiation and are kept along for all of the life of the 
	 *        view.
     */
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb;

	/** \brief RGB colorHandler. 
      *        These are created at viewer instantiation and are kept along for all of the life of the 
	  *        view.
      */
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> handler_z;

protected:
	/** \brief Initialize the PCL Viewer Window
	*/
	void CreateView();

	/** \brief Initialize the PCL Viewer Window
	*/
	void CloudViewerWin::CreateReceiverProjector(VideoCapture::Ptr videoCapture, ReceiverCapture::Ptr receiverCapture);

	/** \brief  Draw the reverence grid on the PCL display point cloud.
	*/

	void DrawGrid();

	/** \brief Add a viewer line to the background
	*/
	void AddViewerLine(const PointXYZRGB &startPoint, const PointXYZRGB &endPoint, 
		double r, double g, double b,
		const std::string &inLineName); 

	/** \brief Add a viewer line to the background
	*/
	void AddViewerLine(float startX, float startY, float startZ,
					  float endX, float endY, float endZ,
					  double r, double g, double b,
					  const std::string &inLineName, int lineID=0); 

	/** \brief Modify the camera view angle on the viewer, from withing keyboard and mouse callbacks
	  *         The camera takes a viewer pointer, in order to be called during callbacks. Static is used for same purpose.
      * \param[in] viewer pointer to the PCL vidualizer.
      * \param[in] inAngle  enumerated identifier of the camera angle.
      */
	static void SetCameraView(boost::shared_ptr<pcl::visualization::PCLVisualizer> inViewer, CloudViewerWin::CameraView inAngle);

	
	/** \brief Process the keyboard callback from the pcl window.
	  *		   Adds the functionality of the view angle keys
	  * \Notes: Current keys are:
	  *\        "d" rear view, "t" topView, "s" side view, "i" isometric view (3/4 rear), "z" zoomed view.
      */
	static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

	/** \brief Process the mouse callback from the pcl window. Placeholder for now.
      */
	static void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void);
	
	/** \brief Modify the videoCapture source.  Thread safe
               Update the internal video format description variables to reflect the new source.
 	  * \param[in] inVideoCapture safe pointer to the video capture device.
     */
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);

	/** \brief Modify the receiver capture source.  Thread safe
               	  * \param[in] inVideoCapture safe pointer to the video capture device.
     */
	void SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture);
	
	/** \brief Pixel size holder. 
	    \remark  pixel size is not stored by the underlying VTK object  until 
		         AddPointCloud is performed.  This is why we store locally.
	*/
	int pixelSize;

	/** \brief Current CameraView angle
	*/
	CameraView cameraView;

	/** \brief the source projector used as a sinmk */
	ReceiverProjector::Ptr sourceProjector;

	/** \brief Our subscription identifier to access to the clud produced by projector. */
	Publisher::SubscriberID currentCloudSubscriberID;

	/** \brief Decimation of the columns to accelerate point-cloud display. */
	int		decimationX;
	/** \brief Decimation of the rows to accelerate point-cloud display. */
	int		decimationY;
	/** \brief Boolean indicates if we display lidar points that are below the ground line. */
	bool	bDisplayUnderZero;
	/** \brief  position from ground */
	double  up;
	/** \brief  forward position from bumper (ideally, should be negative)*/
	double  forward;
	/** \brief  maximum range (which is also distance at whick we project image place)*/
	double  rangeMax;

	/** \brief Horizontal field of view of the camera. */
	float cameraFovWidth;
	/** \brief Vertical field of view of the camera. */
	float cameraFovHeight;
};

}; // namespace awl

#endif