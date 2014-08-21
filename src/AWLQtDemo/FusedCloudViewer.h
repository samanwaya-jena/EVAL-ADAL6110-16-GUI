#ifndef _FUSEDCLOUDVIEWER_H
#define _FUSEDCLOUDVIEWER_H

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/container/vector.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/histogram_visualizer.h>
#endif

#include "sensor.h"
#include "Subscription.h"

using namespace std;
using namespace pcl;


namespace awl
{

/** \brief Threaded Point-Cloud display class.
  *        The VieweDesciptor creates a local copy of the point-cloud and a 
  *        pcl::visualization::PCLVisualizer viewing windo to display it.
  *		   The PointCloudColorHandler of the viewer can be modified while the thread runs.
  * \author Jean-Yves Deschênes
  */
class  CloudViewerWin
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
	           The construction instantiates the viewer window and viewer thread.
	  * \param[in] inSourceProjector ReceiverProjector used as a point-cloud source
	  * \param[in] inColorHandlerType identifier for the initial color handler
      * \param[in] inWindowName title of the viewer display  window.
      * \param[in] inCloudName title of the viewer display  window.
	  *
      */
	 CloudViewerWin(ReceiverProjector::Ptr &inSourceProjector,
					const ColorHandlerType inColorHandlerType,
					const std::string &inWindowName,
					const std::string &inCloudName);

	/** \brief Destructor for the CloudViewerWin object
	           Insures that thread is stopped.
    */
 virtual ~CloudViewerWin();


	/** \brief Start the viewer thread
	    \note In the current implementation, the threads start running continuously.
		      There does not seem to be a way of stopping them.
      */
	void  Go(); 

	/** \brief Stop the vewer thread thread
      */
	void  Stop(); 

	/** \brief Return theviewer thread status
      * \return true if the viewer thread is stoppped.
      */
	bool  WasStopped();

	/** \brief Allow the viewer to go through the vent loop.
	  * \param[in] time delay for which the viewer waits for events
      * \param[in] bForceRedraw  true to force immediate redraw of the viewer window.
      */
	void SpinOnce(int time=1, bool forceRedraw = false);


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


// Public variables
public:
    /** \brief PCLVisualizer object that creates the viewing window. */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

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

	/** \brief Modify the camera view angle on the viewer.
	           The camera takes a viewer pointer, in order to be called during callbacks. Static is used for same purpose.
      * \param[in] viewer pointer to the PCL vidualizer.
      * \param[in] inAngle  enumerated identifier of the camera angle.
      */
	static void SetCameraView(boost::shared_ptr<pcl::visualization::PCLVisualizer> inViewer, CloudViewerWin::CameraView inAngle);

	/** \brief Modify the viewer display so as to hide/show all voxels under ground.
      * \param[in] bDisplayUnderZero If false, values under "-sensorHeight"  not be displayed.  Displayed if true.
      */
	void SetDisplayUnderZero(bool bDisplayUnderZero);

	/** \brief Get the value of the displayUnderZero display mode.
      * \param[out] bDisplayUnderZero If false, values under "-sensorHeight"  are not be displayed.  Displayed if true.
      */
	void GetDisplayUnderZero(bool &bDisplayUnderZero);

	/** \brief Modify the viewer's sensor height parameter.
      * \param[in] inSensorHeight sensor height, in meters
      */
	void SetViewerHeight(double inSensorHeight);

	/** \brief Get the viewer's sensor height in meters.
      * \param[out] outSensorHeight sensor height.
      */
	void GetViewerHeight(double &sensorHeight);

	/** \brief Modify the viewer's sensor depth parameter.
      * \param[in] inSensorDepth sensor depth, in meters
      */

	void SetViewerDepth(double inSensorDepth);

	/** \brief Get the viewer's sensor depth in meters.
      * \param[out] outSensorDepth sensor depth.
      */
	void GetViewerDepth(double &sensorDepth);

	/** \brief Modify the viewer's maximum display range.
      * \param[in] inRangeMax maximum range of the sensor, in meters
      */
	void SetRangeMax(double inRangeMax);

	/** \brief Get the viewer's maximum display range.
      * \param[out] outRangeMax maximum range of the sensor, in meters
      */
	void GetRangeMax(double &outRangeMax);


	/** \brief Return the mutex for internal data.
      * \return a boost mutex for the object.
      */
	boost::mutex& GetMutex() {return (mMutex);} 

	/** \brief Go once through the thread loop, to be called by main thread
      */
	void  DoThreadIteration();

protected:
	/** \briefDo the thread's update and display loop
      * \return true if the video display thread is stoppped.
      */
	void  DoThreadLoop();

protected:
    /** \brief Pixel size holder. 
	    \remark  pixel size is not stored by the underlying VTK object  until 
		         AddPointCloud is performed.  This is why we store locally.
	*/
	int pixelSize;

    /** \brief Local flag indicating the termination of thread. */
	volatile bool mStopRequested;

    /** \brief Point-Cloud Viewer thread . */
    boost::shared_ptr<boost::thread> mThread;

	/** \brief Data sharing mutex. */
    boost::mutex mMutex;

	/** \brief the source projector used as a sinmk */
	ReceiverProjector::Ptr sourceProjector;

	/** \brief Our subscription identifier to access to the clud produced by projector. */
	Subscription::SubscriberID currentCloudSubscriberID;
};



class FusedCloudViewer
{

protected:

    volatile bool mStopRequested;

public:
	typedef boost::shared_ptr<FusedCloudViewer> Ptr;
    typedef boost::shared_ptr<FusedCloudViewer> ConstPtr;

	boost::container::vector<CloudViewerWin::Ptr> viewers;

	/** \brief the source projector used as a sinmk */
	ReceiverProjector::Ptr sourceProjector;
 
protected:
	void DrawGrid();

	void CreateViewerView();

	void AddViewerLines(const PointXYZRGB &startPoint, const PointXYZRGB &endPoint, 
		double r, double g, double b,
		const std::string &inLineName); 

	void AddViewerLines(float startX, float startY, float startZ,
					  float endX, float endY, float endZ,
					  double r, double g, double b,
					  const std::string &inLineName, int lineID=0); 
public:

	FusedCloudViewer(std::string inWindowName,  boost::shared_ptr<awl::ReceiverProjector> inReceiver);
	~FusedCloudViewer() {};

    // Create the thread and start work
    void Go(); 
 
    void Stop(); // Note 1
	bool WasStopped();


	void SpinOnce(int time=1, bool forceRedraw = false);
	void SpinSingleOnce(int viewerIndex, int time, bool forceRedraw=false);

	/** \brief Modify the viewer's height parameter.
      * \param[in] inViewerHeight sensor height, in meters
      */
	void SetViewerHeight(double inViewerHeight);

	/** \brief Get the viewer's  height in meters.
      * \param[out] outSensorHeight sensor height.
      */
	void GetViewerHeight(double &ViewerHeight);

		/** \brief Modify the viewer's  depth parameter.
      * \param[in] inSensorDepth  depth, in meters
      */

	void SetViewerDepth(double inViewerDepth);

	/** \brief Get the viewer's sensor depth in meters.
      * \param[out] outSensorDepth sensor depth.
      */
	void GetViewerDepth(double &viewerDepth);

	/** \brief Modify the viewer's maximum display range.
      * \param[in] inRangeMax maximum range of the sensor, in meters
      */
	void SetRangeMax(double inRangeMax);

	/** \brief Get the viewer's maximum display range.
      * \param[out] outRangeMax maximum range of the sensor, in meters
      */
	void GetRangeMax(double &outRangeMax);


protected:
	double sensorHeight;
	double sensorDepth;
	double rangeMax;

	int	   pixelSize;
   /** \brief Title of the window used to display the cloud. */
	std::string windowName;
};

}; // namespace awl

#endif