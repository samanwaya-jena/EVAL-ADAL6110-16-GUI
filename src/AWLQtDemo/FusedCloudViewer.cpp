#include <iostream>


#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp> 
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#endif

#include "AWLSettings.h"
#include "sensor.h"
#include "FusedCloudViewer.h"

// Frame rate, in milliseconds

using namespace std;
using namespace pcl;
using namespace awl;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void);

const int cloudViewerSpinTime = 100;
const float interactorStillUpdateRate = 0.1;
const float interactorDesiredUpdateRate = 10;

CloudViewerWin::CloudViewerWin(ReceiverProjector::Ptr &inSourceProjector,
					const ColorHandlerType inColorHandlerType,
					const std::string &inWindowName,
					const std::string &inCloudName ):
sourceProjector(inSourceProjector),
cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
viewer(new pcl::visualization::PCLVisualizer (inWindowName)),
windowName(inWindowName),
cloudName(inCloudName),
handler_rgb(cloud),
handler_z (cloud, "z"),
currentHandlerType(inColorHandlerType),
mStopRequested(false),
mThread()

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	pixelSize = globalSettings->pixelSize;

	SetSourceProjector(sourceProjector);

	SetCurrentColorHandlerType(currentHandlerType);
	
	// Set callbacks for viewer
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
 }

CloudViewerWin::~CloudViewerWin()

{
	if (!WasStopped()) Stop();
}

pcl::visualization::PCLVisualizer::ColorHandler * 
CloudViewerWin::SetCurrentColorHandlerType(ColorHandlerType inColorHandlerType)
{
	currentHandlerType = inColorHandlerType;

	switch(inColorHandlerType) 
	{
		case eHandlerRGB:
			currentColorHandlerPtr = (pcl::visualization::PCLVisualizer::ColorHandler *) &handler_rgb;
			break;
		case eHandlerZ:
			currentColorHandlerPtr = (pcl::visualization::PCLVisualizer::ColorHandler *)&handler_z;
			break;
		default:
			currentColorHandlerPtr = (pcl::visualization::PCLVisualizer::ColorHandler *) &handler_rgb;
			break;
	}

	return(currentColorHandlerPtr);
}

CloudViewerWin::ColorHandlerType CloudViewerWin::GetCurrentColorHandlerType()
{
	return(currentHandlerType);
}

pcl::visualization::PCLVisualizer::ColorHandler *
CloudViewerWin::GetCurrentColorHandler()
{
	return(currentColorHandlerPtr);
}

void 
CloudViewerWin::SetSourceProjector(ReceiverProjector::Ptr inSourceProjector)
{
	boost::mutex::scoped_lock updateLock(mMutex);
 
	sourceProjector = inSourceProjector;
	currentCloudSubscriberID = sourceProjector->currentCloudSubscriptions->Subscribe();


	updateLock.unlock();
}

void CloudViewerWin::Go() 
{
	assert(!mThread);
	mStopRequested = false;
//	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(& CloudViewerWin::DoThreadLoop, this)));
}
 

void   CloudViewerWin::Stop() 
{
	if (mStopRequested) return;
	mStopRequested = true;

#if 0
	if (mThread) 
	{
		assert(mThread);
		mThread->join();
		m_thread = NULL;
	}
#endif

	// We should close the window following this strange sequence.
	// Otherwise, the destruction of the VTK Window underneath the PCL Viewer
	// finalizes the application
	HWND viewerWnd = (HWND)  viewer->getRenderWindow()->GetGenericWindowId();
	vtkRenderWindowInteractor* interactor = viewer->getRenderWindow()->GetInteractor();
	interactor->GetRenderWindow()->Finalize();
	viewer->getRenderWindow()->SetWindowId(0);

	viewer.reset();

	if (viewerWnd) DestroyWindow(viewerWnd);

}

bool  CloudViewerWin::WasStopped()
{
	if (mStopRequested) return(true);

	if (viewer->wasStopped())
	{
		Stop();
		return(true);
	}

	if (sourceProjector->WasStopped())
	{
		Stop();
		return(true);
	}

	return(false);
}

void CloudViewerWin::SpinOnce(int time, bool forceRedraw)

{
	viewer->spinOnce(time, forceRedraw);
}

void  CloudViewerWin::DoThreadLoop()
{
	while (!WasStopped())
	{
		DoThreadIteration();
	}
}

void  CloudViewerWin::DoThreadIteration()
{
	if (!WasStopped())
	{
		// Note: Check for update
		if (sourceProjector != NULL && sourceProjector->currentCloudSubscriptions->HasNews(currentCloudSubscriberID)) 
		{
			boost::mutex::scoped_lock updateLock(mMutex);
			sourceProjector->CopyCurrentCloud(cloud, currentCloudSubscriberID);
			if (!viewer->updatePointCloud(cloud,
					(pcl::visualization::PointCloudColorHandler <PointXYZRGB> &) *(currentColorHandlerPtr), 
					cloudName)) 
			{
				viewer->addPointCloud(cloud,
					(pcl::visualization::PointCloudColorHandler <PointXYZRGB> &) *(currentColorHandlerPtr), 
					cloudName);

				// Set the pixel size only once the cloud is added.
				SetPixelSize(pixelSize);
			}

			updateLock.unlock();

			SpinOnce(cloudViewerSpinTime);
		}
	}
}

void CloudViewerWin::SetCameraView(CloudViewerWin::CameraView inAngle)
{
	SetCameraView(viewer, inAngle);
}

void CloudViewerWin::SetDecimation(int inDecimation)
{
	if (inDecimation < 1) inDecimation = 1;

	sourceProjector->SetDecimation(inDecimation, inDecimation);
}

void CloudViewerWin::GetDecimation(int &outDecimation)
{
	int decimationX;
	int decimationY;

	sourceProjector->GetDecimation(decimationX, decimationY);
	outDecimation = decimationX;
}

void CloudViewerWin::SetDisplayUnderZero(bool bDisplayUnderZero)
{
	sourceProjector->SetDisplayUnderZero(bDisplayUnderZero);
}

void CloudViewerWin::GetDisplayUnderZero(bool &bDisplayUnderZero)
{
	int decimationX;
	int decimationY;

	bDisplayUnderZero = sourceProjector->GetDisplayUnderZero();
}

void CloudViewerWin::SetPixelSize(int inPixelSize)
{
	double dSize = (double) inPixelSize;
	pixelSize = inPixelSize;

	// Note that the pixelSize is not set correctly in pcl object until addPointCloud is called.
	// This is why we also keep pixelize in a member variable.
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dSize, cloudName);
}

void CloudViewerWin::GetPixelSize(int &outPixelSize)
{
	double dSize;
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dSize, cloudName);

	// Note that the pixelSize is not set correctly in pcl ovbject until addPointCloud is called.
	// This is why we also keep pixelize in a member variable.
	if (dSize < 1)
	{
		outPixelSize = pixelSize; 
	}
	else
	{
		outPixelSize = dSize;
	}
}

void CloudViewerWin::SetSensorHeight(double inSensorHeight)
{
	sourceProjector->SetSensorHeight(inSensorHeight);
}

void CloudViewerWin::GetSensorHeight(double &outSensorHeight)
{
	double  decimationX;
	int decimationY;

	sourceProjector->GetSensorHeight(outSensorHeight);
}

void CloudViewerWin::SetSensorDepth(double inSensorDepth)
{
	sourceProjector->SetSensorDepth(inSensorDepth);
}

void CloudViewerWin::GetSensorDepth(double &outSensorDepth)
{
	double  decimationX;
	int decimationY;

	sourceProjector->GetSensorDepth(outSensorDepth);
}


void CloudViewerWin::SetRangeMax(double inRangeMax)
{
	sourceProjector->SetRangeMax(inRangeMax);
}

void CloudViewerWin::GetRangeMax(double &outRangeMax)
{
	sourceProjector->GetRangeMax(outRangeMax);
}

void CloudViewerWin::SetCameraView(boost::shared_ptr<pcl::visualization::PCLVisualizer> inViewer, CloudViewerWin::CameraView inAngle)
{
	double topView[11] = {55, 115, 0, -8, 25, -1.35, 82, 25, 1.0, 0, 0};
	double sideView[11] = {42, 125, -8, 1, 25, -80, -2 , 20, 0, 1, 0};
	double isoView[11] = {25,160, 2.25,-0.2, 26, -52, 56, -9, 0.6, 0.75, 0.25};	
	double frontView[11] = {-2, 200, 0,0, 50, 0, 10, -30, 0, 1, 0};
	double zoomView[11] = {-2, 120, 0, -8, 5, -1.35, 24, 5, 1.0, 0, 0};

	double *viewData = topView;

	pcl::visualization::Camera *camera = &inViewer->camera_;

	switch(inAngle) 
	{
	case CloudViewerWin::eCameraFront:
		viewData = frontView;
		break;

	case CloudViewerWin::eCameraTop:
		viewData = topView;
		break;

	case CloudViewerWin::eCameraSide:
		viewData = sideView;
		break;

	case CloudViewerWin::eCameraIsometric:
		viewData = isoView;
		break;

	case CloudViewerWin::eCameraZoom:
		viewData = zoomView;
		break;

	default:
		viewData = isoView;
		break;
	}

	camera->clip[0]= viewData[0];
	camera->clip[1]= viewData[1];
	camera->focal[0] = viewData[2];
	camera->focal[1] = viewData[3];
	camera->focal[2] = viewData[4];
	camera->pos[0] = viewData[5];
	camera->pos[1]= viewData[6];
	camera->pos[2]= viewData[7];
	camera->view[0] = viewData[8];
	camera->view[1] = viewData[9];
	camera->view[2] = viewData[10];

	inViewer->updateCamera();
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

	if (event.getKeySym () == "d" && event.keyDown ()) 
	{
		CloudViewerWin::SetCameraView(viewer, CloudViewerWin::eCameraFront);
	}
	else if (event.getKeySym () == "t" && event.keyDown ()) 
	{
		CloudViewerWin::SetCameraView(viewer, CloudViewerWin::eCameraTop);
	}
	else if (event.getKeySym () == "s" && event.keyDown ()) 
	{
		CloudViewerWin::SetCameraView(viewer, CloudViewerWin::eCameraSide);

	}
	else if (event.getKeySym () == "i" && event.keyDown ()) 
	{
		CloudViewerWin::SetCameraView(viewer, CloudViewerWin::eCameraIsometric);
	}
	else if (event.getKeySym () == "z" && event.keyDown ()) 
	{
		CloudViewerWin::SetCameraView(viewer, CloudViewerWin::eCameraZoom);
	}
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
}

FusedCloudViewer::FusedCloudViewer(std::string inWindowName, boost::shared_ptr<awl::ReceiverProjector> inReceiver)
:
mStopRequested(false),
sourceProjector(inReceiver),
windowName(inWindowName),
viewers()
{
  
}

void FusedCloudViewer::CreateViewerView()

{
  CloudViewerWin::Ptr viewerPtr(new CloudViewerWin(sourceProjector, CloudViewerWin::eHandlerRGB, windowName, "Fused Cloud"));
  viewers.push_back(viewerPtr);

  viewerPtr->viewer->setBackgroundColor(0.3, 0.3, 0.3);
  viewerPtr->viewer->addText("Top View", 10, 10, "v1 text");
  viewerPtr->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,viewerPtr->cloudName);

  // Set Camera for isometric view
  viewerPtr->viewer->initCameraParameters();

  pcl::visualization::Camera *camera = &viewerPtr->viewer->camera_;

  camera->fovy = ((float) DEG2RAD(40));
  viewerPtr->SetCameraView(CloudViewerWin::eCameraIsometric); 
  
 // Set callback for viewer
  viewerPtr->viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewerPtr->viewer);
  viewerPtr->viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewerPtr->viewer);
  
   //Size the window
  vtkSmartPointer<vtkRenderWindow> window = viewerPtr->viewer->getRenderWindow ();
  window->SetPosition(0, 0);
  window->SetSize(600, 300);


  window->GetInteractor()->SetStillUpdateRate(interactorStillUpdateRate);
  window->GetInteractor()->SetDesiredUpdateRate(interactorDesiredUpdateRate);

  // Draw the reference grid on both windows
  DrawGrid();

    // Set the pixelSize
  viewerPtr->SetPixelSize(AWLSettings::GetGlobalSettings()->pixelSize);
}

void FusedCloudViewer::AddViewerLines(const PointXYZRGB &startPoint, const PointXYZRGB &endPoint, 
		double r, double g, double b,
		const std::string &inLineName) 
{
	int viewerQty(viewers.size());	
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->viewer->addLine<PointXYZRGB, PointXYZRGB>(startPoint, endPoint, r, g, b, inLineName);
	}
}

void FusedCloudViewer::AddViewerLines(float startX, float startY, float startZ,
									  float endX, float endY, float endZ,
									  double r, double g, double b,
									  const std::string &inLineName, int lineID) 
{
	PointXYZRGB startPoint;
	startPoint.x = startX;
	startPoint.y = startY;
	startPoint.z = startZ;

	PointXYZRGB endPoint;
	endPoint.x = endX;
	endPoint.y = endY;
	endPoint.z = endZ;

	std::string lineName = inLineName + "_" + std::to_string((_Longlong)lineID);

	AddViewerLines(startPoint, endPoint, r, g, b, lineName);
}

void FusedCloudViewer::DrawGrid()
{
	// In case we are called twice, avoid duplication of shapes
	int viewerQty(viewers.size());	
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->viewer->removeAllShapes();
	}


	// Add our shapes
	double originX = 0.0;
	double originY = 0.0;
	double originZ = 0.0;
	sourceProjector->GetSensorHeight(originY);
	sourceProjector->GetSensorDepth(originZ);

	double rangeMax = 0.0;
	sourceProjector->GetRangeMax(rangeMax);

	// draw a grid to correspond to sensor FOV 
	int channelQty = sourceProjector->GetChannelQty();

	for (int channelIndex =  0; channelIndex < channelQty; channelIndex++)  
	{
		double minX;
		double minY;
		double minZ;
		double maxX;
		double maxY;
		double maxZ;

		sourceProjector->GetChannelLimits(channelIndex, minX, minY, minZ, maxX, maxY, maxZ, originX, originY, originZ);
		double displayZ = maxZ;
		if (minZ > maxZ) displayZ = minZ;

		double r;
		double g;
		double b;
		sourceProjector->GetDisplayColor(channelIndex, r, g, b);

		AddViewerLines(0, 0, 0, minX, -minY, displayZ, r, g, b, "Receiver1", channelIndex);

		AddViewerLines(0, 0, 0, minX, -maxY, displayZ, r, g, b, "Receiver2", channelIndex);

		AddViewerLines(0, 0, 0, maxX, -minY, displayZ, r, g, b, "Receiver3", channelIndex);
	
		AddViewerLines(0, 0, 0, maxX, -maxY, displayZ, r, g, b, "Receiver4", channelIndex);

		AddViewerLines(minX, -minY, displayZ, maxX, -minY, displayZ, r, g, b, "Receiver5", channelIndex);

		AddViewerLines(minX, -maxY, displayZ, maxX, -maxY, displayZ, r, g, b, "Receiver6", channelIndex);

		AddViewerLines(minX, -minY, displayZ, minX, -maxY, displayZ, r, g, b, "Receiver7", channelIndex);

		AddViewerLines(maxX, -minY, displayZ, maxX, -maxY, displayZ, r, g, b, "Receiver8", channelIndex);
	}


		// draw a block grid to the ground 
	    // The ground is at negative offset from the sensor's position.
		// The sensor is always considered at 0, 0, 0.
	    // This is a cheat, but it will work for now.


	for (double x = (-10.0-originX); x <= (10.0-originX); x += 2.0)  
	{
		AddViewerLines(x, -originY, -originZ, x, -originY, rangeMax-originZ, 0, 0, 0, "GroundLineX", (int) x);
	}


	for (double z= 0.0-originZ; z <= rangeMax-originZ; z += 2.0)  
	{
		double shade =  z*3.0/255.0;
		AddViewerLines(-10.0-originX, -originY, z, 10-originX, -originY , z, shade, shade, shade, "GroundLineZ", (int) z);
	}
}

void  FusedCloudViewer::Go() 
{

	int viewerQty(viewers.size());
	if (!viewerQty)
	{
		CreateViewerView();
	}

	viewerQty = viewers.size();

	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->Go();
	}

	mStopRequested = false;
}
 

void  FusedCloudViewer::Stop() 
{
	if (mStopRequested) return;

    mStopRequested = true;
	int viewerQty(viewers.size());
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->Stop();
	}

	viewers.clear();
}
	
bool  FusedCloudViewer::WasStopped()
{
	if (mStopRequested) return(true);

	int viewerQty(viewers.size());
	for (int i = 0; i < viewerQty; i++) 
	{
		if (viewers[i]->viewer->wasStopped())
		{
			Stop();
			return(true);
		}
	}

	return(false);
}

void FusedCloudViewer::SpinOnce(int time, bool forceRedraw)

{	
	int viewerQty(viewers.size());
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->SpinOnce(time, forceRedraw);
	}
}


void  FusedCloudViewer::SetSensorHeight(double inSensorHeight) 
{
	if (mStopRequested) return;

    mStopRequested = true;
	int viewerQty(viewers.size());
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->SetSensorHeight(inSensorHeight);
	}
}

void  FusedCloudViewer::SetSensorDepth(double inSensorDepth) 
{
	if (mStopRequested) return;

    mStopRequested = true;
	int viewerQty(viewers.size());
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->SetSensorDepth(inSensorDepth);
	}
}

void  FusedCloudViewer::SetRangeMax(double inRangeMax) 
{
	if (mStopRequested) return;

    mStopRequested = true;
	int viewerQty(viewers.size());
	for (int i = 0; i < viewerQty; i++) 
	{
		viewers[i]->SetSensorHeight(inRangeMax);
	}
}
