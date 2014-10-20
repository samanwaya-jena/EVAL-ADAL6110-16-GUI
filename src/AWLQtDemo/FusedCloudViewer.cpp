/* FusedCloudViewer.cpp */
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
#include "awlcoord.h"
#include "sensor.h"
#include "FusedCloudViewer.h"

// Frame rate, in milliseconds

using namespace std;
using namespace pcl;
using namespace awl;


const int cloudViewerSpinTime = 100;
const float interactorStillUpdateRate = 0.1;
const float interactorDesiredUpdateRate = 10;

CloudViewerWin::CloudViewerWin(VideoCapture::Ptr inVideoCapture, ReceiverCapture::Ptr inReceiverCapture,
					const std::string &inWindowName):
LoopedWorker(), 
videoCapture(inVideoCapture),
receiverCapture(inReceiverCapture),
sourceProjector(),
cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
viewer(),
windowName(inWindowName),
cloudName(inWindowName),
handler_rgb(cloud),
handler_z (cloud, "z"),
currentHandlerType(CloudViewerWin::eHandlerRGB),
bDisplayUnderZero(true)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	AWLCoordinates *globalCoord = AWLCoordinates::GetGlobalCoordinates();
	pixelSize = globalSettings->pixelSize;

	SetCurrentColorHandlerType(currentHandlerType);

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

 }

CloudViewerWin::~CloudViewerWin()

{
	Stop();
}

void CloudViewerWin::Go()

{
	// Instanciate the projector
	CreateReceiverProjector(videoCapture, receiverCapture);
	sourceProjector->Go();


	// Instanciate viewer, and set default operation mode.
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(windowName));
	CreateView();

	// Set callbacks for viewer
	viewer->registerKeyboardCallback (CloudViewerWin::keyboardEventOccurred, (void*)&viewer);
	viewer->registerMouseCallback (CloudViewerWin::mouseEventOccurred, (void*)&viewer);
	LoopedWorker::Go();
}

void CloudViewerWin::Stop()

{
	if (!mWorkerRunning) return;
	if (!viewer.get())	return;

	if (sourceProjector.get()) sourceProjector->Stop();
	//Allow the viewer to go through the VTK / PCL event loop one last time, 
	//to clearspurious messaging.
  	if (!viewer->wasStopped()) 
	{
		viewer->spinOnce(cloudViewerSpinTime, false);
		// Clear the callbacks from the window
		viewer->registerKeyboardCallback (NULL, (void*)&viewer);
		viewer->registerMouseCallback (NULL, (void*)&viewer);
	}

	viewer.reset();

	if (sourceProjector.get()) sourceProjector.reset();
	LoopedWorker::Stop();
}


bool  CloudViewerWin::WasStopped()
{
	if (LoopedWorker::WasStopped()) return(true);
	if (!viewer.get()) return(true);
	if (viewer->wasStopped())
	{
		Stop();
		return(true);
	}

	if (!sourceProjector.get()) return(true);
	if (sourceProjector->WasStopped())
	{
		Stop();
		return(true);
	}

	HWND viewerWnd = (HWND)  viewer->getRenderWindow()->GetGenericWindowId();
	if (!viewerWnd || sourceProjector->WasStopped())
	{
		Stop();
		return(true);
	}

	return(false);
}

void  CloudViewerWin::SpinOnce()
{
	if (!WasStopped())
	{
		// Note: Check for update
		if (sourceProjector.get())
		{
			sourceProjector->SpinOnce();
		}

		if (sourceProjector.get() && sourceProjector->HasNews(currentCloudSubscriberID)) 
		{
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
		}

		//Allow the viewer to go through the VTK / PCL event loop.
	  	viewer->spinOnce(cloudViewerSpinTime, false);
	}
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

void CloudViewerWin::SetCameraView(CloudViewerWin::CameraView inAngle)
{
	cameraView = inAngle;
	if (viewer.get()) SetCameraView(viewer, inAngle);
}

void CloudViewerWin::SetDecimation(int inDecimation)
{
	if (inDecimation < 1) inDecimation = 1;
	decimationX = inDecimation;
	decimationY = inDecimation;
	if (sourceProjector.get()) sourceProjector->SetDecimation(decimationX, decimationY);
}

void CloudViewerWin::GetDecimation(int &outDecimation)
{
	outDecimation = decimationX;
}

void CloudViewerWin::SetDisplayUnderZero(bool bInDisplayUnderZero)
{
	bDisplayUnderZero = bInDisplayUnderZero;
	if (sourceProjector.get()) sourceProjector->SetDisplayUnderZero(bDisplayUnderZero);
}

void CloudViewerWin::GetDisplayUnderZero(bool &bOutDisplayUnderZero)
{
	bOutDisplayUnderZero = bDisplayUnderZero;
}

void CloudViewerWin::SetPixelSize(int inPixelSize)
{
	double dSize = (double) inPixelSize;
	pixelSize = inPixelSize;

	// Note that the pixelSize is not set correctly in pcl object until addPointCloud is called.
	// This is why we also keep pixelize in a member variable.
	if (viewer.get()) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dSize, cloudName);
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

void CloudViewerWin::SetPositionUp(double inUp, bool bRedraw)
{
	up = inUp;
	if (sourceProjector.get()) sourceProjector->SetPositionUp(inUp);
	if (bRedraw) DrawGrid();
}

void CloudViewerWin::GetPositionUp(double &outUp)
{
	outUp = up;
}

void CloudViewerWin::SetPositionForward(double inForward, bool bRedraw)
{
	forward = inForward;
	if (sourceProjector.get()) sourceProjector->SetPositionForward(forward);
	if (bRedraw) DrawGrid();
}

void CloudViewerWin::GetPositionForward(double &outForward)
{
	outForward = forward;
}


void CloudViewerWin::SetRangeMax(double inRangeMax, bool bRedraw)
{
	rangeMax = inRangeMax;
	if (sourceProjector.get()) sourceProjector->SetRangeMax(inRangeMax);
	if (bRedraw) DrawGrid();
}

void CloudViewerWin::GetRangeMax(double &outRangeMax)
{
	outRangeMax = rangeMax;
}

void CloudViewerWin::UpdateFromGlobalConfig()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	AWLCoordinates *globalCoord = AWLCoordinates::GetGlobalCoordinates();

	int receiverID = receiverCapture->GetReceiverID();
	CartesianCoord relativeCoord(0, 0, 0);
	CartesianCoord worldCoord(0,0,0);
	worldCoord = globalCoord->GetReceiver(receiverID)->ToReferenceCoord(eSensorToWorldCoord, relativeCoord);

	SetPositionForward(worldCoord.up, false);
	SetPositionUp(worldCoord.forward, false);
	SetRangeMax(globalSettings->receiverSettings[receiverID].displayedRangeMax , false);
	SetDecimation(globalSettings->decimation);
	SetDisplayUnderZero(bDisplayUnderZero);

	// Update the ReceiverChannel ranges
	int channelQty = 0;
	if (sourceProjector.get()) channelQty = sourceProjector->GetChannelQty();
	for (int channelID = 0; channelID < channelQty; channelID++)
	{
		sourceProjector->GetChannel(channelID)->SetRangeMax(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].maxRange);
	}

	DrawGrid();
}




void CloudViewerWin::CreateView()

{
  AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

  // Do not stay here if the viewer has not been instanciated.
  if (!viewer.get()) return;

  viewer->setBackgroundColor(0.3, 0.3, 0.3);
  viewer->addText("Top View", 10, 10, "v1 text");
  SetPixelSize(AWLSettings::GetGlobalSettings()->pixelSize);

  // Set Camera for isometric view
  viewer->initCameraParameters();
  pcl::visualization::Camera *camera = &viewer->camera_;
  camera->fovy = videoCapture->calibration.fovHeight;
  SetCameraView(CloudViewerWin::eCameraIsometric); 
  
   //Size the window
  vtkSmartPointer<vtkRenderWindow> window = viewer->getRenderWindow ();
  window->SetPosition(0, 0);
  window->SetSize(600, 300);

  // Set Window Icon if specified in INI file

  if (!globalSettings->sIconFileName.empty())
  {
	  HWND hWnd = (HWND) window->GetGenericWindowId(); // retrieve vtk window Id

	  HICON hIcon = (HICON)::LoadImageA(NULL, globalSettings->sIconFileName.c_str(), IMAGE_ICON,
		  GetSystemMetrics(SM_CXSMICON), 
		  GetSystemMetrics(SM_CYSMICON),
		  LR_LOADFROMFILE);

	  ::SendMessage(hWnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
  }

  window->GetInteractor()->SetStillUpdateRate(interactorStillUpdateRate);
  window->GetInteractor()->SetDesiredUpdateRate(interactorDesiredUpdateRate);

  // Draw the reference grid on windows
  DrawGrid();
}
  
void CloudViewerWin::AddViewerLine(const PointXYZRGB &startPoint, const PointXYZRGB &endPoint, 
		double r, double g, double b,
		const std::string &inLineName) 
{
	if (!viewer.get()) return;
	viewer->addLine<PointXYZRGB, PointXYZRGB>(startPoint, endPoint, r, g, b, inLineName);
}

void CloudViewerWin::AddViewerLine(float startX, float startY, float startZ,
									  float endX, float endY, float endZ,
									  double r, double g, double b,
									  const std::string &inLineName, int lineID) 
{
	if (!viewer.get()) return;

	PointXYZRGB startPoint;
	startPoint.x = startX;
	startPoint.y = startY;
	startPoint.z = startZ;

	PointXYZRGB endPoint;
	endPoint.x = endX;
	endPoint.y = endY;
	endPoint.z = endZ;

	std::string lineName = inLineName + "_" + std::to_string((_Longlong)lineID);

	AddViewerLine(startPoint, endPoint, r, g, b, lineName);
}

void CloudViewerWin::DrawGrid()
{
	if (!viewer.get()) return;

	// In case we are called twice, avoid duplication of shapes
	viewer->removeAllShapes();

	// Add our shapes
	double originX = 0.0;
	double originY = 0.0;
	double originZ = 0.0;
	sourceProjector->GetPositionUp(originY);
	sourceProjector->GetPositionForward(originZ);

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

		AddViewerLine(0, 0, 0, minX, -minY, displayZ, r, g, b, "Receiver1", channelIndex);

		AddViewerLine(0, 0, 0, minX, -maxY, displayZ, r, g, b, "Receiver2", channelIndex);

		AddViewerLine(0, 0, 0, maxX, -minY, displayZ, r, g, b, "Receiver3", channelIndex);
	
		AddViewerLine(0, 0, 0, maxX, -maxY, displayZ, r, g, b, "Receiver4", channelIndex);

		AddViewerLine(minX, -minY, displayZ, maxX, -minY, displayZ, r, g, b, "Receiver5", channelIndex);

		AddViewerLine(minX, -maxY, displayZ, maxX, -maxY, displayZ, r, g, b, "Receiver6", channelIndex);

		AddViewerLine(minX, -minY, displayZ, minX, -maxY, displayZ, r, g, b, "Receiver7", channelIndex);

		AddViewerLine(maxX, -minY, displayZ, maxX, -maxY, displayZ, r, g, b, "Receiver8", channelIndex);
	}


		// draw a block grid to the ground 
	    // The ground is at negative offset from the sensor's position.
		// The sensor is always considered at 0, 0, 0.
	    // This is a cheat, but it will work for now.


	for (double x = (-10.0-originX); x <= (10.0-originX); x += 2.0)  
	{
		AddViewerLine(x, -originY, -originZ, x, -originY, rangeMax-originZ, 0, 0, 0, "GroundLineX", (int) x);
	}


	for (double z= 0.0-originZ; z <= rangeMax-originZ; z += 2.0)  
	{
		double shade =  z*3.0/255.0;
		AddViewerLine(-10.0-originX, -originY, z, 10-originX, -originY , z, shade, shade, shade, "GroundLineZ", (int) z);
	}
}

// Static method
void CloudViewerWin::SetCameraView(boost::shared_ptr<pcl::visualization::PCLVisualizer> inViewer, CloudViewerWin::CameraView inAngle)
{
	double topView[11] = {55, 115, 0, -8, 25, -1.35, 82, 25, 1.0, 0, 0};
	double sideView[11] = {42, 125, -8, 1, 25, -80, -2 , 20, 0, 1, 0};
	double isoView[11] = {25,160, 2.25,-0.2, 26, -52, 56, -9, 0.6, 0.75, 0.25};	
	double frontView[11] = {-2, 200, 0,0, 50, 0, 10, -30, 0, 1, 0};
	double zoomView[11] = {-2, 120, 0, -8, 5, -1.35, 24, 5, 1.0, 0, 0};

	double *viewData = topView;

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

	if(inViewer.get())
	{
	pcl::visualization::Camera *camera = &inViewer->camera_;
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
}

void CloudViewerWin::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	// Make sure that the viewer actually exists befor doing anything.
		if (!viewer.get()) return;

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

void CloudViewerWin::mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
}

void CloudViewerWin::CreateReceiverProjector(VideoCapture::Ptr inVideoCapture, ReceiverCapture::Ptr inReceiverCapture)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();


	// Create a common point-cloud object that will be "projected" upon
	baseCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Create the ReceiverProjector.
	// The projector feeds from the videoCapture and feeds from the base cloud
	sourceProjector = ReceiverProjector::Ptr(new ReceiverProjector(videoCapture, baseCloud, receiverCapture));
	currentCloudSubscriberID = sourceProjector->Subscribe();

	// Add the channels to the point-cloud projector. 
	ReceiverChannel::Ptr channelPtr;
	int receiverID = receiverCapture->GetReceiverID();
	ReceiverSettings * receiverSettingsPtr = &globalSettings->receiverSettings[receiverID];
	for (int channelID = 0; channelID < receiverSettingsPtr->channelsConfig.size(); channelID++)
	{
			RelativePosition channelPosition = AWLCoordinates::GetChannelPosition(receiverID, channelID);
			ReceiverChannel::Ptr receiverChannel(new ReceiverChannel(receiverID, channelID,
				DEG2RAD(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].fovWidth),
				DEG2RAD(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].fovHeight),
				channelPosition.orientation.yaw,
				channelPosition.orientation.pitch,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].maxRange,
				false,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].displayColorRed / 255.0,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].displayColorGreen / 255.0,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].displayColorBlue / 255.0));

			channelPtr = sourceProjector->AddChannel(receiverChannel);
	}

	// Update all of our parameters that are related to the projector
	SetDecimation(decimationX);
	SetDisplayUnderZero(bDisplayUnderZero);
	SetPositionUp(up, false);
	SetPositionForward(forward, false);
	SetRangeMax(rangeMax, false);
	DrawGrid();
}

void CloudViewerWin::SetVideoCapture( VideoCapture::Ptr inVideoCapture)
{
	videoCapture = inVideoCapture;
}

void CloudViewerWin::SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture)
{
	receiverCapture = inReceiverCapture;
}
