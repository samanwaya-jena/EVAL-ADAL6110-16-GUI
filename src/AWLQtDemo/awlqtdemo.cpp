

#include <QTableWidget>
#include <QDesktopWidget>
#include <QApplication>
#include <QTime>
#include <QMessageBox>
#include <QListWidget>

#include <string>

#include "awlqtdemo.h"
#include "Tracker.h"
#include "ReceiverCapture.h"
#include "ReceiverCANCapture.h"
#include "FusedCloudViewer.h"
#include "DebugPrintf.h"
#include "AWLSettings.h"
#include "DetectionStruct.h"
#include "tableview.h"

#include "..\awlqtscope\awlqtscope.h"


using namespace std;
using namespace awl;

// Text update rate, in frame per seconds
#if 1
#define LOOP_RATE	20	
#else
#define LOOP_RATE	20
#endif

TransformationNode::Ptr myBaseNode;

AWLQtDemo::AWLQtDemo(int argc, char *argv[])
	: QMainWindow()
{
	ui.setupUi(this);

	// Read the settigs from the configuration file
	AWLSettings *globalSettings = AWLSettings::InitSettings();
	globalSettings->ReadSettings();

	// Build a reference coodinate system from the settings
	AWLCoordinates *globalCoordinates = AWLCoordinates::InitCoordinates();
	globalCoordinates->BuildCoordinatesFromSettings();

#if 1

	// Test the coordinates system
	TransformationNode::List receiverCoords = AWLCoordinates::GetReceivers();
	SphericalCoord sphericalPointInChannel(10, M_PI_2, 0);
	CartesianCoord cartesianPointInWorld0 = receiverCoords[0]->children[0]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	CartesianCoord cartesianPointInWorld1 = receiverCoords[0]->children[1]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	CartesianCoord cartesianPointInWorld2 = receiverCoords[0]->children[2]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	CartesianCoord cartesianPointInWorld3 = receiverCoords[0]->children[3]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	CartesianCoord cartesianPointInWorld4 = receiverCoords[0]->children[4]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	CartesianCoord cartesianPointInWorld5 = receiverCoords[0]->children[5]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	CartesianCoord cartesianPointInWorld6 = receiverCoords[0]->children[6]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);

	TransformationNode::List cameraCoords = AWLCoordinates::GetCameras();
	SphericalCoord worldPoint = cameraCoords[0]->FromReferenceCoord(eCameraToWorldCoord, cartesianPointInWorld0);
#endif


	// Fill the parameters  tables from the settings
	FillFPGAList(globalSettings);
	FillADCList(globalSettings);
	FillGPIOList(globalSettings);

	// Change the window icon if there is an override in the INI file
	if (!globalSettings->sIconFileName.empty())
	{
		setWindowIcon(QIcon(globalSettings->sIconFileName.c_str()));
	}

	// Position the main widget on the top left corner
	QRect scr = QApplication::desktop()->availableGeometry(QApplication::desktop()->primaryScreen());
	show();
	move(scr.left(),  scr.top()); 


	// In demo mode, put demo mode in window title
	if (globalSettings->bEnableDemo)
	{
		this->setWindowTitle(this->windowTitle()+" [DEMO Mode]");
	}


	// Adjust the default displayed ranges depending on the sensor capabilities
	AdjustDefaultDisplayedRanges();


	// Create the receiver communication objects
	int receiverQty = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Create the LIDAR acquisition thread object
		if (boost::iequals(globalSettings->receiverSettings[receiverID].sReceiverType, "EasySyncCAN"))
		{
			// EasySync CAN Capture is used if defined in the ini file, and by default
			receiverCaptures.push_back(ReceiverCapture::Ptr((ReceiverCapture *) new ReceiverCANCapture(receiverID, receiverID, globalSettings->receiverSettings[receiverID].channelsConfig.size())));
		}

		receiverCaptureSubscriberIDs.push_back(receiverCaptures[receiverID]->currentReceiverCaptureSubscriptions->Subscribe());
	}

	// Create the video capture objects
	int videoCaptureQty = globalSettings->cameraSettings.size();
	for (int cameraID = 0; cameraID < videoCaptureQty; cameraID++)
	{
		videoCaptures.push_back(VideoCapture::Ptr(new VideoCapture(cameraID, argc, argv)));
	}

#if 1
	// Create a common point-cloud object that will be "projected" upon
	baseCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Create the ReceiverProjector.
	// The projector feeds from the videoCapture and feeds from the base cloud
	receiver = ReceiverProjector::Ptr(new ReceiverProjector(videoCaptures[0], baseCloud, receiverCaptures[0]));

	// Add the channels to the point-cloud projector. 
	ReceiverChannel::Ptr channelPtr;

	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		for (int channelID = 0; channelID < globalSettings->receiverSettings[receiverID].channelsConfig.size(); channelID++)
		{
			ReceiverChannel::Ptr receiverChannel(new ReceiverChannel(receiverID, channelID,
				DEG2RAD(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].fovWidth),
				DEG2RAD(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].fovHeight),
				DEG2RAD(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].centerX),
				DEG2RAD(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].centerY),
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].maxRange,
				false,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].displayColorRed / 255.0,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].displayColorGreen / 255.0,
				globalSettings->receiverSettings[receiverID].channelsConfig[channelID].displayColorBlue / 255.0));

			channelPtr = receiver->AddChannel(receiverChannel);
		}
	}

	
	//  Create the fused viewer, that will instantiate all the point-cloud views.
	// All point cloud updates feed from the receiver's point-cloud data.
	// The fused Viewer also uses the receiver configuration info to build the background decorations  
	// used in point-cloud
	fusedCloudViewer = FusedCloudViewer::Ptr(new FusedCloudViewer(this->windowTitle().toStdString() + " 3D View", receiver));

#endif

	// Create the video viewer to display the camera image
	// The video viewer feeds from the  videoCapture (for image) and from the receiver (for distance info)
	int videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		QString cameraName(this->windowTitle()+" Camera");
		cameraName.append(QString().sprintf(" %02d", videoViewerID));
		
		videoViewers.push_back(VideoViewer::Ptr(new VideoViewer(cameraName.toStdString(), videoCaptures[videoViewerID])));
	}

	PrepareParametersView();
	PrepareGlobalParametersView();

	// Initialize the controls from the settings in INI file
	ui.sensorHeightSpinBox->setValue(globalSettings->receiverSettings[0].sensorUp);
	ui.sensorDepthSpinBox->setValue(globalSettings->receiverSettings[0].sensorForward);
	ui.measurementOffsetSpinBox->setValue(globalSettings->receiverSettings[0].rangeOffset);
	ui.sensorRangeMinSpinBox->setValue(globalSettings->receiverSettings[0].displayedRangeMin);

	ui.sensorRangeMax0SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[0].maxRange);
	ui.sensorRangeMax1SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[1].maxRange);
	ui.sensorRangeMax2SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[2].maxRange);
	ui.sensorRangeMax3SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[3].maxRange);
	ui.sensorRangeMax4SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[4].maxRange);
	ui.sensorRangeMax5SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[5].maxRange);
	ui.sensorRangeMax6SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[6].maxRange);

	ui.targetHintDistanceSpinBox->setValue(globalSettings->targetHintDistance);
	ui.targetHintAngleSpinBox->setValue(globalSettings->targetHintAngle);

	ui.pixelSizeSpinBox->setValue(globalSettings->pixelSize);	
	ui.decimationSpinBox->setValue(globalSettings->decimation);

	// Default values
	ChannelMask channelMask;

	if (receiverCaptures[0]) 
	{
		ui.recordChannel1CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel0);
		ui.recordChannel2CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel1);
		ui.recordChannel3CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel2);
		ui.recordChannel4CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel3);
		ui.recordChannel5CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel4);
		ui.recordChannel6CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel5);
		ui.recordChannel7CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel6);

		ui.calibrationChannel1CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel0);
		ui.calibrationChannel2CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel1);
		ui.calibrationChannel3CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel2);
		ui.calibrationChannel4CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel3);
		ui.calibrationChannel5CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel4);
		ui.calibrationChannel6CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel5);
		ui.calibrationChannel7CheckBox->setChecked(receiverCaptures[0]->receiverStatus.channelMask.bitFieldData.channel6);

		ui.frameRateSpinBox->setValue(receiverCaptures[0]->receiverStatus.frameRate);
	}
	else
	{
		ui.recordChannel1CheckBox->setChecked(true);
		ui.recordChannel2CheckBox->setChecked(true);
		ui.recordChannel3CheckBox->setChecked(true);
		ui.recordChannel4CheckBox->setChecked(true);
		ui.recordChannel5CheckBox->setChecked(true);
		ui.recordChannel6CheckBox->setChecked(true);
		ui.recordChannel7CheckBox->setChecked(true);


		ui.calibrationChannel1CheckBox->setChecked(true);
		ui.calibrationChannel2CheckBox->setChecked(true);
		ui.calibrationChannel3CheckBox->setChecked(true);
		ui.calibrationChannel4CheckBox->setChecked(true);
		ui.calibrationChannel5CheckBox->setChecked(true);
		ui.calibrationChannel6CheckBox->setChecked(true);
		ui.calibrationChannel7CheckBox->setChecked(true);
		
		ui.frameRateSpinBox->setValue(globalSettings->receiverSettings[0].receiverFrameRate);
	}

	CloudViewerWin::ColorHandlerType defaultColorType = (CloudViewerWin::ColorHandlerType) globalSettings->colorStyle;
	switch (defaultColorType) 
	{
	case CloudViewerWin::eHandlerRGB:
		ui.colorImageRadioButton->setChecked(true);
		break;

	case CloudViewerWin::eHandlerZ:
		ui.rangeImageRadioButton->setChecked(true);
		break;
	}

	// Initialize from other operating variables.
	if (receiverCaptures[0]) 
	{
		ui.injectSimulatedCheckbox->setChecked(receiverCaptures[0]->IsSimulatedDataEnabled());
	}

	ui.distanceLogFileCheckbox->setChecked(globalSettings->bWriteLogFile);

	 scopeWindow = new AWLQtScope();
	 //scopeWindow->show();

	// Initialize the 2D view
	m2DScan = new FOV_2DScan();
	m2DScan->setWindowTitle(this->windowTitle() + " 2D View");

	// Place the 2D view in the screen

	int frameWindowWidth = 712;
	m2DScan->move(scr.right()-frameWindowWidth, scr.top());
	m2DScan->show();
	QRect frame = m2DScan->frameGeometry();
	QRect client = m2DScan->geometry();
	int verticalDecorationsHeight = frame.height() - client.height();
	int horizontalDecorationsWidth = frame.width() - client.width();
	m2DScan->resize(frameWindowWidth-horizontalDecorationsWidth, scr.height() - verticalDecorationsHeight);

	mCfgSensor.shortRangeDistance = globalSettings->shortRangeDistance;
    mCfgSensor.shortRangeDistanceStartLimited = globalSettings->shortRangeDistanceStartLimited;
    mCfgSensor.shortRangeAngle = globalSettings->shortRangeAngle;
    mCfgSensor.shortRangeAngleStartLimited = globalSettings->shortRangeAngleStartLimited;

    mCfgSensor.longRangeDistance = globalSettings->longRangeDistance;
    mCfgSensor.longRangeDistanceStartLimited = globalSettings->longRangeDistanceStartLimited;
    mCfgSensor.longRangeAngle = globalSettings->longRangeAngle;
    mCfgSensor.longRangeAngleStartLimited = globalSettings->longRangeAngleStartLimited;

	mCfgSensor.spareDepth = -globalSettings->receiverSettings[0].sensorForward;

	m2DScan->slotConfigChanged(mCfgSensor);

	// Initialize the table view
	mTableView = new TableView();
	mTableView->setWindowTitle(this->windowTitle() + " Table View");

	// Place the table view in the top left of screen

	mTableView->move(scr.left(), scr.top());
	mTableView->show();
	frame = mTableView->frameGeometry();
	client = mTableView->geometry();
	verticalDecorationsHeight = frame.height() - client.height();
	horizontalDecorationsWidth = frame.width() - client.width();
	mTableView->resize(client.width(), scr.height() - verticalDecorationsHeight);

	mTableView->slotConfigChanged();

	// Calibration 

	// Menu items signals and slots
	connect(ui.action2D, SIGNAL(toggled(bool )), this, SLOT(on_view2DActionToggled()));
	connect(ui.actionTableView, SIGNAL(toggled(bool )), this, SLOT(on_viewTableViewActionToggled()));
	connect(ui.actionCamera, SIGNAL(toggled(bool )), this, SLOT(on_viewCameraActionToggled()));
	connect(ui.actionGraph, SIGNAL(toggled(bool )), this, SLOT(on_viewGraphActionToggled()));
	connect(ui.action3D_View, SIGNAL(toggled(bool )), this, SLOT(on_view3DActionToggled()));

	connect(ui.actionQuitter, SIGNAL(triggered(bool )), qApp, SLOT(closeAllWindows()));
	// View signals and slots on close
	connect(m2DScan, SIGNAL(closed()), this, SLOT(on_view2DClose()));
	connect(mTableView, SIGNAL(closed()), this, SLOT(on_viewTableViewClose()));
	connect(scopeWindow, SIGNAL(closed( )), this, SLOT(on_viewGraphClose()));


	// Start the threads for background  receiver capture objects
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++) 
	{ 
		receiverCaptures[receiverID]->Go(true);
	}


	// Start the threads for background video capture objects
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{ 
		videoCaptures[cameraID]->Go();
	}


	if (receiver) receiver->Go();

	// Create a timer to keep the UI objects spinning
     myTimer = new QTimer(this);
     connect(myTimer, SIGNAL(timeout()), this, SLOT(on_timerTimeout()));
	 myTimer->start(LOOP_RATE);

	// Initial Update the various status indicators on the display
	DisplayReceiverStatus();

	// Start the threads and display the windows if they are defined as startup in the ini file
	if (globalSettings->bDisplay2DWindow) 
	{
		ui.action2D->toggle();
	}

	if (globalSettings->bDisplayTableViewWindow) 
	{
		ui.actionTableView->toggle();
	}

	if (globalSettings->bDisplay3DWindow) 
	{
		ui.action3D_View->toggle();
	}
	
	if (globalSettings->bDisplayScopeWindow)
	{
		ui.actionGraph->toggle();
	}

	if (globalSettings->bDisplayCameraWindow)
	{
		ui.actionCamera->toggle();
		// Position the video viewer.
		// This has to be done agfter the Go(), to make sure the window is created
//		videoViewer->move(scr.left(), scr.top()); 
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
		    videoViewers[viewerID]->move(scr.left()+(viewerID*10), scr.top()+95+(viewerID*10));
		}
	}

	switch (globalSettings->defaultParametersAlgos.defaultAlgo)
	{
	case 1: 
		{
		ui.algo1RadioButton->setChecked(true);
		}
		break;

	case 2: 
		{
		ui.algo2RadioButton->setChecked(true);
		}
		break;

	case 3: 
		{
		ui.algo3RadioButton->setChecked(true);
		}
		break;

	case 4: 
		{
		ui.algo4RadioButton->setChecked(true);
		}
		break;

	default: 
		{
		ui.algo2RadioButton->setChecked(true);
		}
		break;

	}


	// Calibration 
	ui.calibrationBetaDoubleSpinBox->setValue(1.0);

	// In demo mode, automatically force the injection of data on receiver.
	// put demo mode in window titles
	if (globalSettings->bEnableDemo)
	{
		ui.injectSimulatedCheckbox->setChecked(true);
		m2DScan->setWindowTitle(this->windowTitle() + " 2D View");
		mTableView->setWindowTitle(this->windowTitle() + " Table View");
	}
}

AWLQtDemo::~AWLQtDemo()
{
}

void AWLQtDemo::AdjustDefaultDisplayedRanges()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	// Adjust the default maximum displayed range for the receiver (used in various interfaces) 
	// to reflect the maximum of all its channel ranges.
	int receiverQty = globalSettings->receiverSettings.size();
	long absoluteMaxRange = 0.0;
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		long absoluteMaxRangeForReceiver = 0.0;
		int channelQty = globalSettings->receiverSettings[receiverID].channelsConfig.size();
		for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
		{
			if (globalSettings->receiverSettings[receiverID].channelsConfig[channelIndex].maxRange > absoluteMaxRangeForReceiver)
				absoluteMaxRangeForReceiver = globalSettings->receiverSettings[receiverID].channelsConfig[channelIndex].maxRange;
		}
		AWLSettings::GetGlobalSettings()->receiverSettings[0].displayedRangeMax = absoluteMaxRangeForReceiver;

		if (absoluteMaxRangeForReceiver > absoluteMaxRange) absoluteMaxRange = absoluteMaxRangeForReceiver; 
	}

	// And the default maximum displayed range for interfaces to the max range of all receivers 
	AWLSettings::GetGlobalSettings()->longRangeDistance = absoluteMaxRange;
}


void AWLQtDemo::on_destroy()
{
	if (fusedCloudViewer) fusedCloudViewer->Stop();
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}

	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	if (receiver) receiver->Stop();

	for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
	{
		if (videoViewers[viewerID]) videoViewers[viewerID]->Stop();
	}

	if (m2DScan) delete m2DScan;
	if (mTableView) delete mTableView;	
	if (scopeWindow) delete scopeWindow;
}


void AWLQtDemo::on_colorImageRadioButton_setChecked(bool bChecked)
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCurrentColorHandlerType(CloudViewerWin::eHandlerRGB);
		}
	}
}


void AWLQtDemo::on_rangeImageRadioButton_setChecked(bool bChecked)
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCurrentColorHandlerType(CloudViewerWin::eHandlerZ);
		}
	}
}


void AWLQtDemo::on_simulatedDataInjectCheckBox_setChecked(bool  bChecked)
{
	if (receiverCaptures[0])
	{
		receiverCaptures[0]->EnableSimulationData(bChecked);
	}

	if (receiverCaptures.size() >= 2 && receiverCaptures[1])
	{
		receiverCaptures[1]->EnableSimulationData(bChecked);
	}

}


void AWLQtDemo::on_viewSidePushButton_pressed()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCameraView(CloudViewerWin::eCameraSide);
		}
	}
}


void AWLQtDemo::on_viewTopPushButton_pressed()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCameraView(CloudViewerWin::eCameraTop);
		}
	}
}


void AWLQtDemo::on_viewZoomPushButton_pressed()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCameraView(CloudViewerWin::eCameraZoom);
		}
	}
}


void AWLQtDemo::on_viewFrontPushButton_pressed()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCameraView(CloudViewerWin::eCameraFront);
		}
	}
}


void AWLQtDemo::on_viewIsoPushButton_pressed()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		fusedCloudViewer->viewers[0]->SetCameraView(CloudViewerWin::eCameraIsometric);
		}
	}
}


void AWLQtDemo::on_recordPushButton_clicked()

{
	std::string sRecordFileName(ui.recordFileNameEdit->text().toStdString());
	uint8_t frameRate = ui.frameRateSpinBox->value();
	ChannelMask channelMask;

	channelMask.bitFieldData.channel0 = ui.recordChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel1 = ui.recordChannel2CheckBox->isChecked();
	channelMask.bitFieldData.channel2 = ui.recordChannel3CheckBox->isChecked();
	channelMask.bitFieldData.channel3 = ui.recordChannel4CheckBox->isChecked();
	channelMask.bitFieldData.channel4 = ui.recordChannel5CheckBox->isChecked();
	channelMask.bitFieldData.channel5 = ui.recordChannel6CheckBox->isChecked();
	channelMask.bitFieldData.channel6 = ui.recordChannel7CheckBox->isChecked();
	channelMask.bitFieldData.unused = 0;

	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetRecordFileName(sRecordFileName);
		receiverCaptures[0]->StartRecord(frameRate, channelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}


void AWLQtDemo::on_playbackPushButton_clicked()

{
	std::string sPlaybackFileName(ui.playbackFileNameEdit->text().toStdString());
	uint8_t frameRate = ui.frameRateSpinBox->value();
	ChannelMask channelMask;

	channelMask.bitFieldData.channel0 = ui.recordChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel1 = ui.recordChannel2CheckBox->isChecked();
	channelMask.bitFieldData.channel2 = ui.recordChannel3CheckBox->isChecked();
	channelMask.bitFieldData.channel3 = ui.recordChannel4CheckBox->isChecked();
	channelMask.bitFieldData.channel4 = ui.recordChannel5CheckBox->isChecked();
	channelMask.bitFieldData.channel5 = ui.recordChannel6CheckBox->isChecked();
	channelMask.bitFieldData.channel6 = ui.recordChannel7CheckBox->isChecked();
	channelMask.bitFieldData.unused = 0;
	
	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetPlaybackFileName(sPlaybackFileName);
		receiverCaptures[0]->StartPlayback(frameRate, channelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}


void AWLQtDemo::on_stopPushButton_clicked()

{
	std::string sPlaybackFileName(ui.playbackFileNameEdit->text().toStdString());

	if (receiverCaptures[0]) 
	{
		if (receiverCaptures[0]->receiverStatus.bInPlayback) 
		{
			receiverCaptures[0]->StopPlayback();
		}
		else if (receiverCaptures[0]->receiverStatus.bInRecord) 
		{
			receiverCaptures[0]->StopRecord();
		}
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}


void AWLQtDemo::on_decimationSpin_editingFinished()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
			int decimation = ui.decimationSpinBox->value();
			fusedCloudViewer->viewers[0]->SetDecimation(decimation);
		}
	}
}


void AWLQtDemo::on_pixelSizeSpin_editingFinished()
{
	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1) 
		{
		int pixelSize =ui.pixelSizeSpinBox->value();
		fusedCloudViewer->viewers[0]->SetPixelSize(pixelSize);
		}
	}
}


void AWLQtDemo::on_sensorHeightSpin_editingFinished()
{
	double height = ui.sensorHeightSpinBox->value();
	if (abs(height-AWLSettings::GetGlobalSettings()->receiverSettings[0].sensorUp) < 0.001) return;
	AWLSettings::GetGlobalSettings()->receiverSettings[0].sensorUp = height;

	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();

	// Process
	if (receiver) 
	{	
		receiver->SetViewerHeight(height);
	}

	if (receiverCaptures[0]) 
	{
	//	receiverCaptures[0]->SetSensorHeight(height);
	}


	if (fusedCloudViewer) 
	{
		fusedCloudViewer->SetViewerHeight(height);
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
}


void AWLQtDemo::on_sensorDepthSpin_editingFinished()
{
	double forward = ui.sensorDepthSpinBox->value();
	if (abs(forward-AWLSettings::GetGlobalSettings()->receiverSettings[0].sensorForward) < 0.001) return;
	AWLSettings::GetGlobalSettings()->receiverSettings[0].sensorForward = forward;

	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();

	// Process

	if (receiver) 
	{	
		receiver->SetViewerDepth(forward);
	}

	if (fusedCloudViewer) 
	{
		fusedCloudViewer->SetViewerDepth(forward);
	}

	if (m2DScan && !m2DScan->isHidden())
	{
		mCfgSensor.spareDepth = -forward;
		m2DScan->slotConfigChanged(mCfgSensor);
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
}

void AWLQtDemo::on_calibrationRangeMinSpin_editingFinished()
{
	double range = ui.sensorRangeMinSpinBox->value();
	AWLSettings::GetGlobalSettings()->receiverSettings[0].displayedRangeMin = range;
#if 1
	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetMinDistance(range);
	}
#endif

}

void AWLQtDemo::ChangeRangeMax(int channelID, double range)
{
	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();


	// Update the settings
	AWLSettings *settings = AWLSettings::GetGlobalSettings();
	settings->receiverSettings[0].channelsConfig[channelID].maxRange = range;

	// Calculate the absolute max distance from the settings
	int channelQty = settings->receiverSettings[0].channelsConfig.size();
	long absoluteMaxRange = 0.0;
	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		if (settings->receiverSettings[0].channelsConfig[channelIndex].maxRange > absoluteMaxRange)
			absoluteMaxRange = settings->receiverSettings[0].channelsConfig[channelIndex].maxRange;
	}

	AWLSettings::GetGlobalSettings()->receiverSettings[0].displayedRangeMax = absoluteMaxRange;
	AWLSettings::GetGlobalSettings()->longRangeDistance = absoluteMaxRange;

	// Update user interface parts
	if (receiver) 
	{
		ReceiverChannel::Ptr receiverChannel = receiver->GetChannel(channelID);
		if (receiverChannel) receiverChannel->SetRangeMax(range);
	}

	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetMaxDistance(channelID, range);
	}

	if (fusedCloudViewer) 
	{
		fusedCloudViewer->SetRangeMax(absoluteMaxRange);	
	}

	
	if (m2DScan && !m2DScan->isHidden())
	{

	    mCfgSensor.longRangeDistance = absoluteMaxRange;
		m2DScan->slotConfigChanged(mCfgSensor);
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
}

void AWLQtDemo::on_calibrationRangeMax0Spin_editingFinished()
{
	double range = ui.sensorRangeMax0SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[0].maxRange) < 0.001) return;

	ChangeRangeMax(0, range);
}

void AWLQtDemo::on_calibrationRangeMax1Spin_editingFinished()
{
	double range = ui.sensorRangeMax1SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[1].maxRange) < 0.001) return;

	ChangeRangeMax(1, range);
}

void AWLQtDemo::on_calibrationRangeMax2Spin_editingFinished()
{
	double range = ui.sensorRangeMax2SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[2].maxRange) < 0.001) return;

	ChangeRangeMax(2, range);
}

void AWLQtDemo::on_calibrationRangeMax3Spin_editingFinished()
{
	double range = ui.sensorRangeMax3SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[3].maxRange) < 0.001) return;

	ChangeRangeMax(3, range);
}

void AWLQtDemo::on_calibrationRangeMax4Spin_editingFinished()
{
	double range = ui.sensorRangeMax4SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[4].maxRange) < 0.001) return;

	ChangeRangeMax(4, range);
}

void AWLQtDemo::on_calibrationRangeMax5Spin_editingFinished()
{
	double range = ui.sensorRangeMax5SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[5].maxRange) < 0.001) return;

	ChangeRangeMax(5, range);
}

void AWLQtDemo::on_calibrationRangeMax6Spin_editingFinished()
{
	double range = ui.sensorRangeMax6SpinBox->value();
	if (abs(range-AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig[6].maxRange) < 0.001) return;

	ChangeRangeMax(6, range);
}


void AWLQtDemo::on_measurementOffsetSpin_editingFinished()
{
	double offset = ui.measurementOffsetSpinBox->value();

	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetMeasurementOffset(offset);
	}

	AWLSettings::GetGlobalSettings()->receiverSettings[0].rangeOffset = offset;

}


void AWLQtDemo::on_calibratePushButton_clicked()

{
	uint8_t frameQty = ui.calibrationFrameQtySpinBox->value();
	float   beta = ui.calibrationBetaDoubleSpinBox->value();
	ChannelMask channelMask;

	channelMask.bitFieldData.channel0 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel1 = ui.calibrationChannel2CheckBox->isChecked();
	channelMask.bitFieldData.channel2 = ui.calibrationChannel3CheckBox->isChecked();
	channelMask.bitFieldData.channel3 = ui.calibrationChannel4CheckBox->isChecked();
	channelMask.bitFieldData.channel4 = ui.calibrationChannel5CheckBox->isChecked();
	channelMask.bitFieldData.channel5 = ui.calibrationChannel6CheckBox->isChecked();
	channelMask.bitFieldData.channel6 = ui.calibrationChannel7CheckBox->isChecked();
	channelMask.bitFieldData.unused = 0;

	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->StartCalibration(frameQty, beta, channelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}

void AWLQtDemo::on_targetHintDistanceSpin_editingFinished()
{
	double distance = ui.targetHintDistanceSpinBox->value();

	AWLSettings::GetGlobalSettings()->targetHintDistance = distance;
}

void AWLQtDemo::on_targetHintAngleSpin_editingFinished()
{
	double angle = ui.targetHintAngleSpinBox->value();

	AWLSettings::GetGlobalSettings()->targetHintAngle = angle;
}


void AWLQtDemo::on_distanceLogCheckBox_setChecked(bool  bChecked)
{

	if (bChecked) 
	{
		AWLSettings::GetGlobalSettings()->bWriteLogFile = bChecked;
		if (receiverCaptures[0]) receiverCaptures[0]->BeginDistanceLog();
	}
	else 
	{
		if (receiverCaptures[0]) receiverCaptures[0]->EndDistanceLog();
		AWLSettings::GetGlobalSettings()->bWriteLogFile = bChecked;
	}
}

void AWLQtDemo::on_timerTimeout()
{
	myTimer->stop();

	bool bContinue = true;
			
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++)
	{
		if (videoCaptures[cameraID]->WasStopped()) 
		{
			bContinue = false;
			break;
		}
	}

	if (receiver && receiver->WasStopped())
	{
		bContinue = false;
	}

	if (bContinue)
	{
		for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
		{
			if (!receiverCaptures[receiverID]) 
			{
				bContinue = false;
				break;
			}
			// Use the frame snapped by the  as the current frame
			// all displays will reference to.
			uint32_t lastDisplayedFrame = receiverCaptures[receiverID]->SnapSnapshotFrameID();

			// Update the status information
			if (receiverCaptures[receiverID]->receiverStatus.bUpdated) 
			{
				DisplayReceiverStatus(receiverID);
			}
		}// For
	}

	// Uopdate the 3D display

	if (bContinue && receiver) 
	{
		receiver->DoThreadIteration();

	if (receiver->WasStopped()) bContinue = false;
	}


	if (bContinue) 
	{
		DetectionDataVect detectionData;
		GetLatestDetections(detectionData);

		// Update the 2D view
		if (m2DScan) m2DScan->slotDetectionDataChanged(detectionData);

		// Update the camera views
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) videoViewers[viewerID]->slotDetectionDataChanged(detectionData);
		}

		// Update the table views
		if (mTableView) mTableView->slotDetectionDataChanged(detectionData);
	}


	if (bContinue && fusedCloudViewer && !fusedCloudViewer->WasStopped()) 
	{
		if (fusedCloudViewer->viewers.size()>=1) 
		{
			fusedCloudViewer->viewers[0]->DoThreadIteration();
		}
	}

	// Update the menus for the 3D view and camera view, since we do not get any notifiocation from them
	if (ui.action3D_View->isChecked() && (!fusedCloudViewer || fusedCloudViewer->WasStopped())) 
	{
		ui.action3D_View->toggle();
	}


	if (bContinue)
	{
		myTimer->start(LOOP_RATE);
	}
	else 
	{
		this->close();
	}
}

void AWLQtDemo::GetLatestDetections(DetectionDataVect &detectionData)
{
	AWLSettings *settings = AWLSettings::GetGlobalSettings();

	// Build the list of detections that need to be updated
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];
		// Use the frame snapped by the main display timer as the current frame
		uint32_t lastDisplayedFrame = receiver->GetSnapshotFrameID();

		int channelQty = receiver->GetChannelQty();
		for (int channelID = 0; channelID < channelQty; channelID++) 
		{
			if (receiverCaptures[receiverID]->GetFrameQty()) 
			{

				ChannelFrame::Ptr channelFrame(new ChannelFrame(receiverID, channelID));

				// Thread safe
				// The UI thread "Snaps" the frame ID for all other interface objects to display
				if (receiverCaptures[receiverID]->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, receiverCaptureSubscriberIDs[receiverID])) 
				{

					int detectionQty = channelFrame->detections.size();
					int detectionIndex = 0;
					for (int i = 0; i < detectionQty; i++)
					{
						Detection::Ptr detection = channelFrame->detections.at(i);
						if ((detection->distance >= receiverCaptures[receiverID]->GetMinDistance()) && 
							(detection->distance <= receiverCaptures[receiverID]->GetMaxDistance(channelID))) 
						{
							Detection::Ptr storedDetection = detection;
							detectionData.push_back(storedDetection);
						}
					}
				}

			}
		}
	}
}


void AWLQtDemo::DisplayReceiverStatus()
{

	int receiverQty = receiverCaptures.size();
	 for (int receiverID = 0; receiverID < receiverQty; receiverID++) 
	 {
		DisplayReceiverStatus(receiverID);
	 }

}

void AWLQtDemo::DisplayReceiverStatus(int receiverID)

{
	bool bEnableButtons = true;

	if (receiverID != 0) return;

	if (receiverCaptures[receiverID]) 
	{
		bEnableButtons = true;

		boost::mutex::scoped_lock rawLock(receiverCaptures[receiverID]->currentReceiverCaptureSubscriptions->GetMutex());


		receiverCaptures[receiverID]->receiverStatus.bUpdated = false;
		ReceiverStatus status = receiverCaptures[receiverID]->receiverStatus;
		rawLock.unlock();

		QString formattedString;
		formattedString.sprintf("%d.%d", status.version.major, status.version.minor);
		ui.versionEdit->setText(formattedString);

		formattedString.sprintf("%.1f", status.temperature);
		ui.temperatureEdit->setText(formattedString);

		formattedString.sprintf("%.1f", status.voltage);
		ui.voltageEdit->setText(formattedString);

		ui.bootMainChecksumCheckBox->setChecked(status.bootChecksumError.bitFieldData.mainChecksum);
		ui.bootAuxChecksumCheckBox->setChecked(status.bootChecksumError.bitFieldData.auxChecksum);

		ui.bootEmitter1CheckBox->setChecked(status.bootSelfTest.bitFieldData.emitter0);
		ui.bootEmitter2CheckBox->setChecked(status.bootSelfTest.bitFieldData.emitter1);
		ui.bootReceiverCheckBox->setChecked(status.bootSelfTest.bitFieldData.receiver);
		ui.bootDSPCheckBox->setChecked(status.bootSelfTest.bitFieldData.dsp);
		ui.bootMemoryCheckBox->setChecked(status.bootSelfTest.bitFieldData.memory);
		ui.bootChecksumCheckBox->setChecked(status.bootSelfTest.bitFieldData.checksum);

		ui.hardwareEmitter1CheckBox->setChecked(status.hardwareError.bitFieldData.emitter0);
		ui.hardwareEmitter2CheckBox->setChecked(status.hardwareError.bitFieldData.emitter1);
		ui.hardwareReceiverCheckBox->setChecked(status.hardwareError.bitFieldData.receiver);
		ui.hardwareDSPCheckBox->setChecked(status.hardwareError.bitFieldData.dsp);
		ui.hardwareMemoryCheckBox->setChecked(status.hardwareError.bitFieldData.memory);

		ui.receiverChannel1CheckBox->setChecked(status.receiverError.bitFieldData.channel0);
		ui.receiverChannel2CheckBox->setChecked(status.receiverError.bitFieldData.channel1);
		ui.receiverChannel3CheckBox->setChecked(status.receiverError.bitFieldData.channel2);
		ui.receiverChannel4CheckBox->setChecked(status.receiverError.bitFieldData.channel3);
		ui.receiverChannel5CheckBox->setChecked(status.receiverError.bitFieldData.channel4);
		ui.receiverChannel6CheckBox->setChecked(status.receiverError.bitFieldData.channel5);
		ui.receiverChannel7CheckBox->setChecked(status.receiverError.bitFieldData.channel6);

		ui.statusSelfTestCheckBox->setChecked(status.status.bitFieldData.selfTest);
		ui.statusShutdownCheckBox->setChecked(status.status.bitFieldData.shutdown);
		ui.statusSensorBlockedCheckBox->setChecked(status.status.bitFieldData.sensorBlocked);
		ui.statusReducedPerformanceCheckBox->setChecked(status.status.bitFieldData.reducedPerformance);
		ui.statusSaturationCheckBox->setChecked(status.status.bitFieldData.saturation);

		// Registers
		formattedString.sprintf("%04X", status.fpgaRegisterAddressRead / 4);
		ui.registerFPGAAddressGetLineEdit->setText(formattedString);

		formattedString.sprintf("%08X", status.fpgaRegisterValueRead);
		ui.registerFPGAValueGetLineEdit->setText(formattedString);

		formattedString.sprintf("%04X", status.adcRegisterAddressRead);
		ui.registerADCAddressGetLineEdit->setText(formattedString);

		formattedString.sprintf("%08X", status.adcRegisterValueRead);
		ui.registerADCValueGetLineEdit->setText(formattedString);

		UpdateGPIOList();
		
		// Record / play / stop buttons
		if (status.bInPlayback || status.bInRecord) 
		{
			ui.recordButton->setEnabled(false);
			ui.playbackButton->setEnabled(false);
			ui.calibrateButton->setEnabled(false);
			ui.stopButton->setEnabled(true);
		}
		else 
		{
			ui.recordButton->setEnabled(true);
			ui.playbackButton->setEnabled(true);
			ui.calibrateButton->setEnabled(true);
			ui.stopButton->setEnabled(false);
		}
	}  // If receiverCaptures[receiverID]
	else  /* ! receiverCapture[receiverID] */ 
	{
		bEnableButtons = false;

		ui.recordButton->setEnabled(false);
		ui.playbackButton->setEnabled(false);
		ui.calibrateButton->setEnabled(false);
		ui.stopButton->setEnabled(false);
	}


		ui.versionEdit->setEnabled(bEnableButtons);
		ui.temperatureEdit->setEnabled(bEnableButtons);
		ui.voltageEdit->setEnabled(bEnableButtons);

		ui.bootMainChecksumCheckBox->setEnabled(bEnableButtons);
		ui.bootAuxChecksumCheckBox->setEnabled(bEnableButtons);

		ui.bootEmitter1CheckBox->setEnabled(bEnableButtons);
		ui.bootEmitter2CheckBox->setEnabled(bEnableButtons);
		ui.bootReceiverCheckBox->setEnabled(bEnableButtons);
		ui.bootDSPCheckBox->setEnabled(bEnableButtons);
		ui.bootMemoryCheckBox->setEnabled(bEnableButtons);
		ui.bootChecksumCheckBox->setEnabled(bEnableButtons);

		ui.hardwareEmitter1CheckBox->setEnabled(bEnableButtons);
		ui.hardwareEmitter2CheckBox->setEnabled(bEnableButtons);
		ui.hardwareReceiverCheckBox->setEnabled(bEnableButtons);
		ui.hardwareDSPCheckBox->setEnabled(bEnableButtons);
		ui.hardwareMemoryCheckBox->setEnabled(bEnableButtons);

		ui.receiverChannel1CheckBox->setEnabled(bEnableButtons);
		ui.receiverChannel2CheckBox->setEnabled(bEnableButtons);
		ui.receiverChannel3CheckBox->setEnabled(bEnableButtons);
		ui.receiverChannel4CheckBox->setEnabled(bEnableButtons);
		ui.receiverChannel5CheckBox->setEnabled(bEnableButtons);
		ui.receiverChannel6CheckBox->setEnabled(bEnableButtons);
		ui.receiverChannel7CheckBox->setEnabled(bEnableButtons);

		ui.statusSelfTestCheckBox->setEnabled(bEnableButtons);
		ui.statusShutdownCheckBox->setEnabled(bEnableButtons);
		ui.statusSensorBlockedCheckBox->setEnabled(bEnableButtons);
		ui.statusReducedPerformanceCheckBox->setEnabled(bEnableButtons);
		ui.statusSaturationCheckBox->setEnabled(bEnableButtons);

		ui.registerFPGASetPushButton->setEnabled(bEnableButtons);
		ui.registerFPGAGetPushButton->setEnabled(bEnableButtons);
		ui.registerADCSetPushButton->setEnabled(bEnableButtons);
		ui.registerADCGetPushButton->setEnabled(bEnableButtons);

		UpdateParametersView();
		UpdateGlobalParametersView();
}


void AWLQtDemo::on_algo1RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	int receiverCount = receiverCaptures.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		receiverCaptures[receiverID]->SetAlgorithm(1);
	}
	PrepareParametersView();
}

void AWLQtDemo::on_algo2RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	int receiverCount = receiverCaptures.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		receiverCaptures[receiverID]->SetAlgorithm(2);
	}
	PrepareParametersView();
}

void AWLQtDemo::on_algo3RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	int receiverCount = receiverCaptures.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		receiverCaptures[receiverID]->SetAlgorithm(3);
	}
	PrepareParametersView();
}


void AWLQtDemo::on_algo4RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	int receiverCount = receiverCaptures.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		receiverCaptures[receiverID]->SetAlgorithm(4);
	}
	PrepareParametersView();
}

void AWLQtDemo::PrepareParametersView()

{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = settingsPtr->defaultParametersAlgos.defaultAlgo;

	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->receiverSettings[0].parametersAlgos.defaultAlgo;
	}

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();

	// Make sure headers show up.  Sometimes Qt designer flips that attribute.

	ui.parametersTable->horizontalHeader()->setVisible(true);

	// Set the number of rows in the table depending on the number of parameters for the algo, 
	// based on info in INI file.
	ui.parametersTable->setRowCount(rowCount);

	// Set the column widths
	ui.parametersTable->horizontalHeader()->setSectionResizeMode(eParameterCheckColumn, QHeaderView::Fixed);
	ui.parametersTable->horizontalHeader()->setSectionResizeMode(eParameterDescriptionColumn, QHeaderView::Fixed);
	ui.parametersTable->horizontalHeader()->setSectionResizeMode(eParameterValueColumn, QHeaderView::Fixed);
	ui.parametersTable->horizontalHeader()->setSectionResizeMode(eParameterConfirmColumn, QHeaderView::Fixed);
		// Column 0 is ID/Checkboxmake it quite small 
	ui.parametersTable->setColumnWidth(eParameterCheckColumn, 45);
	// Column 1 is description, make it much larger than the default 
	ui.parametersTable->setColumnWidth(eParameterDescriptionColumn, 160);
	// Column 2 is value , make it slightly larger
	ui.parametersTable->setColumnWidth(eParameterValueColumn, 60);
	// Column 3 is confirmation , make it slightly larger
	ui.parametersTable->setColumnWidth(eParameterConfirmColumn, 60);
		
	// Put the contents in the table
	for (int row = 0; row < rowCount; row++) 
	{
		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *newItem = new QTableWidgetItem(algoParameters[row].sIndex.c_str());
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		newItem->setCheckState(Qt::Unchecked);
		ui.parametersTable->setItem(row, eParameterCheckColumn, newItem);

		// Column 1: Is Description .  Not editable text
		newItem = new QTableWidgetItem(algoParameters[row].sDescription.c_str());
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.parametersTable->setItem(row, eParameterDescriptionColumn, newItem);

		// Column 2: Is Value .  Editable text
		QString sValue;
		if (algoParameters[row].paramType == eAlgoParamInt)
		{
			sValue.sprintf("%d", algoParameters[row].intValue); 
		}
		else
		{
			sValue.sprintf("%f", algoParameters[row].floatValue); 
		}

		newItem = new QTableWidgetItem(sValue);
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable |  Qt::ItemIsEnabled);
		ui.parametersTable->setItem(row, eParameterValueColumn, newItem);
	
		// Column 3: Is Confirmation .  Not editable text
		QString sValueConfirm;
		if (algoParameters[row].paramType == eAlgoParamInt)
		{
			sValueConfirm.sprintf("%d", algoParameters[row].intValue); 
		}
		else
		{
			sValueConfirm.sprintf("%f", algoParameters[row].floatValue); 
		}
		newItem = new QTableWidgetItem(sValueConfirm);
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.parametersTable->setItem(row, eParameterConfirmColumn, newItem);
	}
}

void AWLQtDemo::UpdateParametersView()

{
	int currentAlgo;
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->receiverSettings[0].parametersAlgos.defaultAlgo;
	}

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *checkItem = ui.parametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *confirmItem = ui.parametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::PartiallyChecked)
		{
			// Going from partially checked to another value means we got an update
			// Update the value text.
			if (!algoParameters[row].pendingUpdates) 
			{
				// Update was received.  CheckState falls back to default.

				checkItem->setCheckState(Qt::Unchecked);
				ui.parametersTable->setItem(row, eParameterCheckColumn, checkItem);
				// Get the confirm value and format.
				QString sValueConfirm;
				if (algoParameters[row].paramType == eAlgoParamInt)
				{
					sValueConfirm.sprintf("%d", algoParameters[row].intValue); 
				}
				else
				{
				sValueConfirm.sprintf("%f", algoParameters[row].floatValue); 
				}
				confirmItem->setText(sValueConfirm);
				ui.parametersTable->setItem(row, eParameterConfirmColumn, confirmItem);
			}
		}
		else 
		{
			if (algoParameters[row].pendingUpdates) 
			{
				checkItem->setCheckState(Qt::PartiallyChecked);
				ui.parametersTable->setItem(row, eParameterCheckColumn, checkItem);
			}
		}
	}
}
void AWLQtDemo::on_algoParametersSetPushButton_clicked()
{
	int currentAlgo;
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->receiverSettings[0].parametersAlgos.defaultAlgo;
	}

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.parametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.parametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.parametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.parametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.parametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			// The value value is read.
			// Format depends on the cell type
			
			QString sValueText = valueItem->text();
			uint16_t parameterAddress = algoParameters[row].address;
			uint32_t parameterValue = 0L;
			if (algoParameters[row].paramType == eAlgoParamInt)
			{
				int intValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%d", &intValue);
				// Send to the parameter value as uint32_t
				parameterValue = (uint32_t) intValue;
			}
			else
			{
				int floatValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%f", &floatValue);
				parameterValue = * (uint32_t *) &floatValue;
			}

			receiverCaptures[0]->SetAlgoParameter(currentAlgo, parameterAddress, parameterValue); 
		} // if checked
	} // for 
}

void AWLQtDemo::on_algoParametersGetPushButton_clicked()
{
	int currentAlgo;
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->receiverSettings[0].parametersAlgos.defaultAlgo;
	}

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.parametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.parametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.parametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.parametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.parametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			uint16_t parameterAddress = algoParameters[row].address;
			receiverCaptures[0]->QueryAlgoParameter(currentAlgo,  parameterAddress); 
		} // if checked
	} // for 
}



void AWLQtDemo::PrepareGlobalParametersView()

{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = 0;

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();

	// Make sure headers show up.  Sometimes Qt designer flips that attribute.

	ui.globalParametersTable->horizontalHeader()->setVisible(true);

	// Set the number of rows in the table depending on the number of parameters for the algo, 
	// based on info in INI file.
	ui.globalParametersTable->setRowCount(rowCount);

	// Set the column widths
	ui.globalParametersTable->horizontalHeader()->setSectionResizeMode(eParameterCheckColumn, QHeaderView::Fixed);
	ui.globalParametersTable->horizontalHeader()->setSectionResizeMode(eParameterDescriptionColumn, QHeaderView::Fixed);
	ui.globalParametersTable->horizontalHeader()->setSectionResizeMode(eParameterValueColumn, QHeaderView::Fixed);
	ui.globalParametersTable->horizontalHeader()->setSectionResizeMode(eParameterConfirmColumn, QHeaderView::Fixed);
		// Column 0 is ID/Checkboxmake it quite small 
	ui.globalParametersTable->setColumnWidth(eParameterCheckColumn, 45);
	// Column 1 is description, make it much larger than the default 
	ui.globalParametersTable->setColumnWidth(eParameterDescriptionColumn, 160);
	// Column 2 is value , make it slightly larger
	ui.globalParametersTable->setColumnWidth(eParameterValueColumn, 60);
	// Column 3 is confirmation , make it slightly larger
	ui.globalParametersTable->setColumnWidth(eParameterConfirmColumn, 60);
		
	// Put the contents in the table
	for (int row = 0; row < rowCount; row++) 
	{
		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *newItem = new QTableWidgetItem(algoParameters[row].sIndex.c_str());
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		newItem->setCheckState(Qt::Unchecked);
		ui.globalParametersTable->setItem(row, eParameterCheckColumn, newItem);

		// Column 1: Is Description .  Not editable text
		newItem = new QTableWidgetItem(algoParameters[row].sDescription.c_str());
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.globalParametersTable->setItem(row, eParameterDescriptionColumn, newItem);

		// Column 2: Is Value .  Editable text
		QString sValue;
		if (algoParameters[row].paramType == eAlgoParamInt)
		{
			sValue.sprintf("%d", algoParameters[row].intValue); 
		}
		else
		{
			sValue.sprintf("%f", algoParameters[row].floatValue); 
		}

		newItem = new QTableWidgetItem(sValue);
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable |  Qt::ItemIsEnabled);
		ui.globalParametersTable->setItem(row, eParameterValueColumn, newItem);
	
		// Column 3: Is Confirmation .  Not editable text
		QString sValueConfirm;
		if (algoParameters[row].paramType == eAlgoParamInt)
		{
			sValueConfirm.sprintf("%d", algoParameters[row].intValue); 
		}
		else
		{
			sValueConfirm.sprintf("%f", algoParameters[row].floatValue); 
		}
		newItem = new QTableWidgetItem(sValueConfirm);
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.globalParametersTable->setItem(row, eParameterConfirmColumn, newItem);
	}
}

void AWLQtDemo::UpdateGlobalParametersView()

{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = 0;

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{
		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *checkItem =ui.globalParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *confirmItem = ui.globalParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::PartiallyChecked)
		{
			// Going from partially checked to another value means we got an update
			// Update the value text.
			if (!algoParameters[row].pendingUpdates) 
			{
				// Update was received.  CheckState falls back to default.

				checkItem->setCheckState(Qt::Unchecked);
				ui.globalParametersTable->setItem(row, eParameterCheckColumn, checkItem);
				// Get the confirm value and format.
				QString sValueConfirm;
				if (algoParameters[row].paramType == eAlgoParamInt)
				{
					sValueConfirm.sprintf("%d", algoParameters[row].intValue); 
				}
				else
				{
				sValueConfirm.sprintf("%f", algoParameters[row].floatValue); 
				}
				confirmItem->setText(sValueConfirm);
				ui.globalParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);
			}
		}
		else 
		{
			if (algoParameters[row].pendingUpdates) 
			{
				checkItem->setCheckState(Qt::PartiallyChecked);
				ui.globalParametersTable->setItem(row, eParameterCheckColumn, checkItem);
			}
		}
	}
}

void AWLQtDemo::on_globalParametersSetPushButton_clicked()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = 0;

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.globalParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.globalParametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.globalParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.globalParametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.globalParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			// The value value is read.
			// Format depends on the cell type
			
			QString sValueText = valueItem->text();
			uint16_t parameterAddress = algoParameters[row].address;
			uint32_t parameterValue = 0L;
			if (algoParameters[row].paramType == eAlgoParamInt)
			{
				int intValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%d", &intValue);
				// Send to the parameter value as uint32_t
				parameterValue = (uint32_t) intValue;
			}
			else
			{
				int floatValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%f", &floatValue);
				parameterValue = * (uint32_t *) &floatValue;
			}

			receiverCaptures[0]->SetGlobalAlgoParameter(parameterAddress, parameterValue); 
		} // if checked
	} // for 
}

void AWLQtDemo::on_globalParametersGetPushButton_clicked()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = 0;

	AlgorithmParameterVector algoParameters = settingsPtr->receiverSettings[0].parametersAlgos.algorithms[currentAlgo].parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.globalParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.globalParametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.globalParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.globalParametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.globalParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			uint16_t parameterAddress = algoParameters[row].address;
			receiverCaptures[0]->QueryGlobalAlgoParameter(parameterAddress); 
		} // if checked
	} // for 
}


void AWLQtDemo::on_view3DActionToggled()
{

	if (ui.action3D_View->isChecked())
		fusedCloudViewer->Go();
	else
	{
		fusedCloudViewer->Stop();

		// For some reason, the closing of the 3D window messes up with our timer.
		//Restart it
		myTimer->start(LOOP_RATE);
	}
}

void AWLQtDemo::on_view2DActionToggled()
{
	if (ui.action2D->isChecked())
		m2DScan->show();
	else
		m2DScan->hide();
}

void AWLQtDemo::on_viewTableViewActionToggled()
{
	if (ui.actionTableView->isChecked())
		mTableView->show();
	else
		mTableView->hide();
}

void AWLQtDemo::on_viewGraphActionToggled()
{
	if (ui.actionGraph->isChecked()) 
	{
		scopeWindow->show();
		scopeWindow->start(receiverCaptures[0]);
	}
	else
	{
		scopeWindow->hide();
		scopeWindow->stop();
	}
}

void AWLQtDemo::on_viewCameraActionToggled()
{
	if (ui.actionCamera->isChecked())
	{
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) videoViewers[viewerID]->Go();
		}
	}

	else
	{
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) videoViewers[viewerID]->Stop();
		}
	}
}

void AWLQtDemo::on_view2DClose()
{
	ui.action2D->setChecked(false);
}

void AWLQtDemo::on_viewTableViewClose()
{
	ui.actionTableView->setChecked(false);
}

void AWLQtDemo::on_viewGraphClose()
{
	ui.actionGraph->setChecked(false);
}


void AWLQtDemo::FillFPGAList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < settingsPtr->receiverSettings[0].registersFPGA.size(); i++) 
	{
		QString sLabel = settingsPtr->receiverSettings[0].registersFPGA[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += settingsPtr->receiverSettings[0].registersFPGA[i].sDescription.c_str();
		ui.registerFPGAAddressSetComboBox->addItem(sLabel);
	}

}


void AWLQtDemo::on_registerFPGASetPushButton_clicked()
{
	uint16_t registerAddress;
	QString sValue;
	uint32_t registerValue;

	int comboIndex = ui.registerFPGAAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	registerAddress = settingsPtr->receiverSettings[0].registersFPGA[comboIndex].address;

	sValue = ui.registerFPGAValueSetLineEdit->text();
	bool ok;
	registerValue = sValue.toULong(&ok, 16);
	if (!ok) 
	{
		QMessageBox msgBox;
		msgBox.setText("Invalid format for value field");
		msgBox.exec();
		return;
	}

	// Now update user interface
	ui.registerFPGAAddressGetLineEdit->setText("");
	ui.registerFPGAValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetFPGARegister(registerAddress, registerValue);
	}

}

void AWLQtDemo::on_registerFPGAGetPushButton_clicked()
{
	uint16_t registerAddress;


	int comboIndex = ui.registerFPGAAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	registerAddress = settingsPtr->receiverSettings[0].registersFPGA[comboIndex].address;

	// Now update user interface
	ui.registerFPGAAddressGetLineEdit->setText("");
	ui.registerFPGAValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->QueryFPGARegister(registerAddress);
	}
}

void AWLQtDemo::FillADCList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < settingsPtr->receiverSettings[0].registersADC.size(); i++) 
	{
		QString sLabel = settingsPtr->receiverSettings[0].registersADC[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += settingsPtr->receiverSettings[0].registersADC[i].sDescription.c_str();
		ui.registerADCAddressSetComboBox->addItem(sLabel);
	}

}


void AWLQtDemo::on_registerADCSetPushButton_clicked()
{
	uint16_t registerAddress;
	QString sValue;
	uint32_t registerValue;

	int comboIndex = ui.registerADCAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	registerAddress = settingsPtr->receiverSettings[0].registersADC[comboIndex].address;

	sValue = ui.registerADCValueSetLineEdit->text();
	bool ok;
	registerValue = sValue.toULong(&ok, 16);
	if (!ok) 
	{
		QMessageBox msgBox;
		msgBox.setText("Invalid format for value field");
		msgBox.exec();
		return;
	}

	// Now update user interface
	ui.registerADCAddressGetLineEdit->setText("");
	ui.registerADCValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->SetADCRegister(registerAddress, registerValue);
	}

}

void AWLQtDemo::on_registerADCGetPushButton_clicked()
{
	uint16_t registerAddress;


	int comboIndex = ui.registerADCAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	registerAddress = settingsPtr->receiverSettings[0].registersADC[comboIndex].address;

	// Now update user interface
	ui.registerADCAddressGetLineEdit->setText("");
	ui.registerADCValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCaptures[0]) 
	{
		receiverCaptures[0]->QueryADCRegister(registerAddress);
	}
}


void AWLQtDemo::FillGPIOList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < settingsPtr->receiverSettings[0].registersGPIO.size(); i++) 
	{
		QString sLabel = settingsPtr->receiverSettings[0].registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += settingsPtr->receiverSettings[0].registersGPIO[i].sDescription.c_str();
		if (settingsPtr->receiverSettings[0].registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}
	

		QListWidgetItem *listItem = new QListWidgetItem(sLabel,ui.registerGPIOListWidget);
        listItem->setFlags(listItem->flags() | Qt::ItemIsUserCheckable); // set checkable flag
		listItem->setCheckState(Qt::Unchecked);
		ui.registerGPIOListWidget->addItem(listItem);


	}
}

void AWLQtDemo::UpdateGPIOList()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();

	for (int i = 0; i < settingsPtr->receiverSettings[0].registersGPIO.size(); i++) 
	{
		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);

		QString sLabel = settingsPtr->receiverSettings[0].registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += settingsPtr->receiverSettings[0].registersGPIO[i].sDescription.c_str();
		if (settingsPtr->receiverSettings[0].registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}
	
		if (settingsPtr->receiverSettings[0].registersGPIO[i].value) 
		{
			listItem->setCheckState(Qt::Checked);
		}
		else 
		{
			listItem->setCheckState(Qt::Unchecked);
		}
	}
}

void AWLQtDemo::on_registerGPIOSetPushButton_clicked()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();

	// Update all of the MIOs at the same time
	for (int i = 0; i < settingsPtr->receiverSettings[0].registersGPIO.size(); i++) 
	{
		uint16_t registerAddress = settingsPtr->receiverSettings[0].registersGPIO[i].address;
		uint32_t registerValue = 0;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);
		Qt::CheckState checkState = listItem->checkState();
		if (checkState == Qt::Checked) 
		{
			registerValue = 1;
		}


		// Send the command to the device
		if (receiverCaptures[0]) 
		{
			receiverCaptures[0]->SetGPIORegister(registerAddress, registerValue);
		}

		// Update the user interface
		QString sLabel = settingsPtr->receiverSettings[0].registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += settingsPtr->receiverSettings[0].registersGPIO[i].sDescription.c_str();
		if (settingsPtr->receiverSettings[0].registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);
	}// For
}


void AWLQtDemo::on_registerGPIOGetPushButton_clicked()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();

	// Update all of the MIOs at the same time
	for (int i = 0; i < settingsPtr->receiverSettings[0].registersGPIO.size(); i++) 
	{
		uint16_t registerAddress = settingsPtr->receiverSettings[0].registersGPIO[i].address;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);
	
		// Send the command to the device
		if (receiverCaptures[0]) 
		{
			receiverCaptures[0]->QueryGPIORegister(registerAddress);		
		}

		// Update the user interface
		QString sLabel = settingsPtr->receiverSettings[0].registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += settingsPtr->receiverSettings[0].registersGPIO[i].sDescription.c_str();
		if (settingsPtr->receiverSettings[0].registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);

	}
}

void AWLQtDemo::closeEvent(QCloseEvent * event)
{
	if (fusedCloudViewer) fusedCloudViewer->Stop();

	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}


	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	if (receiver) receiver->Stop();
	for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
	{
		if (videoViewers[viewerID]) videoViewers[viewerID]->Stop();
	}

	qApp->closeAllWindows();
}	

