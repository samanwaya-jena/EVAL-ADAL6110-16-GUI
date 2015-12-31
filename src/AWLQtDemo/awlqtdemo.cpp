/* AWLQtDemo.cpp */
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


#include <boost/foreach.hpp>


#include "AWLSettings.h"
#include "AWLCoord.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "ReceiverEasySyncCapture.h"
#include "ReceiverKvaserCapture.h"
#include "ReceiverSimulatorCapture.h"
#include "ReceiverPostProcessor.h"

#include "DebugPrintf.h"
#include "awlqtdemo.h"

#include <QTableWidget>
#include <QDesktopWidget>
#include <QApplication>
#include <QTime>
#include <QMessageBox>
#include <QListWidget>

#include <string>


#include "tableview.h"

#include "..\awlqtscope\awlqtscope.h"


using namespace std;
using namespace awl;

// Text update rate, in frame per seconds
#define LOOP_RATE	30

TransformationNode::Ptr myBaseNode;

AWLQtDemo::AWLQtDemo(int argc, char *argv[])
	: QMainWindow()
{
	ui.setupUi(this);

	// Read the settigs from the configuration file
	AWLSettings *globalSettings = AWLSettings::InitSettings();
	globalSettings->ReadSettings();

	// Change the window icon if there is an override in the INI file
	if (!globalSettings->sIconFileName.empty())
	{
		QIcon icon(globalSettings->sIconFileName.c_str());
		QApplication::setWindowIcon(icon);
		setWindowIcon(icon);
	}

	// Build a reference coodinate system from the settings
	AWLCoordinates *globalCoordinates = AWLCoordinates::InitCoordinates();
	globalCoordinates->BuildCoordinatesFromSettings(globalSettings->GetPropTree());

	// Adjust the default displayed ranges depending on the sensor capabilities
	AdjustDefaultDisplayedRanges();

	// Create the receiver communication objects
	int receiverQty = globalSettings->receiverSettings.size();
	receiverCaptures.resize(receiverQty);
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Create the LIDAR acquisition thread object, depending on the type identified in the config file

		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string("EasySyncCAN"))
		{
			// EasySync CAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverEasySyncCapture(receiverID, globalSettings->GetPropTree()));
		}
		else if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "KvaserLeaf"))
		{
			// Kvaser Leaf CAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverKvaserCapture(receiverID, globalSettings->GetPropTree()));
		}
		else 
		{
			// If the type is undefined, just use the dumb simulator, not using external device
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverSimulatorCapture(receiverID, globalSettings->GetPropTree()));
		}

		receiverCaptureSubscriberIDs.push_back(receiverCaptures[receiverID]->Subscribe());
	}

	// Create the video capture objects
	int videoCaptureQty = globalSettings->cameraSettings.size();
	for (int cameraID = 0; cameraID < videoCaptureQty; cameraID++)
	{
		videoCaptures.push_back(VideoCapture::Ptr(new VideoCapture(cameraID, argc, argv,globalSettings->GetPropTree())));
	}

	// Create the video viewer to display the camera image
	// The video viewer feeds from the  videoCapture (for image) and from the receiver (for distance info)
	int videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		QString cameraName(this->windowTitle()+" Camera");
		cameraName.append(QString().sprintf(" %02d", videoViewerID));
		
		VideoViewer *viewer =  new VideoViewer(cameraName.toStdString(), videoCaptures[videoViewerID]);
		videoViewers.push_back(VideoViewer::Ptr(viewer));
	}

	// Fill the parameters  tables from the settings
	FillFPGAList(globalSettings);
	FillADCList(globalSettings);
	FillGPIOList(globalSettings);

	// Prepare the parameters view
	PrepareParametersView();
	PrepareGlobalParametersView();

	// Initialize the controls from the settings in INI file
	RelativePosition sensorPosition = AWLCoordinates::GetReceiverPosition(0);
	ui.sensorHeightSpinBox->setValue(sensorPosition.position.up);
	ui.sensorDepthSpinBox->setValue(sensorPosition.position.forward);
	double measurementOffset;
	receiverCaptures[0]->GetMeasurementOffset(measurementOffset);
	ui.measurementOffsetSpinBox->setValue(measurementOffset);
	ui.sensorRangeMinSpinBox->setValue(globalSettings->receiverSettings[0].displayedRangeMin);

	ui.sensorRangeMax0SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[0].maxRange);
	ui.sensorRangeMax1SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[1].maxRange);
	ui.sensorRangeMax2SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[2].maxRange);
	ui.sensorRangeMax3SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[3].maxRange);
	ui.sensorRangeMax4SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[4].maxRange);
	ui.sensorRangeMax5SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[5].maxRange);
	ui.sensorRangeMax6SpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[6].maxRange);

	ui.targetHintDistanceSpinBox->setValue(receiverCaptures[0]->targetHintDistance);
	ui.targetHintAngleSpinBox->setValue(receiverCaptures[0]->targetHintAngle);

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
		
		ui.frameRateSpinBox->setValue(0);
	}


	// Initialize from other operating variables.
	ui.distanceLogFileCheckbox->setChecked(globalSettings->bWriteLogFile);

	 scopeWindow = new AWLQtScope();

	// Initialize the 2D view
	m2DScan = new FOV_2DScan();
	m2DScan->setWindowTitle(this->windowTitle() + " 2D View");
	mCfgSensor.shortRangeDistance = globalSettings->shortRangeDistance;
    mCfgSensor.shortRangeDistanceStartLimited = globalSettings->shortRangeDistanceStartLimited;
    mCfgSensor.shortRangeAngle = globalSettings->shortRangeAngle;
    mCfgSensor.shortRangeAngleStartLimited = globalSettings->shortRangeAngleStartLimited;

    mCfgSensor.longRangeDistance = globalSettings->longRangeDistance;
    mCfgSensor.longRangeDistanceStartLimited = globalSettings->longRangeDistanceStartLimited;
    mCfgSensor.longRangeAngle = globalSettings->longRangeAngle;
    mCfgSensor.longRangeAngleStartLimited = globalSettings->longRangeAngleStartLimited;

	mCfgSensor.spareDepth = -sensorPosition.position.forward;

	m2DScan->slotConfigChanged(mCfgSensor);

	// Initialize the table view
	mTableView = new TableView();
	mTableView->setWindowTitle(this->windowTitle() + " Table View");

	// Start the threads for background  receiver capture objects
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++) 
	{ 
		receiverCaptures[receiverID]->Go();
	}

	// Start the threads for background video capture objects
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{ 
		videoCaptures[cameraID]->Go();
	}

	// Create a timer to keep the UI objects spinning
     myTimer = new QTimer(this);
     connect(myTimer, SIGNAL(timeout()), this, SLOT(on_timerTimeout()));
	 myTimer->start(LOOP_RATE);

	// Initial Update the various status indicators on the display
	DisplayReceiverStatus();

	switch (receiverCaptures[0]->parametersAlgos.defaultAlgo)
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
	ui.calibrationBetaDoubleSpinBox->setValue(0.8);

	// Configure the Toolbar
	SetupToolBar();

	// View signals and slots on close
	connect(m2DScan, SIGNAL(closed()), this, SLOT(on_view2DClose()));
	connect(mTableView, SIGNAL(closed()), this, SLOT(on_viewTableViewClose()));
	connect(scopeWindow, SIGNAL(closed( )), this, SLOT(on_viewGraphClose()));

	connect(ui.actionGraph, SIGNAL(toggled(bool )), this, SLOT(on_viewGraphActionToggled()));
	ui.actionGraph->setChecked(globalSettings->bDisplayScopeWindow);

	// Setup the display grid and position the objects in the grid
	SetupDisplayGrid();

	// Show hide the windows according to menu
	// Scope
	if (ui.actionGraph->isChecked()) 
	 {
		 scopeWindow->show();
	 }
	else 
	{
		scopeWindow->hide();
	}

	// Interface parameters
	if (actionSettingsButton->isChecked()) 
	{   
		ui.interfaceTabs->show();
	}
	else
	{
		ui.interfaceTabs->hide();
	}

	// 2D View
	if (action2DButton->isChecked())
	{
		m2DScan->show();
	}
	else
	{		
		m2DScan->hide();
	}


	// Table view
	if (actionTableButton->isChecked()) 
	{
		mTableView->show();
	}
	else
	{
		mTableView->hide();
	}


	// Camera views
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		if (actionCameraButton->isChecked())
		{
			videoViewers[videoViewerID]->show();
		}
		else 
		{
			videoViewers[videoViewerID]->hide();
		}
	}


	// Position the main widget on the top left corner
	QRect scr = QApplication::desktop()->availableGeometry(QApplication::desktop()->primaryScreen());
	QRect frame = frameGeometry();
	QRect client = geometry();
	int verticalDecorationsHeight = frame.height() - client.height();
	int horizontalDecorationsWidth = frame.width() - client.width();
	move(scr.right() - (frame.width()+1),  scr.bottom()-(frame.height()+33)); 

	if (globalSettings->sDisplayShowSize == std::string("FullScreen"))
		showFullScreen();
	else if (globalSettings->sDisplayShowSize == std::string("Maximized"))
		showMaximized();
	else if (globalSettings->sDisplayShowSize == std::string("Minimized"))
		showMinimized();
	else if (globalSettings->sDisplayShowSize == std::string("Normal"))
		showNormal();
	else
		showNormal();


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


void AWLQtDemo::SetupToolBar()

{
	// Read the settigs from the configuration file
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	ui.mainToolBar->setStyleSheet("QToolBar{spacing:10px;}");
	// Toolbar items signals and slots
	action2DButton = new QAction(QIcon("scan.png"), "2D View", 0);
	action2DButton->setCheckable(true);
	action2DButton->setChecked(globalSettings->bDisplay2DWindow);	
	ui.mainToolBar->addAction(action2DButton);

	actionTableButton = new QAction(QIcon("Grid.png"), "Table View", 0);
	actionTableButton->setCheckable(true);
	actionTableButton->setChecked(globalSettings->bDisplayTableViewWindow);
	ui.mainToolBar->addAction(actionTableButton);

	actionCameraButton = new QAction(QIcon("Camera.png"), "Camera View", 0);
	actionCameraButton->setCheckable(true);
	actionCameraButton->setChecked(globalSettings->bDisplayCameraWindow);
	ui.mainToolBar->addAction(actionCameraButton);

	actionSettingsButton = new QAction(QIcon("settings.png"), "Settings", 0);
	actionSettingsButton->setCheckable(true);
	actionSettingsButton->setChecked(globalSettings->bDisplaySettingsWindow);
	ui.mainToolBar->addAction(actionSettingsButton);

	// Adding the space will force the buttons placed after the spacer to be right-aligned
	QWidget* spacerRightAligned = new QWidget();
	spacerRightAligned->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	ui.mainToolBar->addWidget(spacerRightAligned);

	actionResizeButton = new QAction(QIcon("Maximize.png"), "Quit Application", 0);
	actionResizeButton->setCheckable(true);
	actionResizeButton->setChecked(globalSettings->sDisplayShowSize == std::string("FullScreen"));
	ui.mainToolBar->addAction(actionResizeButton);

	actionQuitButton = new QAction(QIcon("Quit.png"), "Quit Application", 0);
	ui.mainToolBar->addAction(actionQuitButton);

	connect(action2DButton, SIGNAL(toggled(bool )), this, SLOT(on_view2DActionToggled()));
	connect(actionTableButton, SIGNAL(toggled(bool )), this, SLOT(on_viewTableViewActionToggled()));
	connect(actionCameraButton, SIGNAL(toggled(bool )), this, SLOT(on_viewCameraActionToggled()));
	connect(actionSettingsButton, SIGNAL(toggled(bool )), this, SLOT(on_viewSettingsActionToggled()));
	connect(actionResizeButton, SIGNAL(toggled(bool )), this, SLOT(on_resizeActionToggled()));
	connect(actionQuitButton, SIGNAL(triggered(bool )), qApp, SLOT(closeAllWindows()));
}

void AWLQtDemo::SetupDisplayGrid()

{

	// Position the objects in the layout in order
#if 0
	// Read the settigs from the configuration file
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	// Create a label to hold a logo, only if there is one specified in INI file.

	QLabel *mLogoLabel = new QLabel(this);
	QPixmap *myPix = NULL;
	if (!globalSettings->sLogoFileName.empty()) 
	{
		myPix = new QPixmap(globalSettings->sLogoFileName.c_str());
	}
	
	float logoAspectRatio = 3.0/1.0;
	if (myPix && !myPix->isNull())
	{
		float pixWidth = myPix->width();
		float pixHeight = myPix->height();
		float logoAspectRatio = pixWidth / pixHeight;
		mLogoLabel->setPixmap(*myPix);
	}

	if (myPix) delete myPix;

	mLogoLabel->setScaledContents(true);
	ui.gridDisplayLayout->addWidget(mLogoLabel, 2, 0, 1, 1, Qt::AlignBottom | Qt::AlignLeft);

#endif

#if 0
	ui.gridDisplayLayout->addWidget((m2DScan, 0, 0, 3, 3, Qt::AlignTop);
	ui.gridDisplayLayout->addWidget(mTableView, 0, 4, 3, 1, Qt::AlignTop);

	int videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		ui.gridDisplayLayout->addWidget(videoViewers[videoViewerID].get(), videoViewerID, 5, 1, 1, Qt::AlignTop);
	}
#else
	ui.gridDisplayLayout->addWidget(m2DScan, 0, 0, Qt::AlignTop);
	ui.gridDisplayLayout->addWidget(mTableView, 0, 1, Qt::AlignTop);

	int videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		ui.gridDisplayLayout->addWidget(videoViewers[videoViewerID].get(), videoViewerID, 2, Qt::AlignTop);
	}
#endif
}

void AWLQtDemo::on_destroy()
{
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}

	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	if (m2DScan) delete m2DScan;
	if (mTableView) delete mTableView;	
	if (scopeWindow) delete scopeWindow;
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

void AWLQtDemo::on_sensorHeightSpin_editingFinished()
{
	double height = ui.sensorHeightSpinBox->value();
	RelativePosition sensorPosition = AWLCoordinates::GetReceiverPosition(0);
	if (abs(height-sensorPosition.position.up) < 0.001) return;
	sensorPosition.position.up = height;

	AWLCoordinates::SetReceiverPosition(0, sensorPosition);

	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();

	// Process
	if (m2DScan && !m2DScan->isHidden())
	{
		m2DScan->slotConfigChanged(mCfgSensor);
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
}


void AWLQtDemo::on_sensorDepthSpin_editingFinished()
{
	double forward = ui.sensorDepthSpinBox->value();
	RelativePosition sensorPosition = AWLCoordinates::GetReceiverPosition(0);
	if (abs(forward - sensorPosition.position.forward) < 0.001) return;

	sensorPosition.position.forward = forward;
	AWLCoordinates::SetReceiverPosition(0, sensorPosition);


	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();

	// Process
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
}

void AWLQtDemo::on_receiverCalibStorePushButton_clicked()

{
	AWLSettings::GetGlobalSettings()->StoreReceiverCalibration();
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

	receiverCaptures[0]->targetHintDistance = distance;
}

void AWLQtDemo::on_targetHintAngleSpin_editingFinished()
{
	double angle = ui.targetHintAngleSpinBox->value();

	receiverCaptures[0]->targetHintAngle = angle;
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
	

	// Check that the cameras are still working.  Otherwise Stop everyting
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++)
	{
		if (videoCaptures[cameraID]->WasStopped()) 
		{
			bContinue = false;
			break;
		}
	}

	
	// For each receiver. Validate that the receiver exists.
	// Then display the status flags.

	if (bContinue)
	{
		for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
		{
			if (!receiverCaptures[receiverID]) 
			{
				bContinue = false;
				break;
			}

			// Update the status information
			if (receiverCaptures[receiverID]->receiverStatus.bUpdated) 
			{
				DisplayReceiverStatus(receiverID);
			}
		}// For
	}

	// Copy all of the current detection data into a Detection data vector.
	// Update display for:
	//     - The 2 view (pie chart)
	//     - The camera views
	//     - The table views
	if (bContinue) 
	{
		Detection::Vector detectionData;
		bool bNewDetections = GetLatestDetections(detectionData);		
		
		// Update the 2D view only if there is new data.
		if (m2DScan && bNewDetections) m2DScan->slotDetectionDataChanged(detectionData);

		// Update the data for the camera views. Only if detections have changed have changed.
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID] && bNewDetections) videoViewers[viewerID]->slotDetectionDataChanged(detectionData);
		}

		// Update the table views only if there is new data
		if (mTableView && bNewDetections) mTableView->slotDetectionDataChanged(detectionData);
	}

	if (bContinue) 
	{
		// Always spin the video viewers.
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) videoViewers[viewerID]->slotImageChanged();
		}
	}

	// Let's go for the next run
	if (bContinue)
	{
		myTimer->start(LOOP_RATE);
	}
	else 
	{
		this->close();
	}
}

bool AWLQtDemo::GetLatestDetections(Detection::Vector &detectionData)
{
	AWLSettings *settings = AWLSettings::GetGlobalSettings();
	bool bNew = false;

	ReceiverPostProcessor postProcessor;

	// Build the list of detections that need to be updated
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];
		ReceiverSettings &receiverSettings = settings->receiverSettings[receiverID];


		// Use the frame snapped by the main display timer as the current frame
		Publisher::SubscriberID subscriberID = receiverCaptureSubscriberIDs[receiverID];
		uint32_t lastDisplayedFrame = receiver->GetCurrentIssueID(subscriberID);
		if (receiver->HasNews(subscriberID))
		{
			bNew = true;	
		}


		// Thread safe
		// The UI thread "Snaps" the frame ID for all other interface objects to display
		Detection::Vector detectionBuffer;

		if (postProcessor.GetEnhancedDetectionsFromFrame(receiver, lastDisplayedFrame, subscriberID, detectionBuffer))
		{
			// Copy and filter the detection data to keep only those we need.
			BOOST_FOREACH(const Detection::Ptr &detection, detectionBuffer)
			{
				Detection::Ptr storedDetection = detection;
				detectionData.push_back(storedDetection);
			}
		} // If (receiver...
	}

	return(bNew);
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

		ReceiverStatus status;
		receiverCaptures[receiverID]->CopyReceiverStatusData(status, receiverCaptureSubscriberIDs[receiverID]);
		
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
	int currentAlgo = 0;

	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = receiverCaptures[0]->parametersAlgos.defaultAlgo;
	}

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;
	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = receiverCaptures[0]->parametersAlgos.defaultAlgo;
	}

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;
	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = receiverCaptures[0]->parametersAlgos.defaultAlgo;
	}

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;
	if (receiverCaptures[0]) 
	{
		currentAlgo = receiverCaptures[0]->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = receiverCaptures[0]->parametersAlgos.defaultAlgo;
	}

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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
	int currentAlgo = 0;

	if (currentAlgo >= receiverCaptures[0]->parametersAlgos.algorithms.size()) return;

	AlgorithmParameterVector algoParameters = receiverCaptures[0]->parametersAlgos.algorithms[currentAlgo].parameters;
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

void AWLQtDemo::on_view2DActionToggled()
{
	if (action2DButton->isChecked())
	{
		m2DScan->show();
		m2DScan->slotConfigChanged(mCfgSensor);  // Force redraw with the current video parameters
		action2DButton->setChecked(true);
	}
	else 
	{
		m2DScan->hide();
		action2DButton->setChecked(false);
	}
}

void AWLQtDemo::on_viewTableViewActionToggled()
{
	if (actionTableButton->isChecked()) 
	{
		mTableView->show();
		mTableView->slotConfigChanged();
		actionTableButton->setChecked(true);
	}
	else
	{
		mTableView->hide();
		actionTableButton->setChecked(false);
	}
}

void AWLQtDemo::on_viewSettingsActionToggled()
{
	if (actionSettingsButton->isChecked()) 
	{
	
		ui.interfaceTabs->show();
		actionSettingsButton->setChecked(true);
	}
	else 
	{
		ui.interfaceTabs->hide();
		actionSettingsButton->setChecked(false);
	}
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
	if (actionCameraButton->isChecked())
	{
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) {
				videoViewers[viewerID]->show();
			}
		}
		actionCameraButton->setChecked(true);
	}

	else
	{
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) 
			{
				videoViewers[viewerID]->hide();
			}
		}
		actionCameraButton->setChecked(false);
	}
}

void AWLQtDemo::on_resizeActionToggled()
{
	if (actionResizeButton->isChecked()) 
	{
	
		showFullScreen();
		actionResizeButton->setChecked(true);
	}
	else 
	{
		showMaximized();
		actionResizeButton->setChecked(false);
	}
}

void AWLQtDemo::on_view2DClose()
{
	action2DButton->setChecked(false);
}

void AWLQtDemo::on_viewTableViewClose()
{
	actionTableButton->setChecked(false);
}

void AWLQtDemo::on_viewGraphClose()
{
	ui.actionGraph->setChecked(false);
}


void AWLQtDemo::FillFPGAList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < receiverCaptures[0]->registersFPGA.size(); i++) 
	{
		QString sLabel = receiverCaptures[0]->registersFPGA[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersFPGA[i].sDescription.c_str();
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

	if (comboIndex >= receiverCaptures[0]->registersFPGA.size()) return;

	registerAddress = receiverCaptures[0]->registersFPGA[comboIndex].address;

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

	if (comboIndex >= receiverCaptures[0]->registersFPGA.size()) return;

	registerAddress = receiverCaptures[0]->registersFPGA[comboIndex].address;

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
	for (int i = 0; i < receiverCaptures[0]->registersADC.size(); i++) 
	{
		QString sLabel = receiverCaptures[0]->registersADC[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersADC[i].sDescription.c_str();
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

	if (comboIndex >= receiverCaptures[0]->registersADC.size()) return;
	registerAddress = receiverCaptures[0]->registersADC[comboIndex].address;

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

	if (comboIndex >= receiverCaptures[0]->registersFPGA.size()) return;
	registerAddress = receiverCaptures[0]->registersADC[comboIndex].address;

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
	for (int i = 0; i < receiverCaptures[0]->registersGPIO.size(); i++) 
	{
		QString sLabel = receiverCaptures[0]->registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[i].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates)
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
	for (int i = 0; i < receiverCaptures[0]->registersGPIO.size(); i++) 
	{
		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);

		QString sLabel = receiverCaptures[0]->registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[i].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}
	
		if (receiverCaptures[0]->registersGPIO[i].value) 
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
	// Update all of the MIOs at the same time
	for (int i = 0; i < receiverCaptures[0]->registersGPIO.size(); i++) 
	{
		uint16_t registerAddress = receiverCaptures[0]->registersGPIO[i].address;
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
		QString sLabel = receiverCaptures[0]->registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[i].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);
	}// For
}


void AWLQtDemo::on_registerGPIOGetPushButton_clicked()
{
	// Update all of the MIOs at the same time
	for (int i = 0; i < receiverCaptures[0]->registersGPIO.size(); i++) 
	{
		uint16_t registerAddress = receiverCaptures[0]->registersGPIO[i].address;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);
	
		// Send the command to the device
		if (receiverCaptures[0]) 
		{
			receiverCaptures[0]->QueryGPIORegister(registerAddress);		
		}

		// Update the user interface
		QString sLabel = receiverCaptures[0]->registersGPIO[i].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[i].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);

	}
}

void AWLQtDemo::closeEvent(QCloseEvent * event)
{
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}


	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	qApp->closeAllWindows();
}	

