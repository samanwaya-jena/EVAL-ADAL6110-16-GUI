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
#include "awlcoord.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#ifdef USE_CAN_EASYSYNC
#include "ReceiverEasySyncCapture.h"
#endif
#ifdef USE_CAN_SOCKETCAN
#include "ReceiverSocketCANCapture.h"
#endif
#ifdef USE_POSIXUDP
#include "ReceiverPosixUDPCapture.h"
#endif
#ifdef USE_POSIXTTY
#include "ReceiverPosixTTYCapture.h"
#endif
#ifdef USE_LIBUSB
#include "ReceiverLibUSBCapture.h"
#endif
#ifdef USE_TCP
#include "ReceiverTCPCapture.h"
#endif
#ifdef USE_CAN_KVASER
#include "ReceiverKvaserCapture.h"
#endif
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
#include <QStandardPaths>
#include <QDir>


#include <string>


#include "TableView.h"

#include "awlqtscope.h"


using namespace std;
using namespace awl;

// Text update rate, in frame per seconds
#define LOOP_RATE	30

TransformationNode::Ptr myBaseNode;

const QString actionResizeMaximizeString("Maximize");
const QString actionResizeRestoreDownString("Restore down");

AWLQtDemo::AWLQtDemo(int argc, char *argv[])
	: QMainWindow(),
	m2DScan(NULL),
	mTableView(NULL),
	scopeWindow(NULL),
	myTimer(NULL),
	actionSettingsButton(NULL),
	action2DButton(NULL),
	actionTableButton(NULL),
	actionAScanButton(NULL),
#ifdef USE_OPENCV_VIDEO
	actionCameraButton(NULL),
#endif
	actionResizeButton(NULL),
	actionQuitButton(NULL),
	actionResizeMaximizeIcon(NULL),
	actionResizeRestoreDownIcon(NULL),
	m_bConnected(false),
	m_frameRate(1)
{
	labelConnected = new QLabel("Initializing...");
	labelFramerate = new QLabel();

	QMessageBox msgBox(this);

	ui.setupUi(this);

	// Set the basic paths
	QCoreApplication::setOrganizationName("Phantom Intelligence");
	QCoreApplication::setApplicationName("Phantom Intelligence Lidar Demo");
	
	// Set-up the debug and log file paths

	// First, ask Qt for the recommmended directory
	QString sDebugAndLogPath = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
	//  if the directory does not exist, make it.
	QDir debugAndLogDir(sDebugAndLogPath);
	if (!debugAndLogDir.exists()) 
	{
		debugAndLogDir.mkpath(".");
	}

	// Append the last "/", which Qt does not do.
	sDebugAndLogPath += "/";
#if 1
	sDebugAndLogPath = QString("");
#endif
	SetLogAndDebugFilePath(sDebugAndLogPath.toStdString().c_str());


	// Read the settigs from the configuration file
	//
	// First, ask Qt for the existence of the file
	QString sSettingsPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
	sSettingsPath += "/";
#if 1
	sSettingsPath = QString("");
#endif
	//  if the file does not exist, use the current directory.
	QFile settingsFile(QString("AWLDemoSettings.xml"));

	if (!settingsFile.exists())
	{
		settingsFile.setFileName(QString("AWLDemoSettings.xml"));

		// If no file found, exit gracefully
		if (!settingsFile.exists())
		{
			msgBox.setText("Cannot find configuration file AWLDemoSettings.xml");
			msgBox.exec();
			exit(0);
		}
	}

	AWLSettings *globalSettings = AWLSettings::InitSettings(settingsFile.fileName().toStdString());
	globalSettings->ReadSettings();

	// Change the window icon if there is an override in the INI file
	if (!globalSettings->sIconFileName.empty())
	{
		QIcon icon(globalSettings->sIconFileName.c_str());
		QApplication::setWindowIcon(icon);
	}

	// Build a reference coodinate system from the settings
	AWLCoordinates *globalCoordinates = AWLCoordinates::InitCoordinates();
	globalCoordinates->BuildCoordinatesFromSettings(globalSettings->GetPropTree());

	// Adjust the default displayed ranges depending on the sensors capabilities
	AdjustDefaultDisplayedRanges();

	// Create the receiver communication objects
	int receiverQty = globalSettings->receiverSettings.size();
	receiverCaptures.resize(receiverQty);
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Create the LIDAR acquisition thread object, depending on the type identified in the config file

#ifdef USE_CAN_EASYSYNC
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string("EasySyncCAN"))
		{
			// EasySync CAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverEasySyncCapture(receiverID, globalSettings->GetPropTree()));
		}
		else
#endif
#ifdef USE_CAN_KVASER
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "KvaserLeaf"))
		{
			// Kvaser Leaf CAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverKvaserCapture(receiverID, globalSettings->GetPropTree()));
		}
		else
#endif
#ifdef USE_CAN_SOCKETCAN
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "SocketCAN"))
		{
			// SocketCAN Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverSocketCANCapture(receiverID, globalSettings->GetPropTree()));
		}
		else 
#endif
#ifdef USE_POSIXUDP
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "PosixUDP"))
		{
			// PosixUDP Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverPosixUDPCapture(receiverID, globalSettings->GetPropTree()));
		}
		else 
#endif
#ifdef USE_POSIXTTY
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "PosixTTY"))
		{
			// PosixTTY Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverPosixTTYCapture(receiverID, globalSettings->GetPropTree()));
		}
		else 
#endif
#ifdef USE_TCP
    if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string("TCP"))
    {
      // PosixTTY Capture is used if defined in the ini file
      receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverTCPCapture(receiverID, globalSettings->GetPropTree()));
    }
    else
#endif
#ifdef USE_LIBUSB
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "LibUSB"))
		{
			// LibUSB Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverLibUSBCapture(receiverID, globalSettings->GetPropTree()));
		}
		else 
#endif
		{
			// If the type is undefined, just use the dumb simulator, not using external device
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverSimulatorCapture(receiverID, globalSettings->GetPropTree()));
		}

		receiverCaptureSubscriberIDs.push_back(receiverCaptures[receiverID]->Subscribe());
	}

#ifdef USE_OPENCV_VIDEO
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
#endif

	// Fill the parameters  tables from the settings
	FillFPGAList(globalSettings);
	FillADCList(globalSettings);
	FillGPIOList(globalSettings);

	// Prepare the parameters view
	PrepareAlgoParametersView();
	PrepareGlobalParametersView();

	// Initialize the controls from the settings in INI file

	ui.calibrationChannelMaskGroupBox->setVisible(false);
	ui.channelMaskGroupBox->setVisible(false);

	RelativePosition sensorPosition = AWLCoordinates::GetReceiverPosition(0);
	ui.sensorHeightSpinBox->setValue(sensorPosition.position.up);
	ui.sensorDepthSpinBox->setValue(sensorPosition.position.forward);
	double measurementOffset;
	receiverCaptures[0]->GetMeasurementOffset(measurementOffset);
	ui.measurementOffsetSpinBox->setValue(measurementOffset);
	ui.sensorRangeMinSpinBox->setValue(globalSettings->receiverSettings[0].displayedRangeMin);

	ui.sensorRangeMaxSpinBox->setValue(globalSettings->receiverSettings[0].channelsConfig[0].maxRange);

	FillChannelSelectList();

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

	// Fill in the algo select combo box
	ui.algoSelectComboBox->clear();
	int algoQty = receiverCaptures[0]->parametersAlgos.algorithms.size();
	for (int i = 1; i < algoQty; i++)
	{
		QString algoLabel = QString(receiverCaptures[0]->parametersAlgos.algorithms[i].sAlgoName.c_str());
		uint16_t algoID = receiverCaptures[0]->parametersAlgos.algorithms[i].algoID;
		ui.algoSelectComboBox->addItem(algoLabel, QVariant(algoID));
	}

	int selectedAlgoID = receiverCaptures[0]->parametersAlgos.defaultAlgo;
	ui.algoSelectComboBox->setCurrentIndex(ui.algoSelectComboBox->findData(selectedAlgoID));

	// Fill in the tracker select combo box
	ui.trackerSelectComboBox->clear();
	int trackerQty = receiverCaptures[0]->parametersTrackers.algorithms.size();
	for (int i = 0; i < trackerQty; i++)
	{
		QString trackerLabel = QString(receiverCaptures[0]->parametersTrackers.algorithms[i].sAlgoName.c_str());
		uint16_t trackerID = receiverCaptures[0]->parametersTrackers.algorithms[i].algoID;
		ui.trackerSelectComboBox->addItem(trackerLabel, QVariant(trackerID));
	}

	int selectedTrackerID = receiverCaptures[0]->parametersTrackers.defaultAlgo;
	ui.trackerSelectComboBox->setCurrentIndex(ui.trackerSelectComboBox->findData(selectedTrackerID));

  // AdvancedMode
  connect(ui.checkBoxAdvanceMode, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAdvanceModeToggled()));

  // Ascan selection
  connect(ui.checkBox_1, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_2, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_3, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_4, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_5, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_6, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_7, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_8, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_9, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_10, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_11, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_12, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_13, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_14, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_15, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));
  connect(ui.checkBox_16, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAscanSelToggled()));

  // AutoScale
  connect(ui.checkBoxAutoScale, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAutoScaleToggled()));

  ui.comboBoxMaxRange->setDisabled(true);
  ui.comboBoxMaxRange->clear();
  ui.comboBoxMaxRange->addItem("100%");
  ui.comboBoxMaxRange->addItem("75%");
  ui.comboBoxMaxRange->addItem("50%");
  ui.comboBoxMaxRange->addItem("25%");

	// Calibration 
	ui.calibrationBetaDoubleSpinBox->setValue(0.8);
		

	// Initialize from other operating variables.
	ui.distanceLogFileCheckbox->setChecked(globalSettings->bWriteLogFile);

	// Initialize the scope window
	//scopeWindow = new AWLScopePlot();
	scopeWindow = new AWLPlotScan();

	// Initialize the 2D view
	m2DScan = new FOV_2DScan();
	m2DScan->setWindowTitle(this->windowTitle() + " 2D View");
   
	m2DScan->slotConfigChanged();

	// Initialize the table view
	mTableView = new TableView();
	mTableView->setWindowTitle(this->windowTitle() + " Table View");

	// Create a timer to keep the UI objects spinning
     myTimer = new QTimer(this);
     connect(myTimer, SIGNAL(timeout()), this, SLOT(on_timerTimeout()));
	 myTimer->start(LOOP_RATE);

	// Initial Update the various status indicators on the display
	DisplayReceiverStatus();

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
	// 
	
	if (!globalSettings->bTabSettingCalibration) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.calibrationTab));
        }
        if (!globalSettings->bTabSettingControl) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.controlTab));
        }
        if (!globalSettings->bTabSettingStatus) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.statusTab));
        }
        if (!globalSettings->bTabSettingRegisters) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.registersTab));
        }
        if (!globalSettings->bTabSettingGPIOs) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.gpiosTab));
        }
        if (!globalSettings->bTabSettingAlgoControl) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.AlgoTab));
        }
	if (!globalSettings->bTabSettingTrackerControl) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.TrackerTab));
        }

	on_view2DActionToggled();
        on_viewTableViewActionToggled();
        on_viewAScanViewActionToggled();
#ifdef USE_OPENCV_VIDEO
        on_viewCameraActionToggled();
#endif
        on_viewSettingsActionToggled();


	// Position the main widget on the top left corner
	QRect scr = QApplication::desktop()->availableGeometry(QApplication::desktop()->primaryScreen());
	QRect frame = frameGeometry();
	QRect client = geometry();
	int verticalDecorationsHeight = frame.height() - client.height();
	int horizontalDecorationsWidth = frame.width() - client.width();
	move(scr.right() - (frame.width()+1),  scr.bottom()-(frame.height()+33)); 

	if (globalSettings->sDisplayShowSize == std::string("FullScreen"))
	{
		showMaximized();
		showFullScreen();
	}
	else if (globalSettings->sDisplayShowSize == std::string("Maximized"))
		showMaximized();
	else if (globalSettings->sDisplayShowSize == std::string("Minimized"))
		showMinimized();
	else if (globalSettings->sDisplayShowSize == std::string("Normal"))
		showNormal();
	else
		showNormal();

#if 1
	// Start the threads for background  receiver capture objects
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		receiverCaptures[receiverID]->Go();
	}

#ifdef USE_OPENCV_VIDEO
	// Start the threads for background video capture objects
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++)
	{
		videoCaptures[cameraID]->Go();
	}
#endif

#endif

        //20180719-JYD  Restart the scope...... It accidentally started before the receivers were available and the time base is off....

        scopeWindow->start(receiverCaptures[0]);
	
	// Set size of statusbar & font
	ui.statusBar->setFixedHeight(30);
	ui.statusBar->setStyleSheet("QStatusBar{padding-left:20px;background:rgba(0,0,0,255);color:lightgray;font-weight:bold;font-size:20px;}");
	//ui.statusBar->showMessage(tr("Initialization ..."));

	labelConnected->setStyleSheet("QLabel{padding-left:20px;background:rgba(0,0,0,255);color:lightgray;font-weight:bold;font-size:20px;}");
	labelFramerate->setStyleSheet("QLabel{padding-left:20px;background:rgba(0,0,0,255);color:lightgray;font-weight:bold;font-size:20px;}");

	ui.statusBar->addWidget(labelConnected);
	ui.statusBar->addWidget(labelFramerate);

	// For debugging ...
	//boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&AWLQtDemo::DoThreadLoop, this)));
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
}


void AWLQtDemo::SetupToolBar()

{
	// Read the settigs from the configuration file
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	ui.mainToolBar->setStyleSheet("QToolBar{spacing:10px;}");
	// Toolbar items signals and slots
	action2DButton = new QAction(QIcon("./Images/ButtonBitmaps/Scan.png"), "2D View", 0);
	action2DButton->setCheckable(true);
	action2DButton->setChecked(globalSettings->bDisplay2DWindow);	
	ui.mainToolBar->addAction(action2DButton);

	actionTableButton = new QAction(QIcon("./Images/ButtonBitmaps/Grid.png"), "Table View", 0);
	actionTableButton->setCheckable(true);
	actionTableButton->setChecked(globalSettings->bDisplayTableViewWindow);
	ui.mainToolBar->addAction(actionTableButton);

	actionAScanButton = new QAction(QIcon("./Images/ButtonBitmaps/AScan.png"), "AScan View", 0);
	actionAScanButton->setCheckable(true);
	actionAScanButton->setChecked(globalSettings->bDisplayAScanViewWindow);
	ui.mainToolBar->addAction(actionAScanButton);

#ifdef USE_OPENCV_VIDEO
	actionCameraButton = new QAction(QIcon("./Images/ButtonBitmaps/Camera.png"), "Camera View", 0);
	actionCameraButton->setCheckable(true);
	actionCameraButton->setChecked(globalSettings->bDisplayCameraWindow);
	ui.mainToolBar->addAction(actionCameraButton);
#endif

	actionSettingsButton = new QAction(QIcon("./Images/ButtonBitmaps/Settings.png"), "Settings", 0);
	actionSettingsButton->setCheckable(true);
	actionSettingsButton->setChecked(globalSettings->bDisplaySettingsWindow);
	ui.mainToolBar->addAction(actionSettingsButton);

	// Adding the space will force the buttons placed after the spacer to be right-aligned
	QWidget* spacerRightAligned = new QWidget();
	spacerRightAligned->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	ui.mainToolBar->addWidget(spacerRightAligned);

	actionAboutButton = new QAction(QIcon("./Images/ButtonBitmaps/About.png"), "About", 0);
	actionAboutButton->setCheckable(true);
	actionAboutButton->setChecked(globalSettings->bDisplayAboutWindow);
	ui.mainToolBar->addAction(actionAboutButton);

	actionResizeMaximizeIcon = new QIcon("./Images/ButtonBitmaps/Maximize.png");
	actionResizeRestoreDownIcon = new QIcon("./Images/ButtonBitmaps/RestoreDown.png");
	actionResizeButton = new QAction(*actionResizeRestoreDownIcon, actionResizeRestoreDownString, 0);
	actionResizeButton->setCheckable(true);
	actionResizeButton->setChecked(globalSettings->sDisplayShowSize == std::string("FullScreen"));
	ui.mainToolBar->addAction(actionResizeButton);

	actionQuitButton = new QAction(QIcon("./Images/ButtonBitmaps/Quit.png"), "Quit Application", 0);
	ui.mainToolBar->addAction(actionQuitButton);

	connect(action2DButton, SIGNAL(toggled(bool )), this, SLOT(on_view2DActionToggled()));
	connect(actionTableButton, SIGNAL(toggled(bool )), this, SLOT(on_viewTableViewActionToggled()));
	connect(actionAScanButton, SIGNAL(toggled(bool )), this, SLOT(on_viewAScanViewActionToggled()));
	connect(actionAboutButton, SIGNAL(triggered(bool )), this, SLOT(on_viewAboutActionTriggered()));
#ifdef USE_OPENCV_VIDEO
	connect(actionCameraButton, SIGNAL(toggled(bool )), this, SLOT(on_viewCameraActionToggled()));
#endif
	connect(actionSettingsButton, SIGNAL(toggled(bool )), this, SLOT(on_viewSettingsActionToggled()));
	connect(actionResizeButton, SIGNAL(toggled(bool )), this, SLOT(on_resizeActionToggled()));
	connect(actionQuitButton, SIGNAL(triggered(bool )), qApp, SLOT(closeAllWindows()));
}

void AWLQtDemo::SetupDisplayGrid()

{

	// Position the objects in the layout in order

#if 0
	ui.gridDisplayLayout->addWidget((m2DScan, 0, 0, 3, 3, Qt::AlignTop);
	ui.gridDisplayLayout->addWidget(mTableView, 0, 4, 3, 1, Qt::AlignTop);

#ifdef USE_OPENCV_VIDEO
	int videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		ui.gridDisplayLayout->addWidget(videoViewers[videoViewerID].get(), videoViewerID, 5, 1, 1, Qt::AlignTop);
	}
#endif

#else
	ui.gridDisplayLayout->addWidget(m2DScan, 0, 0, Qt::AlignTop);
	ui.gridDisplayLayout->addWidget(mTableView, 0, 1, Qt::AlignTop);
       //20180719- JYD Add the scope to a bottom row of the layout
       // ui.gridDisplayLayout->addWidget(scopeWindow, videoViewerQty, 0, videoViewerQty, 3, Qt::AlignTop);
       ui.gridDisplayLayout->addWidget(scopeWindow, 0, 3, Qt::AlignTop);

#ifdef USE_OPENCV_VIDEO
	int videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		ui.gridDisplayLayout->addWidget(videoViewers[videoViewerID].get(), videoViewerID, 2, Qt::AlignTop);
	}
#endif

#endif
}

void AWLQtDemo::on_destroy()
{
#ifdef USE_OPENCV_VIDEO
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}
#endif

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
	channelMask.bitFieldData.channel7 = 1;

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
	channelMask.bitFieldData.channel7= 1;
	
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
		m2DScan->slotConfigChanged();
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
		m2DScan->slotConfigChanged();
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
}

void AWLQtDemo::on_calibrationRangeMinSpin_editingFinished()
{

	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();

	// Update the settings
	double range = ui.sensorRangeMinSpinBox->value();
	AWLSettings::GetGlobalSettings()->receiverSettings[0].displayedRangeMin = range;

	// Calculate the absolute max distance from the settings
	AdjustDefaultDisplayedRanges();

	// Update user interface parts
	if (m2DScan && !m2DScan->isHidden())
	{
		m2DScan->slotConfigChanged();
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
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
	AdjustDefaultDisplayedRanges();

	// Update user interface parts
	
	if (m2DScan && !m2DScan->isHidden())
	{
		m2DScan->slotConfigChanged();
	}

	// Restore the wait cursor
	setCursor(Qt::ArrowCursor);
}

void AWLQtDemo::on_calibrationRangeMaxSpin_editingFinished()
{
	double range = ui.sensorRangeMaxSpinBox->value();

	// Calculate the absolute max distance from the settings
	int channelQty = AWLSettings::GetGlobalSettings()->receiverSettings[0].channelsConfig.size();
	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		QListWidgetItem *listItem = ui.channelSelectListWidget->item(channelIndex);
		Qt::CheckState checkState = listItem->checkState();
		if (checkState == Qt::Checked)
		{
			ChangeRangeMax(channelIndex, range);
		}
	}
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
	channelMask.bitFieldData.channel7 = 1;

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
	
#ifdef USE_OPENCV_VIDEO
	// Check that the cameras are still working.  Otherwise Stop everyting
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++)
	{
		if (videoCaptures[cameraID]->WasStopped()) 
		{
			bContinue = false;
			break;
		}
	}
#endif

	
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

#ifdef USE_OPENCV_VIDEO
		// Update the data for the camera views. Only if detections have changed have changed.
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID] && bNewDetections) videoViewers[viewerID]->slotDetectionDataChanged(detectionData);
		}
#endif

		// Update the table views only if there is new data
		if (mTableView && bNewDetections) mTableView->slotDetectionDataChanged(detectionData);

    if (scopeWindow && bNewDetections)
    {
      AScan::Vector aScanData;
      aScanData.clear();

      bool bNewAScans = GetLatestAScans(aScanData);
      scopeWindow->AScanDataChanged(aScanData);
      scopeWindow->update();
    }
	}

  {
    bool bConnected = receiverCaptures[0]->IsConnected();
    int frameRate = receiverCaptures[0]->GetFrameRate();

		{
			QString str;
			if (bConnected)
				str = "Connected";
			else
				str = "Not connected";
			labelConnected->setText(str);

			if (bConnected)
				str.sprintf("Framerate: %3d Hz", frameRate);
			else
				str = "";
			labelFramerate->setText(str);

			//ui.statusBar->showMessage(str);
		}

  }

	if (bContinue) 
	{
#ifdef USE_OPENCV_VIDEO
		// Always spin the video viewers.
		for (int viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) videoViewers[viewerID]->slotImageChanged();
		}
#endif
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
		FrameID lastDisplayedFrame = receiver->GetCurrentIssueID(subscriberID);
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

bool AWLQtDemo::GetLatestAScans(AScan::Vector &aScanData)
{
	bool bNew = false;
	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];
		// Use the frame snapped by the main display timer as the current frame
		Publisher::SubscriberID subscriberID = receiverCaptureSubscriberIDs[receiverID];
		FrameID lastDisplayedFrame = receiver->GetCurrentIssueID(subscriberID);
		bNew = receiver->CopyReceiverAScans(lastDisplayedFrame, aScanData, subscriberID);
		//printf ("receiver %d\n", receiverID);
	}
	/*
	BOOST_FOREACH(AScan::Ptr aScan, aScanData) {
		printf ("copied ascan %d-%d\n", aScan->receiverID, aScan->channelID);
	}
	*/

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

		UpdateAlgoParametersView();
		UpdateGlobalParametersView();
		UpdateTrackerParametersView();
}

void AWLQtDemo::FillChannelSelectList()
{

	ui.channelSelectListWidget->clear();

	int channelQty = receiverCaptures[0]->GetChannelQty();
	for (int channel = 0; channel < channelQty; channel++)
	{
		int row = channel / receiverCaptures[0]->receiverColumnQty;
		int column= channel % receiverCaptures[0]->receiverColumnQty;

		QString sLabel= QString("Row: %1 Col: %2").arg(row+1, 3).arg(column+1, 3);
	
		QListWidgetItem *listItem = new QListWidgetItem(sLabel, ui.channelSelectListWidget);
		listItem->setFlags(listItem->flags() | Qt::ItemIsUserCheckable); // set checkable flag
		listItem->setCheckState(Qt::Checked);
		ui.channelSelectListWidget->addItem(listItem);
	}
}



void AWLQtDemo::on_algoSelectComboBox_indexChanged(int newIndex)

{
	if (newIndex < 0) return;

	int receiverCount = receiverCaptures.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		uint16_t algoID = ui.algoSelectComboBox->itemData(newIndex).value<int>();
		receiverCaptures[receiverID]->SetAlgorithm(algoID);
	}
	PrepareAlgoParametersView();

}


void AWLQtDemo::PrepareAlgoParametersView()

{
	int currentAlgoID = 0;
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		currentAlgoID = receiverCaptures[0]->receiverStatus.currentAlgo;
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;

	int rowCount = algoParameters.size();

	// Make sure headers show up.  Sometimes Qt designer flips that attribute.

	ui.algoParametersTable->horizontalHeader()->setVisible(true);

	// Set the number of rows in the table depending on the number of parameters for the algo, 
	// based on info in INI file.
	ui.algoParametersTable->setRowCount(rowCount);

	// Set the column widths
	ui.algoParametersTable->horizontalHeader()->setSectionResizeMode(eParameterCheckColumn, QHeaderView::Fixed);
	ui.algoParametersTable->horizontalHeader()->setSectionResizeMode(eParameterDescriptionColumn, QHeaderView::Fixed);
	ui.algoParametersTable->horizontalHeader()->setSectionResizeMode(eParameterValueColumn, QHeaderView::Fixed);
	ui.algoParametersTable->horizontalHeader()->setSectionResizeMode(eParameterConfirmColumn, QHeaderView::Fixed);
		// Column 0 is ID/Checkboxmake it quite small 
	ui.algoParametersTable->setColumnWidth(eParameterCheckColumn, 45);
	// Column 1 is description, make it much larger than the default 
	ui.algoParametersTable->setColumnWidth(eParameterDescriptionColumn, 160);
	// Column 2 is value , make it slightly larger
	ui.algoParametersTable->setColumnWidth(eParameterValueColumn, 60);
	// Column 3 is confirmation , make it slightly larger
	ui.algoParametersTable->setColumnWidth(eParameterConfirmColumn, 60);
		
	// Put the contents in the table
	for (int row = 0; row < rowCount; row++) 
	{
		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *newItem = new QTableWidgetItem(algoParameters[row].sIndex.c_str());
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		newItem->setCheckState(Qt::Unchecked);
		ui.algoParametersTable->setItem(row, eParameterCheckColumn, newItem);

		// Column 1: Is Description .  Not editable text
		newItem = new QTableWidgetItem(algoParameters[row].sDescription.c_str());
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.algoParametersTable->setItem(row, eParameterDescriptionColumn, newItem);

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
		ui.algoParametersTable->setItem(row, eParameterValueColumn, newItem);
	
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
		ui.algoParametersTable->setItem(row, eParameterConfirmColumn, newItem);
	}
}

void AWLQtDemo::UpdateAlgoParametersView()

{
	uint16_t comboAlgoID = ui.algoSelectComboBox->currentData().value<int>();

	int currentAlgoID = 0;
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		currentAlgoID = receiverCaptures[0]->receiverStatus.currentAlgo;
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

#if 1
	if (comboAlgoID != currentAlgoID) {
		ui.algoSelectComboBox->setCurrentIndex(ui.algoSelectComboBox->findData(currentAlgoID));
		PrepareTrackerParametersView();
	}
#endif
	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;
	
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *checkItem = ui.algoParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *confirmItem = ui.algoParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		// Going from partially checked to another value means we got an update
		// Update the value text.
		switch (algoParameters[row].pendingUpdates) {
		case updateStatusPendingVisual:
		{
			// Update was received.  CheckState falls back to default.

			if (originalCheckState != Qt::Unchecked) checkItem->setCheckState(Qt::Unchecked);
			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);
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
			ui.algoParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);
			algoParameters[row].pendingUpdates = updateStatusUpToDate;
			break;

		}
		case updateStatusPendingUpdate:
		{
			// Update was received.  CheckState falls back to default.

			if (originalCheckState != Qt::PartiallyChecked) checkItem->setCheckState(Qt::PartiallyChecked);
			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);
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
			ui.algoParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);
			break;
		}

		default:
		{
			if (originalCheckState != Qt::Unchecked) checkItem->setCheckState(Qt::Unchecked);
			//			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);
			break;
		}
		}// end case
	} // for (int row
}

void AWLQtDemo::on_algoParametersSetPushButton_clicked()
{
	int currentAlgoID = 0;
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		currentAlgoID = receiverCaptures[0]->receiverStatus.currentAlgo;
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.algoParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.algoParametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.algoParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.algoParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

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
				float floatValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%f", &floatValue);
				parameterValue = * (uint32_t *) &floatValue;
			}

			receiverCaptures[0]->SetAlgoParameter(currentAlgoID, parameterAddress, parameterValue); 
		} // if checked
	} // for 
}

void AWLQtDemo::on_algoParametersGetPushButton_clicked()
{
	int currentAlgoID = 0;
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		currentAlgoID = receiverCaptures[0]->receiverStatus.currentAlgo;
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;

	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.algoParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.algoParametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.algoParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.algoParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			uint16_t parameterAddress = algoParameters[row].address;
			receiverCaptures[0]->QueryAlgoParameter(currentAlgoID,  parameterAddress); 
		} // if checked
	} // for 
}



void AWLQtDemo::PrepareGlobalParametersView()

{
	int currentAlgoID = 0; // Algo 0 is global parameters
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;
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
	int currentAlgoID = 0; // Algo 0 is global parameters
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;
	int rowCount = algoParameters.size();
	for (int row = 0; row < rowCount; row++)
	{

		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *checkItem = ui.globalParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *confirmItem = ui.globalParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		// Going from partially checked to another value means we got an update
		// Update the value text.
		switch (algoParameters[row].pendingUpdates) {
		case updateStatusPendingVisual:
		{
			// Update was received.  CheckState falls back to default.

			if (originalCheckState != Qt::Unchecked) checkItem->setCheckState(Qt::Unchecked);
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
			algoParameters[row].pendingUpdates = updateStatusUpToDate;
			break;

		}
		case updateStatusPendingUpdate:
		{
			// Update was received.  CheckState falls back to default.

			if (originalCheckState != Qt::PartiallyChecked) checkItem->setCheckState(Qt::PartiallyChecked);
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
			break;
		}

		default:
		{
			if (originalCheckState != Qt::Unchecked) checkItem->setCheckState(Qt::Unchecked);
			//			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);
			break;
		}
		}// end case
	} // for (int row
}

void AWLQtDemo::on_globalParametersSetPushButton_clicked()
{
	int currentAlgoID = 0; // Algo 0 is global parameters
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;

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
				float floatValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%f", &floatValue);
				parameterValue = * (uint32_t *) &floatValue;
			}

			receiverCaptures[0]->SetGlobalAlgoParameter(parameterAddress, parameterValue); 
		} // if checked
	} // for 
}

void AWLQtDemo::on_globalParametersGetPushButton_clicked()
{
	int currentAlgoID = 0; // Algo 0 is global parameters
	AlgorithmDescription *algoDescription = NULL;

	if (receiverCaptures[0])
	{
		algoDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersAlgos, currentAlgoID);
	}

	if (algoDescription == NULL) return;

	AlgorithmParameterVector algoParameters = algoDescription->parameters;

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

void AWLQtDemo::on_trackerSelectComboBox_indexChanged(int newIndex)

{
	if (newIndex < 0) return;

	int receiverCount = receiverCaptures.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		uint16_t trackerID = ui.trackerSelectComboBox->itemData(newIndex).value<int>();
		receiverCaptures[receiverID]->SetTracker(trackerID);
	}
	PrepareTrackerParametersView();

}

void AWLQtDemo::PrepareTrackerParametersView()

{
	int currentTrackerID = 0;
	AlgorithmDescription *trackerDescription = NULL;

	if (receiverCaptures[0])
	{
		currentTrackerID = receiverCaptures[0]->receiverStatus.currentTracker;
		trackerDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersTrackers, currentTrackerID);
	}

	if (trackerDescription == NULL) return;

	AlgorithmParameterVector trackerParameters = trackerDescription->parameters;
	int rowCount = trackerParameters.size();

	// Make sure headers show up.  Sometimes Qt designer flips that attribute.

	ui.trackerParametersTable->horizontalHeader()->setVisible(true);

	// Set the number of rows in the table depending on the number of parameters for the algo, 
	// based on info in INI file.
	ui.trackerParametersTable->setRowCount(rowCount);

	// Set the column widths
	ui.trackerParametersTable->horizontalHeader()->setSectionResizeMode(eParameterCheckColumn, QHeaderView::Fixed);
	ui.trackerParametersTable->horizontalHeader()->setSectionResizeMode(eParameterDescriptionColumn, QHeaderView::Fixed);
	ui.trackerParametersTable->horizontalHeader()->setSectionResizeMode(eParameterValueColumn, QHeaderView::Fixed);
	ui.trackerParametersTable->horizontalHeader()->setSectionResizeMode(eParameterConfirmColumn, QHeaderView::Fixed);
	// Column 0 is ID/Checkboxmake it quite small 
	ui.trackerParametersTable->setColumnWidth(eParameterCheckColumn, 45);
	// Column 1 is description, make it much larger than the default 
	ui.trackerParametersTable->setColumnWidth(eParameterDescriptionColumn, 160);
	// Column 2 is value , make it slightly larger
	ui.trackerParametersTable->setColumnWidth(eParameterValueColumn, 60);
	// Column 3 is confirmation , make it slightly larger
	ui.trackerParametersTable->setColumnWidth(eParameterConfirmColumn, 60);

	// Put the contents in the table
	for (int row = 0; row < rowCount; row++)
	{
		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *newItem = new QTableWidgetItem(trackerParameters[row].sIndex.c_str());
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		newItem->setCheckState(Qt::Unchecked);
		ui.trackerParametersTable->setItem(row, eParameterCheckColumn, newItem);

		// Column 1: Is Description .  Not editable text
		newItem = new QTableWidgetItem(trackerParameters[row].sDescription.c_str());
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.trackerParametersTable->setItem(row, eParameterDescriptionColumn, newItem);

		// Column 2: Is Value .  Editable text
		QString sValue;
		if (trackerParameters[row].paramType == eAlgoParamInt)
		{
			sValue.sprintf("%d", trackerParameters[row].intValue);
		}
		else
		{
			sValue.sprintf("%f", trackerParameters[row].floatValue);
		}

		newItem = new QTableWidgetItem(sValue);
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsEnabled);
		ui.trackerParametersTable->setItem(row, eParameterValueColumn, newItem);

		// Column 3: Is Confirmation .  Not editable text
		QString sValueConfirm;
		if (trackerParameters[row].paramType == eAlgoParamInt)
		{
			sValueConfirm.sprintf("%d", trackerParameters[row].intValue);
		}
		else
		{
			sValueConfirm.sprintf("%f", trackerParameters[row].floatValue);
		}
		newItem = new QTableWidgetItem(sValueConfirm);
		newItem->setFlags(Qt::ItemIsEnabled);
		ui.trackerParametersTable->setItem(row, eParameterConfirmColumn, newItem);
	}
}

void AWLQtDemo::UpdateTrackerParametersView()

{
	uint16_t comboTrackerID = ui.trackerSelectComboBox->currentData().value<int>();

	int currentTrackerID = 0;
	AlgorithmDescription *trackerDescription = NULL;

	if (receiverCaptures[0])
	{
		currentTrackerID = receiverCaptures[0]->receiverStatus.currentTracker;
		trackerDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersTrackers, currentTrackerID);
	}
#if 1
	if (comboTrackerID != currentTrackerID) {
		ui.trackerSelectComboBox->setCurrentIndex(ui.trackerSelectComboBox->findData(currentTrackerID));
		PrepareTrackerParametersView();
	}
#endif
	if (trackerDescription == NULL) return;

	AlgorithmParameterVector trackerParameters = trackerDescription->parameters;

	int rowCount = trackerParameters.size();
	for (int row = 0; row < rowCount; row++)
	{

		// Column 0 is "Select" row:  Editable checkbox.
		QTableWidgetItem *checkItem = ui.trackerParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *confirmItem = ui.trackerParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		// Going from partially checked to another value means we got an update
		// Update the value text.
		switch (trackerParameters[row].pendingUpdates) {
		case updateStatusPendingVisual:
		{
			// Update was received.  CheckState falls back to default.

			if (originalCheckState != Qt::Unchecked) checkItem->setCheckState(Qt::Unchecked);
			ui.trackerParametersTable->setItem(row, eParameterCheckColumn, checkItem);
			// Get the confirm value and format.
			QString sValueConfirm;
			if (trackerParameters[row].paramType == eAlgoParamInt)
			{
				sValueConfirm.sprintf("%d", trackerParameters[row].intValue);
			}
			else
			{
				sValueConfirm.sprintf("%f", trackerParameters[row].floatValue);
			}
			confirmItem->setText(sValueConfirm);
			ui.trackerParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);
			trackerParameters[row].pendingUpdates = updateStatusUpToDate;
			break;

		}
		case updateStatusPendingUpdate:
		{
			// Update was received.  CheckState falls back to default.

			if (originalCheckState != Qt::PartiallyChecked) checkItem->setCheckState(Qt::PartiallyChecked);
			ui.trackerParametersTable->setItem(row, eParameterCheckColumn, checkItem);
			// Get the confirm value and format.
			QString sValueConfirm;
			if (trackerParameters[row].paramType == eAlgoParamInt)
			{
				sValueConfirm.sprintf("%d", trackerParameters[row].intValue);
			}
			else
			{
				sValueConfirm.sprintf("%f", trackerParameters[row].floatValue);
			}
			confirmItem->setText(sValueConfirm);
			ui.trackerParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);
			break;
		}

		default:
		{
			if (originalCheckState != Qt::Unchecked) checkItem->setCheckState(Qt::Unchecked);
			//			ui.algoParametersTable->setItem(row, eParameterCheckColumn, checkItem);
			break;
		}
		}// end case
	} // for (int row
}

void AWLQtDemo::on_trackerParametersSetPushButton_clicked()
{
	int currentTrackerID = 0;
	AlgorithmDescription *trackerDescription = NULL;

	if (receiverCaptures[0])
	{
		currentTrackerID = receiverCaptures[0]->receiverStatus.currentTracker;
		trackerDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersTrackers, currentTrackerID);
	}

	if (trackerDescription == NULL) return;

	AlgorithmParameterVector trackerParameters = trackerDescription->parameters;

	int rowCount = trackerParameters.size();
	for (int row = 0; row < rowCount; row++)
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.trackerParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.trackerParametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.trackerParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.trackerParametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.trackerParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			// The value value is read.
			// Format depends on the cell type

			QString sValueText = valueItem->text();
			uint16_t parameterAddress = trackerParameters[row].address;
			uint32_t parameterValue = 0L;
			if (trackerParameters[row].paramType == eAlgoParamInt)
			{
				int intValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%d", &intValue);
				// Send to the parameter value as uint32_t
				parameterValue = (uint32_t)intValue;
			}
			else
			{
				float floatValue = 0;
				sscanf(sValueText.toStdString().c_str(), "%f", &floatValue);
				parameterValue = *(uint32_t *)&floatValue;
			}

			receiverCaptures[0]->SetTrackerParameter(currentTrackerID, parameterAddress, parameterValue);
		} // if checked
	} // for 
}

void AWLQtDemo::on_trackerParametersGetPushButton_clicked()
{
	int currentTrackerID = 0;
	AlgorithmDescription *trackerDescription = NULL;

	if (receiverCaptures[0])
	{
		currentTrackerID = receiverCaptures[0]->receiverStatus.currentTracker;
		trackerDescription = receiverCaptures[0]->FindAlgoDescriptionByID(receiverCaptures[0]->parametersTrackers, currentTrackerID);
	}

	if (trackerDescription == NULL) return;

	AlgorithmParameterVector trackerParameters = trackerDescription->parameters;

	int rowCount = trackerParameters.size();
	for (int row = 0; row < rowCount; row++)
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.trackerParametersTable->item(row, eParameterCheckColumn);
		QTableWidgetItem *valueItem = ui.trackerParametersTable->item(row, eParameterValueColumn);
		QTableWidgetItem *confirmItem = ui.trackerParametersTable->item(row, eParameterConfirmColumn);

		Qt::CheckState originalCheckState = checkItem->checkState();
		if (originalCheckState == Qt::Checked)
		{
			// Going from checked to PartiallyChecked, while update is going on.
			checkItem->setCheckState(Qt::PartiallyChecked);
			ui.trackerParametersTable->setItem(row, eParameterCheckColumn, checkItem);

			// The confirmation value is emptied
			confirmItem->setText("");
			ui.trackerParametersTable->setItem(row, eParameterConfirmColumn, confirmItem);

			uint16_t parameterAddress = trackerParameters[row].address;
			receiverCaptures[0]->QueryTrackerParameter(currentTrackerID, parameterAddress);
		} // if checked
	} // for 
}


void AWLQtDemo::on_view2DActionToggled()
{
	if (action2DButton->isChecked())
	{
		m2DScan->show();
		m2DScan->slotConfigChanged();  // Force redraw with the current video parameters
		action2DButton->setChecked(true);
	}
	else 
	{
		m2DScan->hide();
		action2DButton->setChecked(false);
	}

	ui.gridDisplayLayout->update();
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

	ui.gridDisplayLayout->update();
}

void AWLQtDemo::on_viewAScanViewActionToggled()
{
	//m2DScan->ShowAScan (actionAScanButton->isChecked()) ;
	scopeWindow->ShowAScan (actionAScanButton->isChecked()) ;
}

void AWLQtDemo::on_viewAboutActionTriggered()
{
	QMessageBox msgBox(this);
	msgBox.setWindowTitle("About");
	msgBox.setTextFormat(Qt::RichText);   //this is what makes the links clickable
	msgBox.setText("QtDemo 1.3.3<br><br>Copyright @ 2018 Phantom Intelligence inc.<br><br><a href='http://phantomintelligence.com'>http://phantomintelligence.com</a>");
	msgBox.exec();
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

	on_resizeActionToggled();
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

#ifdef USE_OPENCV_VIDEO
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
#endif

void AWLQtDemo::on_checkBoxAdvanceModeToggled()
{
  AWLSettings *settings = AWLSettings::GetGlobalSettings();
  FillFPGAList(settings);
}


void AWLQtDemo::on_pushButtonSelectAllAscan_clicked()
{
  ui.checkBox_1->setChecked(true);
  ui.checkBox_2->setChecked(true);
  ui.checkBox_3->setChecked(true);
  ui.checkBox_4->setChecked(true);
  ui.checkBox_5->setChecked(true);
  ui.checkBox_6->setChecked(true);
  ui.checkBox_7->setChecked(true);
  ui.checkBox_8->setChecked(true);
  ui.checkBox_9->setChecked(true);
  ui.checkBox_10->setChecked(true);
  ui.checkBox_11->setChecked(true);
  ui.checkBox_12->setChecked(true);
  ui.checkBox_13->setChecked(true);
  ui.checkBox_14->setChecked(true);
  ui.checkBox_15->setChecked(true);
  ui.checkBox_16->setChecked(true);

  scopeWindow->setChannelMask(0xFFFF);
}

void AWLQtDemo::on_pushButtonSelectNoneAscan_clicked()
{
  ui.checkBox_1->setChecked(false);
  ui.checkBox_2->setChecked(false);
  ui.checkBox_3->setChecked(false);
  ui.checkBox_4->setChecked(false);
  ui.checkBox_5->setChecked(false);
  ui.checkBox_6->setChecked(false);
  ui.checkBox_7->setChecked(false);
  ui.checkBox_8->setChecked(false);
  ui.checkBox_9->setChecked(false);
  ui.checkBox_10->setChecked(false);
  ui.checkBox_11->setChecked(false);
  ui.checkBox_12->setChecked(false);
  ui.checkBox_13->setChecked(false);
  ui.checkBox_14->setChecked(false);
  ui.checkBox_15->setChecked(false);
  ui.checkBox_16->setChecked(false);

  scopeWindow->setChannelMask(0);
}

void AWLQtDemo::on_checkBoxAscanSelToggled()
{
  uint32_t mask = 0;
  if (ui.checkBox_1->isChecked()) mask |= 1 << 0;
  if (ui.checkBox_2->isChecked()) mask |= 1 << 1;
  if (ui.checkBox_3->isChecked()) mask |= 1 << 2;
  if (ui.checkBox_4->isChecked()) mask |= 1 << 3;
  if (ui.checkBox_5->isChecked()) mask |= 1 << 4;
  if (ui.checkBox_6->isChecked()) mask |= 1 << 5;
  if (ui.checkBox_7->isChecked()) mask |= 1 << 6;
  if (ui.checkBox_8->isChecked()) mask |= 1 << 7;
  if (ui.checkBox_9->isChecked()) mask |= 1 << 8;
  if (ui.checkBox_10->isChecked()) mask |= 1 << 9;
  if (ui.checkBox_11->isChecked()) mask |= 1 << 10;
  if (ui.checkBox_12->isChecked()) mask |= 1 << 11;
  if (ui.checkBox_13->isChecked()) mask |= 1 << 12;
  if (ui.checkBox_14->isChecked()) mask |= 1 << 13;
  if (ui.checkBox_15->isChecked()) mask |= 1 << 14;
  if (ui.checkBox_16->isChecked()) mask |= 1 << 15;
  scopeWindow->setChannelMask(mask);
}

void AWLQtDemo::on_comboBoxMaxRange_indexChanged(int newIndex)
{
  if (!ui.checkBoxAutoScale->isChecked())
  {
    switch (newIndex)
    {
    case 0: scopeWindow->SetMaxRange(32767.0F); break;
    case 1: scopeWindow->SetMaxRange(32767.0F * 0.75F); break;
    case 2: scopeWindow->SetMaxRange(32767.0F * 0.50F); break;
    default:scopeWindow->SetMaxRange(32767.0F * 0.25F); break;
    }
  }
}

void AWLQtDemo::on_checkBoxAutoScaleToggled()
{
  if (ui.checkBoxAutoScale->isChecked())
  {
    ui.comboBoxMaxRange->setDisabled(true);
    scopeWindow->SetMaxRange(0.0F);
  }
  else
  {
    ui.comboBoxMaxRange->setDisabled(false);
    int sel = ui.comboBoxMaxRange->currentIndex();
    switch (sel)
    {
    case 0: scopeWindow->SetMaxRange(32767.0F); break;
    case 1: scopeWindow->SetMaxRange(32767.0F * 0.75F); break;
    case 2: scopeWindow->SetMaxRange(32767.0F * 0.50F); break;
    default:scopeWindow->SetMaxRange(32767.0F * 0.25F); break;
    }
  }
}

void AWLQtDemo::on_resizeActionToggled()
{
	if (actionResizeButton->isChecked()) 
	{
	
		showFullScreen();
		actionResizeButton->setIcon(*actionResizeRestoreDownIcon);
		actionResizeButton->setToolTip(actionResizeRestoreDownString);
		actionResizeButton->setChecked(true);
	}
	else 
	{
		showMaximized();
		actionResizeButton->setIcon(*actionResizeMaximizeIcon);
		actionResizeButton->setToolTip(actionResizeMaximizeString);
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
  bool bAdvancedModeChecked = ui.checkBoxAdvanceMode->isChecked();

	if (!receiverCaptures[0]->registersFPGALabel.empty())
		ui.registerFPGAGroupBox->setTitle(receiverCaptures[0]->registersFPGALabel.c_str());

  ui.registerFPGAAddressSetComboBox->clear();

	for (int i = 0; i < receiverCaptures[0]->registersFPGA.size(); i++) 
	{
    if (!bAdvancedModeChecked && receiverCaptures[0]->registersFPGA[i].bAdvanced)
      ; // Skip advanced register
    else
    {
      QString sLabel = receiverCaptures[0]->registersFPGA[i].sIndex.c_str();
      sLabel += ": ";
      sLabel += receiverCaptures[0]->registersFPGA[i].sDescription.c_str();
      ui.registerFPGAAddressSetComboBox->addItem(sLabel);
    }
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

void AWLQtDemo::on_registerFPGASaveToFlash_clicked()
{
  QMessageBox::StandardButton reply;

  reply = QMessageBox::question(this, "Warning", "Save current configuration to Flash. Are you sure?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes)
  {
    // Send the command to the device
    if (receiverCaptures[0])
    {
      receiverCaptures[0]->SetFPGARegister(0x3FFE, 0);
    }
  }
}

void AWLQtDemo::on_registerFPGARestoreFactoryDefaults_clicked()
{
  QMessageBox::StandardButton reply;

  reply = QMessageBox::question(this, "Warning", "Restore factory default configuration. Are you sure?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes)
  {
    // Send the command to the device
    if (receiverCaptures[0])
    {
      receiverCaptures[0]->SetFPGARegister(0x3FFF, 0);
    }
  }
}

void AWLQtDemo::on_registerADCSaveToFlash_clicked()
{
  QMessageBox::StandardButton reply;

  reply = QMessageBox::question(this, "Warning", "Save current configuration to Flash. Are you sure?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes)
  {
    // Send the command to the device
    if (receiverCaptures[0])
    {
      receiverCaptures[0]->SetADCRegister(0x3FFE, 0);
    }
  }
}

void AWLQtDemo::on_registerADCRestoreFactoryDefaults_clicked()
{
  QMessageBox::StandardButton reply;

  reply = QMessageBox::question(this, "Warning", "Restore factory default configuration. Are you sure?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes)
  {
    // Send the command to the device
    if (receiverCaptures[0])
    {
      receiverCaptures[0]->SetADCRegister(0x3FFF, 0);
    }
  }
}

void AWLQtDemo::FillADCList(AWLSettings *settingsPtr)
{
	if (!receiverCaptures[0]->registersADCLabel.empty())
		ui.registerADCGroupBox->setTitle(receiverCaptures[0]->registersADCLabel.c_str());

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
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates == updateStatusPendingUpdate)
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
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates  == updateStatusPendingUpdate)
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

		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates == updateStatusPendingVisual)
			receiverCaptures[0]->registersGPIO[i].pendingUpdates = updateStatusUpToDate;
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
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates == updateStatusPendingUpdate)
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
		if (receiverCaptures[0]->registersGPIO[i].pendingUpdates = updateStatusPendingUpdate)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);

	}
}

void AWLQtDemo::closeEvent(QCloseEvent * event)
{
#ifdef USE_OPENCV_VIDEO
	for (int cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}
#endif


	for (int receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	qApp->closeAllWindows();
}	

#if 0
void AWLQtDemo::DoThreadLoop()
{
	int dgg = 0;
	char str[256];
	int nbrPrevFrame = 0;
	int nbrPrevRaw = 0;

	while (true)
	{
		int nbrFrame = receiverCaptures[0]->m_nbrCompletedFrameCumul;
		int nbrRaw = receiverCaptures[0]->m_nbrRawCumul;

		sprintf(str, "frame: %5d (%3d) - Raw: %5d (%3d)\n", nbrFrame, nbrFrame - nbrPrevFrame, nbrRaw, nbrRaw - nbrPrevRaw);
		OutputDebugStringA(str);

		nbrPrevFrame = nbrFrame;
		nbrPrevRaw = nbrRaw;

		Sleep(1000);
	}
}
#endif
