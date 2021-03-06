/* AWLQtDemo.cpp : CuteApplication user interface management*/
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

#include <boost/foreach.hpp>


#include "AWLSettings.h"
#include "SensorCoord.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#ifdef USE_LIBUSB
#include "ReceiverLibUSBCapture.h"
#include "ReceiverLibUSB2Capture.h"
#endif

#include "ReceiverSimulatorCapture.h"
#include "ReceiverPostProcessor.h"

#include "DebugPrintf.h"
#include "awlqtdemo.h"
#ifdef WIN32
	#include <QTableWidget.h>
#endif
#include <QProcess>
#include <QThread>
#include <QTableWidget>
#include <QDesktopWidget>
#include <QApplication>
#include <QTime>
#include <QMessageBox>
#include <QListWidget>
#include <QStandardPaths>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <string>

#include "TableView.h"

using namespace awl;
SENSORCORE_USE_NAMESPACE

// Text update rate, in frame per seconds
#define LOOP_RATE	30

TransformationNode::Ptr myBaseNode;

const QString actionResizeMaximizeString("Maximize");
const QString actionResizeRestoreDownString("Restore down");

AWLQtDemo::AWLQtDemo(int argc, char *argv[])
	: QMainWindow(),
	m2DScan(NULL),
	mTableView(NULL),
	mAScanView(NULL),
	myTimer(NULL),
	actionSettingsButton(NULL),
	action2DButton(NULL),
	actionTableButton(NULL),
	actionAScanButton(NULL),
#if defined (USE_OPENCV_VIDEO)
	actionCameraButton(NULL),
#endif
	actionResizeButton(NULL),
	actionQuitButton(NULL),
	actionResizeMaximizeIcon(NULL),
	actionResizeRestoreDownIcon(NULL),
	m_bConnected(false)
{

	labelConnected = new QLabel("Initializing...");
	labelFramerate = new QLabel();

	QMessageBox msgBox(this);

	ui.setupUi(this);
	//setStyleSheet("background-color:rgb(30,64,86)");
	//setAutoFillBackground( true );

#ifdef _WINDOWS_

	// Set the thread priority for the application under Windows
	// Process priority has to be elevated to support real-time communications, 

    HANDLE proc = GetCurrentProcess();
	SetPriorityClass(proc, ABOVE_NORMAL_PRIORITY_CLASS);


	QThread::currentThread()->setPriority(QThread::LowestPriority);
#endif

	// Set the basic paths
//	QCoreApplication::setOrganizationName("Phantom Intelligence");
//	QCoreApplication::setApplicationName("Phantom Intelligence Lidar Demo");	

	// Read the settigs from the configuration file
	//
	// First, ask Qt for the existence of the file.  Add the "/" that Qt does not integrate.
	QString sSettingsPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);

#if 1 // Override the default windows directory.  Run in the 
	sSettingsPath = QString(".");
#endif

	// Append the last "/", which Qt does not do.
	sSettingsPath += "/";


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
	SensorCoordinates *globalCoordinates = SensorCoordinates::InitCoordinates();
	globalCoordinates->BuildCoordinatesFromSettings(globalSettings->GetPropTree());

	// Adjust the default displayed ranges depending on the sensors capabilities
	AdjustDefaultDisplayedRanges();

	// Create the receiver communication objects
	size_t receiverQty = globalSettings->receiverSettings.size();
	receiverCaptures.resize(receiverQty);
	for (size_t receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Create the LIDAR acquisition thread object, depending on the type identified in the config file


#ifdef USE_LIBUSB
		if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string( "LibUSB"))
		{
			// LibUSB Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverLibUSBCapture(receiverID, globalSettings->GetPropTree()));
		}
		else if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string("LibUSB2"))
		{
			// LibUSB2 Capture is used if defined in the ini file
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverLibUSB2Capture(receiverID, globalSettings->GetPropTree()));

		}
		else  // Includes the case if (globalSettings->receiverSettings[receiverID].sReceiverType == std::string("Simulator")), which is the default.
#endif
		{  
			// If the type is undefined, just use the dumb simulator, not using external device
			receiverCaptures[receiverID] = ReceiverCapture::Ptr(new ReceiverSimulatorCapture(receiverID, globalSettings->GetPropTree()));
		}

		receiverCaptureSubscriberIDs.push_back(receiverCaptures[receiverID]->Subscribe());
	}

	int videoCaptureQty = 0;
	int opencvCameraID =  0;

#if defined (USE_OPENCV_VIDEO)
	// Create the video capture objects
	videoCaptureQty = globalSettings->cameraSettings.size();
	for (int cameraID = 0; cameraID < videoCaptureQty; cameraID++)
	{
		QString cameraName(this->windowTitle() + " Camera");
		cameraName.append(QString().sprintf(" %02d", cameraID));

		printf("%s\n", globalSettings->cameraSettings[cameraID].sCameraAPI.c_str());
		
		videoCaptures.push_back(VideoCapture::Ptr(new VideoCapture(cameraID, argc, argv, globalSettings->GetPropTree())));
		// Create the video viewer to display the camera image
		// The video viewer feeds from the  videoCapture (for image) and from the receiver (for distance info)
		VideoViewer* viewer = new VideoViewer(cameraName.toStdString(), videoCaptures[opencvCameraID]);
		videoViewers.push_back(VideoViewer::Ptr(viewer));
		opencvCameraID++;
	}
#endif

	// Fill the parameters  tables from the settings
	FillFPGAList();
	FillADCList();
	FillGPIOList();

	// Prepare the parameters view
	PrepareAlgoParametersView();
	PrepareGlobalParametersView();

	// Initialize the controls from the settings in INI file

	ui.internalCalibrationGroupBox->setVisible(globalSettings->bCalibrationTabShowSensorCalib);

	RelativePosition sensorPosition = SensorCoordinates::GetReceiverPosition(0);
	ui.sensorHeightSpinBox->setValue(sensorPosition.position.bodyRelative.up);
	ui.sensorDepthSpinBox->setValue(sensorPosition.position.bodyRelative.forward);
	float measurementOffset;
	receiverCaptures[0]->GetMeasurementOffset(measurementOffset);
	ui.measurementOffsetSpinBox->setValue(measurementOffset);
	ui.sensorRangeMinSpinBox->setValue(globalSettings->receiverSettings[0].displayedRangeMin);

	ui.sensorRangeMaxSpinBox->setValue(globalSettings->receiverSettings[0].voxelsConfig[0].maxRange);

	FillVoxelSelectList();

	ui.targetHintDistanceSpinBox->setValue(receiverCaptures[0]->targetHintDistance);
	ui.targetHintAngleSpinBox->setValue(receiverCaptures[0]->targetHintAngle);

	// Default values, currently unused
	if (receiverCaptures[0]) 
	{
		ui.recordVoxel1CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel0);
		ui.recordVoxel2CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel1);
		ui.recordVoxel3CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel2);
		ui.recordVoxel4CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel3);
		ui.recordVoxel5CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel4);
		ui.recordVoxel6CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel5);
		ui.recordVoxel7CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel6);

		ui.calibrationVoxel1CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel0);
		ui.calibrationVoxel2CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel1);
		ui.calibrationVoxel3CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel2);
		ui.calibrationVoxel4CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel3);
		ui.calibrationVoxel5CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel4);
		ui.calibrationVoxel6CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel5);
		ui.calibrationVoxel7CheckBox->setChecked(receiverCaptures[0]->receiverStatus.voxelMask.bitFieldData.voxel6);

		ui.frameRateSpinBox->setValue((int) receiverCaptures[0]->GetDemandedFrameRate());
	}
	else
	{
		ui.recordVoxel1CheckBox->setChecked(true);
		ui.recordVoxel2CheckBox->setChecked(true);
		ui.recordVoxel3CheckBox->setChecked(true);
		ui.recordVoxel4CheckBox->setChecked(true);
		ui.recordVoxel5CheckBox->setChecked(true);
		ui.recordVoxel6CheckBox->setChecked(true);
		ui.recordVoxel7CheckBox->setChecked(true);


		ui.calibrationVoxel1CheckBox->setChecked(true);
		ui.calibrationVoxel2CheckBox->setChecked(true);
		ui.calibrationVoxel3CheckBox->setChecked(true);
		ui.calibrationVoxel4CheckBox->setChecked(true);
		ui.calibrationVoxel5CheckBox->setChecked(true);
		ui.calibrationVoxel6CheckBox->setChecked(true);
		ui.calibrationVoxel7CheckBox->setChecked(true);
		
		ui.frameRateSpinBox->setValue(0);
	}

	// Fill in the algo select combo box
	ui.algoSelectComboBox->clear();
	size_t algoQty = receiverCaptures[0]->parametersAlgos.algorithms.size();
	for (size_t i = 1; i < algoQty; i++)
	{
		QString algoLabel = QString(receiverCaptures[0]->parametersAlgos.algorithms[i].sAlgoName.c_str());
		uint16_t algoID = receiverCaptures[0]->parametersAlgos.algorithms[i].algoID;
		ui.algoSelectComboBox->addItem(algoLabel, QVariant(algoID));
	}

	int selectedAlgoID = receiverCaptures[0]->parametersAlgos.defaultAlgo;
	ui.algoSelectComboBox->setCurrentIndex(ui.algoSelectComboBox->findData(selectedAlgoID));

	// Fill in the tracker select combo box
	ui.trackerSelectComboBox->clear();
	size_t trackerQty = receiverCaptures[0]->parametersTrackers.algorithms.size();
	for (size_t i = 0; i < trackerQty; i++)
	{
		QString trackerLabel = QString(receiverCaptures[0]->parametersTrackers.algorithms[i].sAlgoName.c_str());
		uint16_t trackerID = receiverCaptures[0]->parametersTrackers.algorithms[i].algoID;
		ui.trackerSelectComboBox->addItem(trackerLabel, QVariant(trackerID));
	}

	int selectedTrackerID = receiverCaptures[0]->parametersTrackers.defaultAlgo;
	ui.trackerSelectComboBox->setCurrentIndex(ui.trackerSelectComboBox->findData(selectedTrackerID));

  // AdvancedMode
  connect(ui.checkBoxFPGAAdvanceMode, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxFPGAAdvanceModeToggled()));
  connect(ui.checkBoxADCAdvanceMode, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxADCAdvanceModeToggled()));

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

  connect(ui.radioButton_1, SIGNAL(toggled(bool)), this, SLOT(on_radioReceiverSelToggled()));
  connect(ui.radioButton_2, SIGNAL(toggled(bool)), this, SLOT(on_radioReceiverSelToggled()));
  connect(ui.radioButton_3, SIGNAL(toggled(bool)), this, SLOT(on_radioReceiverSelToggled()));
  connect(ui.radioButton_4, SIGNAL(toggled(bool)), this, SLOT(on_radioReceiverSelToggled()));
  connect(ui.radioButton_5, SIGNAL(toggled(bool)), this, SLOT(on_radioReceiverSelToggled()));
  connect(ui.radioButton_6, SIGNAL(toggled(bool)), this, SLOT(on_radioReceiverSelToggled()));


//  connect(ui.pushButtonSwap, SIGNAL(toggled(bool)), this, SLOT(on_ButtonSwapToggled()));

  ui.radioButton_1->hide();
  ui.radioButton_2->hide();
  ui.radioButton_3->hide();
  ui.radioButton_4->hide();
  ui.radioButton_5->hide();
  ui.radioButton_6->hide();

  if (receiverCaptures.size() > 0) ui.radioButton_1->show();
  if (receiverCaptures.size() > 1) ui.radioButton_2->show();
  if (receiverCaptures.size() > 2) ui.radioButton_3->show();
  if (receiverCaptures.size() > 3) ui.radioButton_4->show();
  if (receiverCaptures.size() > 4) ui.radioButton_5->show();
  if (receiverCaptures.size() > 5) ui.radioButton_6->show();

  
  ui.spinBoxReceiver1->setMinimum(1);
  ui.spinBoxReceiver1->setMaximum((int)receiverCaptures.size());
  ui.spinBoxReceiver2->setMinimum(1);
  ui.spinBoxReceiver2->setMaximum((int)receiverCaptures.size());
  ui.groupBox_5->hide();
  if (receiverCaptures.size() > 1) {
	bool show = true;
	for (size_t i = 0; i < receiverCaptures.size(); i++) {
		if (globalSettings->receiverSettings[i].sReceiverType != std::string( "LibUSB")) show = false;
	}
	if (show) {
		ui.groupBox_5->show();
		ui.spinBoxReceiver1->setValue(1);
		ui.spinBoxReceiver2->setValue(2);
	 }
  }


  // AutoScale
  connect(ui.checkBoxAutoScale, SIGNAL(toggled(bool)), this, SLOT(on_checkBoxAutoScaleToggled()));

  ui.comboBoxMaxRange->setDisabled(true);
  ui.comboBoxMaxRange->clear();
  ui.comboBoxMaxRange->addItem("100%", QVariant((float)1.00));
  ui.comboBoxMaxRange->addItem("75%", QVariant((float)0.75));
  ui.comboBoxMaxRange->addItem("50%", QVariant((float)0.50));
  ui.comboBoxMaxRange->addItem("25%", QVariant((float)0.25));

	// Calibration 
	ui.calibrationBetaDoubleSpinBox->setValue(0.8);
		

	// Initialize from other operating variables.
	ui.distanceLogFileCheckbox->setChecked(globalSettings->bWriteLogFile);
	ui.logFilePathLabel->setText(QString(globalSettings->GetLogAndDebugFilePath().c_str()));
	ui.logFileNameLabel->setText(QString(globalSettings->GetLogFileName().c_str()));

	// Initialize the AScan window
	mAScanView = new AWLPlotScan();

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
	connect(mAScanView, SIGNAL(closed( )), this, SLOT(on_viewAScanClose()));


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
	if (!globalSettings->bTabSettingAScan) {
                ui.interfaceTabs->removeTab(ui.interfaceTabs->indexOf(ui.tab));
        }

	if (ui.interfaceTabs->count() > 0) ui.interfaceTabs->setCurrentIndex(0);

	    on_view2DActionToggled();
        on_viewTableViewActionToggled();
        on_viewAScanViewActionToggled();
#if defined (USE_OPENCV_VIDEO)
        on_viewCameraActionToggled();
#endif
        on_viewSettingsActionToggled();


	// Position the main widget on the top left corner
	QRect scr = QApplication::desktop()->availableGeometry(QApplication::desktop()->primaryScreen());
	QRect frame = frameGeometry();
	QRect client = geometry();
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


#if defined (USE_OPENCV_VIDEO)
	// Start the threads for background video capture objects
	for (size_t cameraID = 0; cameraID < videoCaptures.size(); cameraID++)
	{
		videoCaptures[cameraID]->Go();
	}
#endif


	// Start the threads for background  receiver capture objects
	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		receiverCaptures[receiverID]->Go();
	}


	
	// Set size of statusbar & font
	ui.statusBar->setFixedHeight(30);
	ui.statusBar->setStyleSheet("QStatusBar{padding-left:20px;background:rgba(0,0,0,255);color:lightgray;font-weight:bold;font-size:20px;}");
	//ui.statusBar->showMessage(tr("Initialization ..."));

	labelConnected->setStyleSheet("QLabel{padding-left:20px;background:rgba(0,0,0,255);color:lightgray;font-weight:bold;font-size:20px;}");
	labelFramerate->setStyleSheet("QLabel{padding-left:20px;background:rgba(0,0,0,255);color:lightgray;font-weight:bold;font-size:20px;}");

	ui.statusBar->addWidget(labelConnected);
	ui.statusBar->addWidget(labelFramerate);

	// Just hide the frameRate control for now.
	ui.frameRateLabel->setVisible(false);
	ui.frameRateSpinBox->setVisible(false);

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
	// to reflect the maximum of all its voxel ranges.
	size_t receiverQty = globalSettings->receiverSettings.size();
	//long absoluteMaxRange = 0.0;
	for (size_t receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		long absoluteMaxRangeForReceiver = 0.0;
		int voxelQty = globalSettings->receiverSettings[receiverID].voxelsConfig.size();
		for (int voxelIndex = 0; voxelIndex < voxelQty; voxelIndex++)
		{
			if (globalSettings->receiverSettings[receiverID].voxelsConfig[voxelIndex].maxRange > absoluteMaxRangeForReceiver)
				absoluteMaxRangeForReceiver = globalSettings->receiverSettings[receiverID].voxelsConfig[voxelIndex].maxRange;
		}
		AWLSettings::GetGlobalSettings()->receiverSettings[receiverID].displayedRangeMax = absoluteMaxRangeForReceiver;

		//if (absoluteMaxRangeForReceiver > absoluteMaxRange) absoluteMaxRange = absoluteMaxRangeForReceiver; 
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

#if defined (USE_OPENCV_VIDEO)
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
#if defined (USE_OPENCV_VIDEO)
	connect(actionCameraButton, SIGNAL(toggled(bool )), this, SLOT(on_viewCameraActionToggled()));
#endif
	connect(actionSettingsButton, SIGNAL(toggled(bool )), this, SLOT(on_viewSettingsActionToggled()));
	connect(actionResizeButton, SIGNAL(toggled(bool )), this, SLOT(on_resizeActionToggled()));
	connect(actionQuitButton, SIGNAL(triggered(bool )), qApp, SLOT(closeAllWindows()));
}

void AWLQtDemo::SetupDisplayGrid()

{

	// Position the objects in the layout in order


	ui.gridDisplayLayout->addWidget(m2DScan, 0, 0, Qt::AlignTop);
	ui.gridDisplayLayout->addWidget(mTableView, 0, 1, Qt::AlignTop);
    ui.gridDisplayLayout->addWidget(mAScanView, 0, 3, Qt::AlignTop);

        int videoViewerQty = 0;
#if defined (USE_OPENCV_VIDEO)
        videoViewerQty = videoCaptures.size();
	for (int videoViewerID = 0; videoViewerID < videoViewerQty; videoViewerID++)
	{
		ui.gridDisplayLayout->addWidget(videoViewers[videoViewerID].get(), videoViewerID, 2, Qt::AlignTop);
	}
#endif
}

void AWLQtDemo::on_destroy()
{
#if defined (USE_OPENCV_VIDEO)
	for (size_t cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}
#endif

	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	if (m2DScan) delete m2DScan;
	if (mTableView) delete mTableView;	
	if (mAScanView) delete mAScanView;
}



void AWLQtDemo::on_recordPushButton_clicked()

{
	std::string sRecordFileName(ui.recordFileNameEdit->text().toStdString());
	ReceiverFrameRate frameRate = (ReceiverFrameRate) ui.frameRateSpinBox->value();
	VoxelMask voxelMask;

	voxelMask.bitFieldData.voxel0 = ui.recordVoxel1CheckBox->isChecked();
	voxelMask.bitFieldData.voxel1 = ui.recordVoxel2CheckBox->isChecked();
	voxelMask.bitFieldData.voxel2 = ui.recordVoxel3CheckBox->isChecked();
	voxelMask.bitFieldData.voxel3 = ui.recordVoxel4CheckBox->isChecked();
	voxelMask.bitFieldData.voxel4 = ui.recordVoxel5CheckBox->isChecked();
	voxelMask.bitFieldData.voxel5 = ui.recordVoxel6CheckBox->isChecked();
	voxelMask.bitFieldData.voxel6 = ui.recordVoxel7CheckBox->isChecked();
	voxelMask.bitFieldData.voxel7 = 1;

	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
	{
		receiverCaptures[0]->SetRecordFileName(sRecordFileName);
		receiverCaptures[0]->StartRecord(frameRate, voxelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}


void AWLQtDemo::on_playbackPushButton_clicked()

{
	std::string sPlaybackFileName(ui.playbackFileNameEdit->text().toStdString());
	ReceiverFrameRate frameRate = (ReceiverFrameRate)ui.frameRateSpinBox->value();
	VoxelMask voxelMask;

	voxelMask.bitFieldData.voxel0 = ui.recordVoxel1CheckBox->isChecked();
	voxelMask.bitFieldData.voxel1 = ui.recordVoxel2CheckBox->isChecked();
	voxelMask.bitFieldData.voxel2 = ui.recordVoxel3CheckBox->isChecked();
	voxelMask.bitFieldData.voxel3 = ui.recordVoxel4CheckBox->isChecked();
	voxelMask.bitFieldData.voxel4 = ui.recordVoxel5CheckBox->isChecked();
	voxelMask.bitFieldData.voxel5 = ui.recordVoxel6CheckBox->isChecked();
	voxelMask.bitFieldData.voxel6 = ui.recordVoxel7CheckBox->isChecked();
	voxelMask.bitFieldData.voxel7= 1;
	
	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected()) {
		receiverCaptures[0]->SetPlaybackFileName(sPlaybackFileName);
		receiverCaptures[0]->StartPlayback(frameRate, voxelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}


void AWLQtDemo::on_pushButtonSwap_clicked()
{	
	size_t r1, r2;
	void *h1, *h2;
	size_t cnt = receiverCaptures.size();
	r1 = ui.spinBoxReceiver1->value();
	r2 = ui.spinBoxReceiver2->value();
	//fprintf(stderr, "swap %d %d of %d\n", r1, r2, cnt);
	boost::shared_ptr<ReceiverPolledCapture> p1;
	boost::shared_ptr<ReceiverPolledCapture> p2;
	
	if (r1 > 0 && r2 > 0 && r1 <= cnt && r2 <= cnt && r1 != r2) {
		p1 = boost::reinterpret_pointer_cast<ReceiverPolledCapture>(receiverCaptures[r1 - 1]);
		p2 = boost::reinterpret_pointer_cast<ReceiverPolledCapture>(receiverCaptures[r2 - 1]);
		h1 = p1->GetHandle();
		h2 = p2->GetHandle();
		p1->SetHandle(h2);
		p2->SetHandle(h1);
		fprintf(stderr, "Swapping %p %p\n", h1, h2);
	}
}

void AWLQtDemo::on_stopPushButton_clicked()

{
	std::string sPlaybackFileName(ui.playbackFileNameEdit->text().toStdString());

	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
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
	float height = (float) ui.sensorHeightSpinBox->value();
	RelativePosition sensorPosition = SensorCoordinates::GetReceiverPosition(0);
	if (abs(height-sensorPosition.position.bodyRelative.up) < 0.001) return;
	sensorPosition.position.bodyRelative.up = height;

	SensorCoordinates::SetReceiverPosition(0, sensorPosition);

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
	float forward = (float) ui.sensorDepthSpinBox->value();
	RelativePosition sensorPosition = SensorCoordinates::GetReceiverPosition(0);
	if (abs(forward - sensorPosition.position.bodyRelative.forward) < 0.001) return;

	sensorPosition.position.bodyRelative.forward = forward;
	SensorCoordinates::SetReceiverPosition(0, sensorPosition);


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
	float range = ui.sensorRangeMinSpinBox->value();
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

void AWLQtDemo::ChangeRangeMax(int voxelID, float range)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	// Wait Cursor
	setCursor(Qt::WaitCursor);
	QApplication::processEvents();


	// Update the settings
	AWLSettings *settings = AWLSettings::GetGlobalSettings();
        size_t receiverQty = globalSettings->receiverSettings.size();

        for (size_t receiverID = 0; receiverID < receiverQty; receiverID++)
        {


		settings->receiverSettings[receiverID].voxelsConfig[voxelID].maxRange = range;
	
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
}

void AWLQtDemo::on_calibrationRangeMaxSpin_editingFinished()
{
	float range = (float) ui.sensorRangeMaxSpinBox->value();

	// Calculate the absolute max distance from the settings
	size_t voxelQty = AWLSettings::GetGlobalSettings()->receiverSettings[0].voxelsConfig.size();
	for (size_t voxelIndex = 0; voxelIndex < voxelQty; voxelIndex++)
	{
		QListWidgetItem *listItem = ui.voxelSelectListWidget->item(voxelIndex);
		Qt::CheckState checkState = listItem->checkState();
		if (checkState == Qt::Checked)
		{
			ChangeRangeMax(voxelIndex, range);
		}
	}
}


void AWLQtDemo::on_measurementOffsetSpin_editingFinished()
{
	float offset = (float) ui.measurementOffsetSpinBox->value();

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
	float   beta = (float) ui.calibrationBetaDoubleSpinBox->value();
	VoxelMask voxelMask;

	voxelMask.bitFieldData.voxel0 = ui.calibrationVoxel1CheckBox->isChecked();
	voxelMask.bitFieldData.voxel1 = ui.calibrationVoxel2CheckBox->isChecked();
	voxelMask.bitFieldData.voxel2 = ui.calibrationVoxel3CheckBox->isChecked();
	voxelMask.bitFieldData.voxel3 = ui.calibrationVoxel4CheckBox->isChecked();
	voxelMask.bitFieldData.voxel4 = ui.calibrationVoxel5CheckBox->isChecked();
	voxelMask.bitFieldData.voxel5 = ui.calibrationVoxel6CheckBox->isChecked();
	voxelMask.bitFieldData.voxel6 = ui.calibrationVoxel7CheckBox->isChecked();
	voxelMask.bitFieldData.voxel7 = 1;

	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
	{
		receiverCaptures[0]->StartCalibration(frameQty, beta, voxelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus(0);
}

void AWLQtDemo::on_targetHintDistanceSpin_editingFinished()
{
	float distance = (float) ui.targetHintDistanceSpinBox->value();

	receiverCaptures[0]->targetHintDistance = distance;
}

void AWLQtDemo::on_targetHintAngleSpin_editingFinished()
{
	float angle = (float) ui.targetHintAngleSpinBox->value();

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

void AWLQtDemo::on_logFileSelectorButton_pressed()
{
	//QCoreApplication::processEvents();
	ui.logFileSelectButton->setEnabled(false);

	QString dialogFileName = QFileDialog::getSaveFileName(this,
		tr("Set Log File Path"), 
		QString(AWLSettings::GetGlobalSettings()->GetLogAndDebugFilePath().c_str()), tr("Distance Log Files (*.csv)"));

	if (!dialogFileName.isEmpty())
	{
		QFileInfo fileInfo(dialogFileName);

		// Append the last "/", which Qt does not do.
		QString path = fileInfo.absolutePath() + "/";
		QString fileName = fileInfo.fileName();
		AWLSettings::GetGlobalSettings()->SetLogAndDebugFilePath(path.toStdString());
		AWLSettings::GetGlobalSettings()->SetLogFileName(fileName.toStdString());
		ui.logFilePathLabel->setText(path);
		ui.logFileNameLabel->setText(fileName);
	}

	ui.logFileSelectButton->setEnabled(true);
}

void AWLQtDemo::on_timerTimeout()
{
	myTimer->stop();

	bool bContinue = true;

#if defined (USE_OPENCV_VIDEO)
	// Check that the cameras are still working.  Otherwise Stop everyting
	for (size_t cameraID = 0; cameraID < videoCaptures.size(); cameraID++)
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
		for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
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

#if defined (USE_OPENCV_VIDEO)
		// Update the data for the camera views. Only if detections have changed have changed.
		for (size_t viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID] && bNewDetections) videoViewers[viewerID]->slotDetectionDataChanged(detectionData);
		}
#endif

		// Update the table views only if there is new data
		if (mTableView && bNewDetections) mTableView->slotDetectionDataChanged(detectionData);

		if (mAScanView && bNewDetections)
		{
			AScan::Vector aScanData;
			aScanData.clear();

			GetLatestAScans(aScanData);
			mAScanView->AScanDataChanged(aScanData);
		}
	}

	// Update receiver status
	bool bWasConnected = labelConnected->text().compare(QString("Connected")) == 0;
	bool bConnected = receiverCaptures[0]->IsConnected();
	ReceiverFrameRate frameRate = receiverCaptures[0]->GetCalculatedFrameRate();

	{
		QString str;
		if (bConnected) 
		{
			str = "Connected";
			if (!bWasConnected)
			{
				// Reupdate the lists
				FillFPGAList();
				FillADCList();
				DisplayReceiverStatus();
			}
		}
		else
			str = "Not connected";
		labelConnected->setText(str);

		if (bConnected)
			str.sprintf("Framerate: %3d Hz", (int) frameRate);
		else
			str = "";
		labelFramerate->setText(str);

		//ui.statusBar->showMessage(str);
	}



	if (bContinue)
	{
#if defined (USE_OPENCV_VIDEO)
		// Always spin the video viewers.
		for (size_t viewerID = 0; viewerID < videoViewers.size(); viewerID++)
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
	bool bNew = false;

	ReceiverPostProcessor postProcessor;

	// Build the list of detections that need to be updated
	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];

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
	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		ReceiverCapture::Ptr receiver = receiverCaptures[receiverID];
		// Use the frame snapped by the main display timer as the current frame
		Publisher::SubscriberID subscriberID = receiverCaptureSubscriberIDs[receiverID];
		FrameID lastDisplayedFrame = receiver->GetCurrentIssueID(subscriberID);
		bNew = receiver->CopyReceiverAScans(lastDisplayedFrame, aScanData, subscriberID);
	}

	return(bNew);
}

void AWLQtDemo::DisplayReceiverStatus()
{

	size_t receiverQty = receiverCaptures.size();
	 for (size_t receiverID = 0; receiverID < receiverQty; receiverID++) 
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
		receiverCaptures[receiverID]->CopyReceiverStatusData(status);
		
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

		ui.receiverVoxel1CheckBox->setChecked(status.receiverError.bitFieldData.voxel0);
		ui.receiverVoxel2CheckBox->setChecked(status.receiverError.bitFieldData.voxel1);
		ui.receiverVoxel3CheckBox->setChecked(status.receiverError.bitFieldData.voxel2);
		ui.receiverVoxel4CheckBox->setChecked(status.receiverError.bitFieldData.voxel3);
		ui.receiverVoxel5CheckBox->setChecked(status.receiverError.bitFieldData.voxel4);
		ui.receiverVoxel6CheckBox->setChecked(status.receiverError.bitFieldData.voxel5);
		ui.receiverVoxel7CheckBox->setChecked(status.receiverError.bitFieldData.voxel6);

		ui.statusSelfTestCheckBox->setChecked(status.status.bitFieldData.selfTest);
		ui.statusShutdownCheckBox->setChecked(status.status.bitFieldData.shutdown);
		ui.statusSensorBlockedCheckBox->setChecked(status.status.bitFieldData.sensorBlocked);
		ui.statusReducedPerformanceCheckBox->setChecked(status.status.bitFieldData.reducedPerformance);
		ui.statusSaturationCheckBox->setChecked(status.status.bitFieldData.saturation);

		// Registers

		formattedString.sprintf("%04X", status.fpgaRegisterAddressRead);
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

		ui.receiverVoxel1CheckBox->setEnabled(bEnableButtons);
		ui.receiverVoxel2CheckBox->setEnabled(bEnableButtons);
		ui.receiverVoxel3CheckBox->setEnabled(bEnableButtons);
		ui.receiverVoxel4CheckBox->setEnabled(bEnableButtons);
		ui.receiverVoxel5CheckBox->setEnabled(bEnableButtons);
		ui.receiverVoxel6CheckBox->setEnabled(bEnableButtons);
		ui.receiverVoxel7CheckBox->setEnabled(bEnableButtons);

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

void AWLQtDemo::FillVoxelSelectList()
{

	ui.voxelSelectListWidget->clear();

	int voxelQty = receiverCaptures[0]->GetVoxelQty();
	for (int voxelID = 0; voxelID < voxelQty; voxelID++)
	{
		int row = voxelID / receiverCaptures[0]->receiverColumnQty;
		int column= voxelID % receiverCaptures[0]->receiverColumnQty;

		QString sLabel= QString("Row: %1 Col: %2").arg(row, 3).arg(column, 3);
	
		QListWidgetItem *listItem = new QListWidgetItem(sLabel, ui.voxelSelectListWidget);
		listItem->setFlags(listItem->flags() | Qt::ItemIsUserCheckable); // set checkable flag
		listItem->setCheckState(Qt::Checked);
		ui.voxelSelectListWidget->addItem(listItem);
	}
}



void AWLQtDemo::on_algoSelectComboBox_indexChanged(int newIndex)

{
	if (newIndex < 0) return;

	size_t receiverCount = receiverCaptures.size();
	for (size_t receiverID = 0; receiverID < receiverCount; receiverID++)
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

	size_t rowCount = algoParameters.size();

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
	for (size_t row = 0; row < rowCount; row++) 
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
	
	size_t rowCount = algoParameters.size();
	for (size_t row = 0; row < rowCount; row++) 
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
	size_t rowCount = algoParameters.size();
	for (size_t row = 0; row < rowCount; row++) 
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
				intValue = sValueText.toInt(NULL);
				// Send to the parameter value as uint32_t
				parameterValue = (uint32_t) intValue;
			}
			else
			{
				float floatValue = 0;
				floatValue = sValueText.toFloat(NULL);
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

	size_t rowCount = algoParameters.size();
	for (size_t row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.algoParametersTable->item(row, eParameterCheckColumn);
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
	size_t rowCount = algoParameters.size();

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
	for (size_t row = 0; row < rowCount; row++) 
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
	size_t rowCount = algoParameters.size();
	for (size_t row = 0; row < rowCount; row++)
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

	size_t rowCount = algoParameters.size();
	for (size_t row = 0; row < rowCount; row++) 
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
				intValue = sValueText.toInt(NULL);

				// Send to the parameter value as uint32_t
				parameterValue = (uint32_t) intValue;
			}
			else
			{
				float floatValue = 0;
				floatValue = sValueText.toFloat(NULL);
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

	size_t rowCount = algoParameters.size();
	for (size_t row = 0; row < rowCount; row++) 
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.globalParametersTable->item(row, eParameterCheckColumn);
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

	size_t receiverCount = receiverCaptures.size();
	for (size_t receiverID = 0; receiverID < receiverCount; receiverID++)
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
	size_t rowCount = trackerParameters.size();

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
	for (size_t row = 0; row < rowCount; row++)
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

	size_t rowCount = trackerParameters.size();
	for (size_t row = 0; row < rowCount; row++)
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

	size_t rowCount = trackerParameters.size();
	for (size_t row = 0; row < rowCount; row++)
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
				intValue = sValueText.toInt(NULL);
				// Send to the parameter value as uint32_t
				parameterValue = (uint32_t)intValue;
			}
			else
			{
				float floatValue = 0;
				floatValue = sValueText.toFloat(NULL);
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

	size_t rowCount = trackerParameters.size();
	for (size_t row = 0; row < rowCount; row++)
	{

		// Column 0 is "Select" row:  Editable checkbox.
		// If the checkbox is checked, then we will set that parameter.
		QTableWidgetItem *checkItem = ui.trackerParametersTable->item(row, eParameterCheckColumn);
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
	if (actionAScanButton->isChecked())
	{
		mAScanView->ShowAScan(true);
		mAScanView->show();
	}
	else 
	{
		mAScanView->ShowAScan(false);
		mAScanView->hide();
	}

	ui.gridDisplayLayout->update();
}

void AWLQtDemo::on_viewAboutActionTriggered()
{
	QMessageBox msgBox(this);
	msgBox.setWindowTitle("About");
	msgBox.setTextFormat(Qt::RichText);   //this is what makes the links clickable

	// allocate a block of memory for the version info
	QString sApplicationVersion = QApplication::applicationVersion();

	QString aboutText = QString("<p>Cute Demo: LiDAR demo application<br>Version ") + QApplication::applicationVersion() + "</p>"+
		"<p>"
		"The CuteDemo Application and Kit Copyright(C) 2013-2019 Phantom Intelligence Inc.<br>"
		"<a href = \"https://www.phantomintelligence.com/\"style=\"color: gray;\">https://www.phantomintelligence.com/</a></p>"
		"<p>"
		"You may use, distribute and copy the this application, its libraries and accompanying source code under the terms of the GNU Lesser General Public License version 3 (LGPL3), "
		"which supplements GNU General Public License Version 3, appearing in document &quot;PHANTOM_LICENSE.LGPL3&quot; included in the packaging of this application.<br>"
		"Alternatively, these files may be used under the terms of the GNU General Public License  version 3, appearing in the file &quot;PHANTOM_LICENSE.GPL3&quot; included in the packaging of this application.</p>"
		"<p>"
		"This program also makes use of third party tools and libraries, which are used without alteration to their source code.</p>"
		"<p>"
		"The use of each of the individual librairies is subject to its own license terms. A copy of the licenses for each of these is available in the <br>"
		" &quot;Licences&quot; directory packaged with the application.</p>"
		"<p>"
		"QT portion of the application is developed under the QT Open Source License<br>"
		"(a LGPLV3 license).The Qt Toolkit is Copyright(C) 2017 The Qt Company Ltd.<br>"
		"QT License is in subdirectory &quot;Qt open Source&quot;.<br>"
		"<a href = \"https://www.qt.io/\"style=\"color: gray;\">https://www.qt.io/</a></p>"
		"<p>"
		"OpenCV license is in subdirectory &quot;OpenCV&quot;.OpenCV also makes use of FFmpeg, under a separate license also in subdirectory &quot;OpenCV&quot;.<br>"
		"<a href =\"https://opencv.org/\"style=\"color: gray;\">https://opencv.org/</a></p>"
		"<p>"
		"BOOST license is in subdirectory &quot;BOOST&quot;.<br>"
		"<a href = \"https://www.boost.org/\"style=\"color: gray;\";>https://www.boost.org/</a></p>"
		"< p >"
		"LibUSB license is provided in subdirectory &quot;LibUSB&quot;.<br>"
		"<a href = \"https://libusb.info/\"style=\"color: gray;\">https://libusb.info/</a></p>"
		"< p >"
		"Microsoft Windows Redistributables are subject to the license terms outlined in &quot;Microsoft Visual C++2015 Redistributable (x86) 14.23.27820&quot; </p>";

	msgBox.setText(aboutText);
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

	ui.gridDisplayLayout->update();
}


#if defined (USE_OPENCV_VIDEO)
void AWLQtDemo::on_viewCameraActionToggled()
{
	if (actionCameraButton->isChecked())
	{
#ifdef USE_OPENCV_VIDEO
		for (size_t viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) {
				videoViewers[viewerID]->show();
			}
		}
#endif

		actionCameraButton->setChecked(true);
	}
	else
	{
#ifdef USE_OPENCV_VIDEO
		for (size_t viewerID = 0; viewerID < videoViewers.size(); viewerID++)
		{
			if (videoViewers[viewerID]) 
			{
				videoViewers[viewerID]->hide();
			}
		}
#endif

		actionCameraButton->setChecked(false);
	}
}
#endif

void AWLQtDemo::on_checkBoxFPGAAdvanceModeToggled()
{
  FillFPGAList();
}

void AWLQtDemo::on_checkBoxADCAdvanceModeToggled()
{
	FillADCList();
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

  mAScanView->setVoxelMask(0xFFFF);
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

  mAScanView->setVoxelMask(0);
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
  mAScanView->setVoxelMask(mask);
}

void AWLQtDemo::on_radioReceiverSelToggled()
{
  int receiver = 0;
  if (ui.radioButton_1->isChecked()) receiver = 0;
  if (ui.radioButton_2->isChecked()) receiver = 1;
  if (ui.radioButton_3->isChecked()) receiver = 2;
  if (ui.radioButton_4->isChecked()) receiver = 3;
  if (ui.radioButton_5->isChecked()) receiver = 4;
  if (ui.radioButton_6->isChecked()) receiver = 5;
  mAScanView->selectReceiver(receiver);
}

void AWLQtDemo::on_comboBoxMaxRange_indexChanged(int newIndex)
{
  if (!ui.checkBoxAutoScale->isChecked())
  {
	  if (newIndex >= 0) {
		  float rangeScale = ui.comboBoxMaxRange->itemData(newIndex).value<float>();
		  mAScanView->SetMaxRange(32767.0F * rangeScale);
	  }
  }
}

void AWLQtDemo::on_checkBoxAutoScaleToggled()
{
  if (ui.checkBoxAutoScale->isChecked())
  {
    ui.comboBoxMaxRange->setDisabled(true);
	mAScanView->SetMaxRange(0.0F);
  }
  else
  {
    ui.comboBoxMaxRange->setDisabled(false);
    int sel = ui.comboBoxMaxRange->currentIndex();
	on_comboBoxMaxRange_indexChanged(sel);
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

void AWLQtDemo::on_viewAScanClose()
{
	ui.actionAScan->setChecked(false);
}


void AWLQtDemo::FillFPGAList()
{
	bool bAdvancedModeChecked = ui.checkBoxFPGAAdvanceMode->isChecked();

	if (!receiverCaptures[0]->registersFPGALabel.empty())
		ui.registerFPGAGroupBox->setTitle(receiverCaptures[0]->registersFPGALabel.c_str());

	ui.registerFPGAAddressSetComboBox->clear();

	for (size_t i = 0; i < receiverCaptures[0]->registersFPGA.size(); i++)
	{
		if (!bAdvancedModeChecked && receiverCaptures[0]->registersFPGA[i].bAdvanced)
			; // Skip advanced register
		else
		{
			QString sLabel = receiverCaptures[0]->registersFPGA[i].sIndex.c_str();
			sLabel += ": ";
			sLabel += receiverCaptures[0]->registersFPGA[i].sDescription.c_str();
			ui.registerFPGAAddressSetComboBox->addItem(sLabel, QVariant((int16_t)i));
		}
	}

	if (receiverCaptures[0]->registersFPGA.size() > 0) 
	{
		ui.registerFPGAAddressSetComboBox->setCurrentIndex(0);
		on_registerFPGAAddressSetComboBox_indexChanged(0);
	}
}

void AWLQtDemo::on_registerFPGAAddressSetComboBox_indexChanged(int)
{
	// On change of index, force a get of the current value, to force a refresh
	AWLQtDemo::on_registerFPGAGetPushButton_clicked();
}

void AWLQtDemo::on_registerFPGASetPushButton_clicked()
{
	uint16_t registerAddress;
	QString sValue;
	uint32_t registerValue;

	size_t comboIndex = ui.registerFPGAAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	// Index in combo box does not correspond to address in registerList, 
	// since some items in regisster list are not always displayed.
	// Trust the user data
	size_t registerIndex = ui.registerFPGAAddressSetComboBox->itemData(comboIndex).value<size_t>();
	if (registerIndex >= receiverCaptures[0]->registersFPGA.size()) return;

	registerAddress = receiverCaptures[0]->registersFPGA[registerIndex].address;

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
	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
	{
		receiverCaptures[0]->SetFPGARegister(registerAddress, registerValue);
	}


	// Force a get of the current value, to force a refresh
	AWLQtDemo::on_registerFPGAGetPushButton_clicked();

}

void AWLQtDemo::on_registerFPGAGetPushButton_clicked()
{
	uint16_t registerAddress;


	size_t comboIndex = ui.registerFPGAAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;
	// Index in combo box does not correspond to address in registerList, 
// since some items in regisster list are not always displayed.
// Trust the user data
	size_t registerIndex = ui.registerFPGAAddressSetComboBox->itemData(comboIndex).value<size_t>();
	if (registerIndex >= receiverCaptures[0]->registersFPGA.size()) return;

	registerAddress = receiverCaptures[0]->registersFPGA[registerIndex].address;

	// Now update user interface
	ui.registerFPGAAddressGetLineEdit->setText("");
	ui.registerFPGAValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected()) 
	{
		receiverCaptures[0]->QueryFPGARegister(registerAddress);
	}
}

void AWLQtDemo::on_registerSaveToFlashPushButton_clicked()
{
  QMessageBox::StandardButton reply;

  reply = QMessageBox::question(this, "Warning", "Save current configuration to Flash. Are you sure?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes)
  {
    // Send the command to the device
    if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
    {
      receiverCaptures[0]->SetADCRegister(0xFE, 0);
    }
  }
}

void AWLQtDemo::on_registerRestoreFactoryDefaultsPushButton_clicked()
{
  QMessageBox::StandardButton reply;

  reply = QMessageBox::question(this, "Warning", "Restore factory default configuration. Are you sure?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes)
  {
    // Send the command to the device
    if (receiverCaptures[0])
    {
      receiverCaptures[0]->SetADCRegister(0xFF, 0);
    }
  }

  // Force a get of the current values, to force a refresh
  AWLQtDemo::on_registerADCGetPushButton_clicked();
  AWLQtDemo::on_registerFPGAGetPushButton_clicked();
}

void AWLQtDemo::FillADCList()
{
	bool bAdvancedModeChecked = ui.checkBoxADCAdvanceMode->isChecked();

	if (!receiverCaptures[0]->registersADCLabel.empty())
		ui.registerADCGroupBox->setTitle(receiverCaptures[0]->registersADCLabel.c_str());

	ui.registerADCAddressSetComboBox->clear();

	for (size_t i = 0; i < receiverCaptures[0]->registersADC.size(); i++)
	{
		if (!bAdvancedModeChecked && receiverCaptures[0]->registersADC[i].bAdvanced)
			; // Skip advanced register
		else
		{
			QString sLabel = receiverCaptures[0]->registersADC[i].sIndex.c_str();
			sLabel += ": ";
			sLabel += receiverCaptures[0]->registersADC[i].sDescription.c_str();
			ui.registerADCAddressSetComboBox->addItem(sLabel, QVariant((int16_t)i));
		}
	}

	if (receiverCaptures[0]->registersADC.size() > 0)
	{
		ui.registerADCAddressSetComboBox->setCurrentIndex(0);
		on_registerADCAddressSetComboBox_indexChanged(0);
	}
}

void AWLQtDemo::on_registerADCAddressSetComboBox_indexChanged(int)
{
	// On change of index, force a get of the current value, to force a refresh
	AWLQtDemo::on_registerADCGetPushButton_clicked();
}

void AWLQtDemo::on_registerADCSetPushButton_clicked()
{
	uint16_t registerAddress;
	QString sValue;
	uint32_t registerValue;

	size_t comboIndex = ui.registerADCAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;
	// Index in combo box does not correspond to address in registerList, 
    // since some items in register list are not always displayed.
    // Trust the user data
	size_t registerIndex = ui.registerADCAddressSetComboBox->itemData(comboIndex).value<size_t>();
	if (registerIndex >= receiverCaptures[0]->registersADC.size()) return;

	registerAddress = receiverCaptures[0]->registersADC[registerIndex].address;

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
	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
	{
		receiverCaptures[0]->SetADCRegister(registerAddress, registerValue);
	}

	// Force a get of the current value, to force a refresh
	AWLQtDemo::on_registerADCGetPushButton_clicked();

}

void AWLQtDemo::on_registerADCGetPushButton_clicked()
{
	uint16_t registerAddress;


	size_t comboIndex = ui.registerADCAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;
	// Index in combo box does not correspond to address in registerList, 
	// since some items in register list are not always displayed.
	// Trust the user data
	size_t registerIndex = ui.registerADCAddressSetComboBox->itemData(comboIndex).value<size_t>();
	if (registerIndex >= receiverCaptures[0]->registersADC.size()) return;

	registerAddress = receiverCaptures[0]->registersADC[registerIndex].address;

	// Now update user interface
	ui.registerADCAddressGetLineEdit->setText("");
	ui.registerADCValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
	{
		receiverCaptures[0]->QueryADCRegister(registerAddress);
	}
}


void AWLQtDemo::FillGPIOList()
{
	for (size_t i = 0; i < receiverCaptures[0]->registersGPIO.size(); i++) 
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
	size_t comboSize = (size_t) ui.registerGPIOListWidget->count();
	for (size_t comboIndex = 0; comboIndex < comboSize; comboIndex++) 
	{
		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(comboIndex);
		size_t registerIndex = comboIndex;

		QString sLabel = receiverCaptures[0]->registersGPIO[registerIndex].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[registerIndex].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[registerIndex].pendingUpdates  == updateStatusPendingUpdate)
		{
			sLabel += " -- UPDATING...";
		}
	
		if (receiverCaptures[0]->registersGPIO[registerIndex].value) 
		{
			listItem->setCheckState(Qt::Checked);
		}
		else 
		{
			listItem->setCheckState(Qt::Unchecked);
		}

		if (receiverCaptures[0]->registersGPIO[registerIndex].pendingUpdates == updateStatusPendingVisual)
			receiverCaptures[0]->registersGPIO[registerIndex].pendingUpdates = updateStatusUpToDate;
	}
}

void AWLQtDemo::on_registerGPIOSetPushButton_clicked()
{
	// Update all of the MIOs at the same time
	size_t comboSize = (size_t) ui.registerGPIOListWidget->count();
	for (size_t comboIndex = 0; comboIndex < comboSize; comboIndex++)
	{
		size_t registerIndex = comboIndex;
		uint16_t registerAddress = receiverCaptures[0]->registersGPIO[registerIndex].address;
		uint32_t registerValue = 0;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(comboIndex);
		Qt::CheckState checkState = listItem->checkState();
		if (checkState == Qt::Checked) 
		{
			registerValue = 1;
		}


		// Send the command to the device
		if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
		{
			receiverCaptures[0]->SetGPIORegister(registerAddress, registerValue);
		}

		// Update the user interface
		QString sLabel = receiverCaptures[0]->registersGPIO[registerIndex].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[registerIndex].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[registerIndex].pendingUpdates == updateStatusPendingUpdate)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);
	}// For

	  // Force a get of the current value, to force a refresh
	AWLQtDemo::on_registerGPIOGetPushButton_clicked();
}


void AWLQtDemo::on_registerGPIOGetPushButton_clicked()
{
	// Update all of the MIOs at the same time
	size_t comboSize = (size_t) ui.registerGPIOListWidget->count();
	for (size_t comboIndex = 0; comboIndex < comboSize; comboIndex++)
	{
		size_t registerIndex = comboIndex;
		uint16_t registerAddress = receiverCaptures[0]->registersGPIO[registerIndex].address;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(comboIndex);
	
		// Send the command to the device
		if (receiverCaptures[0] && receiverCaptures[0]->IsConnected())
		{
			receiverCaptures[0]->QueryGPIORegister(registerAddress);		
		}

		// Update the user interface
		QString sLabel = receiverCaptures[0]->registersGPIO[registerIndex].sIndex.c_str();
		sLabel += ": ";
		sLabel += receiverCaptures[0]->registersGPIO[registerIndex].sDescription.c_str();
		if (receiverCaptures[0]->registersGPIO[registerIndex].pendingUpdates = updateStatusPendingUpdate)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);

	}
}

void AWLQtDemo::closeEvent(QCloseEvent * /*event*/)
{
#if defined (USE_OPENCV_VIDEO)
	for (size_t cameraID = 0; cameraID < videoCaptures.size(); cameraID++) 
	{
		if (videoCaptures[cameraID]) videoCaptures[cameraID]->Stop();
	}
#endif

	for (size_t receiverID = 0; receiverID < receiverCaptures.size(); receiverID++)
	{
		if (receiverCaptures[receiverID]) receiverCaptures[receiverID]->Stop();
	}

	qApp->closeAllWindows();
}	

