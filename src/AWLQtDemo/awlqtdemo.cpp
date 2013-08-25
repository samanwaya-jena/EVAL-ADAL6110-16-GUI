#include "awlqtdemo.h"

#include <QTableWidget>
#include <QDesktopWidget>
#include <QApplication>
#include <QTime>
#include <QSettings>
#include <QMessageBox>
#include <QListWidget>


#include <string>
#include "boost\pointer_cast.hpp"
#include "Tracker.h"
#include "ReceiverCapture.h"
#include "ReceiverFileCapture.h"
#include "ReceiverCANCapture.h"
#include "ReceiverBareMetalCapture.h"
#include "FusedCloudViewer.h"
#include "DebugPrintf.h"
#include "AWLSettings.h"
#include "DetectionStruct.h"

#include "boost\pointer_cast.hpp"
#include "..\awlqtscope\awlqtscope.h"
#include "..\awlqtscope\curvedata.h"
#include "..\awlqtscope\signaldata.h"

using namespace std;
using namespace awl;
using namespace pcl;



// Frame rate, in frame per seconds
#define FRAME_RATE	33.0

// Text update rate, in frame per seconds
#if 0
#define LOOP_RATE	30
#else
#define LOOP_RATE	20
#endif
const int channelQty = 7;
const int  detectionsPerChannel =  8;



AWLQtDemo::AWLQtDemo(int argc, char *argv[])
	: QMainWindow()
{
	ui.setupUi(this);
	AWLSettings *globalSettings = AWLSettings::InitSettings();
	globalSettings->ReadSettings();

	FillFPGAList(globalSettings);
	FillADCList(globalSettings);
	FillGPIOList(globalSettings);


	// Position the widget on the bottonm left corner
	QRect scr = QApplication::desktop()->screenGeometry();
	int bottom = scr.bottom() - (this->frameSize().height())-10;
	move(scr.left(), bottom); 

	// In demo mode, put demo mode in window title
	if (globalSettings->bEnableDemo)
	{
		this->setWindowTitle(this->windowTitle()+" [DEMO Mode]");
	}

	PrepareTableViews();

	PrepareParametersView();
	PrepareGlobalParametersView();

	// Create the image acquistion thread object
	videoCapture = VideoCapture::Ptr(new VideoCapture(argc, argv));
	// Create the LIDAR acquisition thread object
	if (!globalSettings->sReceiverType.compare("BareMetal", Qt::CaseInsensitive))
	{
		receiverCapture = ReceiverCapture::Ptr((ReceiverCapture *) new ReceiverBareMetalCapture(0, channelQty, detectionsPerChannel, argc, argv));
	}
	else 
	{
		// CAN Capture is used if defined in the ini file, and by default
		receiverCapture = ReceiverCapture::Ptr((ReceiverCapture *) new ReceiverCANCapture(0, channelQty, detectionsPerChannel, argc, argv));
	}

	receiverCaptureSubscriberID = receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();
	// Create a common point-cloud object that will be "projected" upon
	baseCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Create the ReceiverProjector.
	// The projector feeds from the videoCapture and feeds from the base cloud
	receiver = ReceiverProjector::Ptr(new ReceiverProjector(videoCapture, baseCloud, receiverCapture));

	// Add the channels to the projector
	ReceiverChannel::Ptr channelPtr;

	// Short-range sensors
	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(0, (float) DEG2RAD(9.0), (float) DEG2RAD(9.0), (float) DEG2RAD(-15.0), (float) DEG2RAD(-7.8), (float) 10.0, "ChannelMask1.bmp", "ChannelFrame1.bmp", false, 128.0/255.0, 192.0/255.0, 128.0/255.0)));

	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(1, (float) DEG2RAD(9.0), (float) DEG2RAD(9.0), (float) DEG2RAD(-5.0), (float) DEG2RAD(-7.8), (float) 10.0, "ChannelMask2.bmp", "ChannelFrame2.bmp", false, 128.0/255.0, 192.0/255.0, 128.0/255.0)));

	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(2, (float) DEG2RAD(9.0), (float) DEG2RAD(9.0), (float) DEG2RAD(5.0), (float) DEG2RAD(-7.8), (float) 10.0, "ChannelMask3.bmp", "ChannelFrame3.bmp", false, 128.0/255.0, 192.0/255.0, 128.0/255.0)));

	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(3, (float) DEG2RAD(9.0), (float) DEG2RAD(9.0), (float) DEG2RAD(15.0), (float) DEG2RAD(-7.8), (float) 10.0,"ChannelMask4.bmp", "ChannelFrame4.bmp", false, 128.0/255.0, 192.0/255.0, 128.0/255.0)));

		// Long-Range sensors
	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(4, (float) DEG2RAD(4.3), (float) DEG2RAD(4.3), (float) DEG2RAD(-4.6), (float) DEG2RAD(-2.15), (float) 30.0,  "ChannelMask5.bmp", "ChannelFrame5.bmp", false, 143.0/255.0, 163.0/255.0, 190.0/255.0)));

	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(5, (float) DEG2RAD(4.3), (float) DEG2RAD(4.3), (float) DEG2RAD(0.0), (float) DEG2RAD(-2.15), (float) 30.0, "ChannelMask6.bmp", "ChannelFrame6.bmp", false, 143.0/255.0, 163.0/255.0, 190.0/255.0)));

	channelPtr = receiver->AddChannel(ReceiverChannel::Ptr(new ReceiverChannel(6, (float) DEG2RAD(4.3), (float) DEG2RAD(4.3), (float) DEG2RAD(4.6), (float) DEG2RAD(-2.15), (float) 30.0, "ChannelMask7.bmp", "ChannelFrame7.bmp", false, 143.0/255.0, 163.0/255.0, 190.0/255.0)));


	// Create the video viewer to display the camera image
	// The video viewer feeds from the  videoCapture (for image) and from the receiver (for distance info)
	videoViewer = VideoViewer::Ptr(new VideoViewer(this->windowTitle().toStdString(), videoCapture, receiverCapture, receiver));

	//  Create the fused viewer, that will instantiate all the point-cloud views.
	// All point cloud updates feed from the receiver's point-cloud data.
	// The fused Viewer also uses the receiver configuration info to build the background decorations  
	// used in point-cloud
	fusedCloudViewer = FusedCloudViewer::Ptr(new FusedCloudViewer(this->windowTitle().toStdString(), receiver));

	// Initialize the controls from the settings in INI file
	ui.sensorHeightSpinBox->setValue(globalSettings->sensorHeight);
	ui.sensorDepthSpinBox->setValue(globalSettings->sensorDepth);
	ui.measurementOffsetSpinBox->setValue(globalSettings->rangeOffset);
	ui.sensorRangeMinSpinBox->setValue(globalSettings->displayedRangeMin);
	ui.sensorRangeMaxSpinBox->setValue(globalSettings->displayedRangeMax);

	ui.pixelSizeSpinBox->setValue(globalSettings->pixelSize);	
	ui.decimationSpinBox->setValue(globalSettings->decimation);
	
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
	if (receiverCapture) 
	{
		ui.injectSimulatedCheckbox->setChecked(receiverCapture->IsSimulatedDataEnabled());
	}

	 scopeWindow = new AWLQtScope();
	 //scopeWindow->show();

	// Initialize the 2D view
	m2DScan = new FOV_2DScan();
	m2DScan->setWindowTitle(this->windowTitle());

	mCfgSensor.shortRangeDistance = globalSettings->shortRangeDistance;
    mCfgSensor.shortRangeDistanceStartLimited = globalSettings->shortRangeDistanceStartLimited;
    mCfgSensor.shortRangeAngle = globalSettings->shortRangeAngle;
    mCfgSensor.shortRangeAngleStartLimited = globalSettings->shortRangeAngleStartLimited;

    mCfgSensor.longRangeDistance = globalSettings->longRangeDistance;
    mCfgSensor.longRangeDistanceStartLimited = globalSettings->longRangeDistanceStartLimited;
    mCfgSensor.longRangeAngle = globalSettings->longRangeAngle;
    mCfgSensor.longRangeAngleStartLimited = globalSettings->longRangeAngleStartLimited;

    mCfgSensor.sensorDepth = 0 - globalSettings->sensorDepth;
    mCfgSensor.sensorHeight = globalSettings->sensorHeight;

	m2DScan->slotConfigChanged(&mCfgSensor);

	// Menu items signals and slots
	connect(ui.action2D, SIGNAL(toggled(bool )), this, SLOT(on_view2DActionToggled()));
	connect(ui.actionCamera, SIGNAL(toggled(bool )), this, SLOT(on_viewCameraActionToggled()));
	connect(ui.actionGraph, SIGNAL(toggled(bool )), this, SLOT(on_viewGraphActionToggled()));
	connect(ui.action3D_View, SIGNAL(toggled(bool )), this, SLOT(on_view3DActionToggled()));

	connect(ui.actionQuitter, SIGNAL(triggered(bool )), qApp, SLOT(closeAllWindows()));
	// View signals and slots on close
	connect(m2DScan, SIGNAL(closed()), this, SLOT(on_view2DClose()));
	connect(scopeWindow, SIGNAL(closed( )), this, SLOT(on_viewGraphClose()));


	// Start the threads for background capture objects
	videoCapture->Go();
	receiverCapture->Go(true);
	receiver->Go();

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
	}

	if (globalSettings->defaultAlgo == 1)
	{
		ui.algo1RadioButton->setChecked(true);
	}
	else if (globalSettings->defaultAlgo == 2)
	{
		ui.algo2RadioButton->setChecked(true);
	}
	else if (globalSettings->defaultAlgo == 3)
	{
		ui.algo2RadioButton->setChecked(true);
	}
	else  // Default
	{
		ui.algo2RadioButton->setChecked(true);
	}


	// In demo mode, automatically force the injection of data on receiver.
	// put demo mode in window title
	if (globalSettings->bEnableDemo)
	{
		ui.injectSimulatedCheckbox->setChecked(true);
		m2DScan->setWindowTitle(this->windowTitle());
	}
}

AWLQtDemo::~AWLQtDemo()
{
}

void AWLQtDemo::on_destroy()
{
	if (fusedCloudViewer) fusedCloudViewer->Stop();
	if (videoCapture) videoCapture->Stop();
	if (receiverCapture) receiverCapture->Stop();
	if (receiver) receiver->Stop();
	if (videoViewer) videoViewer->Stop();

	if (m2DScan) delete m2DScan;
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
	if (receiverCapture)
	{
		receiverCapture->EnableSimulationData(bChecked);
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
	ReceiverCapture::ChannelMask channelMask;

	channelMask.bitFieldData.channel0 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel1 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel2 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel3 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel4 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel5 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel6 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.unused = 0;

	if (receiverCapture) 
	{
		receiverCapture->SetRecordFileName(sRecordFileName);
		receiverCapture->StartRecord(frameRate, channelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus();
}


void AWLQtDemo::on_playbackPushButton_clicked()

{
	std::string sPlaybackFileName(ui.playbackFileNameEdit->text().toStdString());
	uint8_t frameRate = ui.frameRateSpinBox->value();
	ReceiverCapture::ChannelMask channelMask;

	channelMask.bitFieldData.channel0 = ui.channel1CheckBox->isChecked();
	channelMask.bitFieldData.channel1 = ui.channel2CheckBox->isChecked();
	channelMask.bitFieldData.channel2 = ui.channel3CheckBox->isChecked();
	channelMask.bitFieldData.channel3 = ui.channel4CheckBox->isChecked();
	channelMask.bitFieldData.channel4 = ui.channel5CheckBox->isChecked();
	channelMask.bitFieldData.channel5 = ui.channel6CheckBox->isChecked();
	channelMask.bitFieldData.channel6 = ui.channel7CheckBox->isChecked();
	channelMask.bitFieldData.unused = 0;
	
	if (receiverCapture) 
	{
		receiverCapture->SetPlaybackFileName(sPlaybackFileName);
		receiverCapture->StartPlayback(frameRate, channelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus();
}


void AWLQtDemo::on_stopPushButton_clicked()

{
	std::string sPlaybackFileName(ui.playbackFileNameEdit->text().toStdString());

	if (receiverCapture) 
	{
		if (receiverCapture->receiverStatus.bInPlayback) 
		{
			receiverCapture->StopPlayback();
		}
		else if (receiverCapture->receiverStatus.bInRecord) 
		{
			receiverCapture->StopRecord();
		}
	}

	// Update the state of buttons
	DisplayReceiverStatus();
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

	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1)
		{
		
			fusedCloudViewer->viewers[0]->SetSensorHeight(height);

		}
	}

	if (m2DScan && !m2DScan->isHidden())
	{
	    mCfgSensor.sensorHeight = height;
		m2DScan->slotConfigChanged(&mCfgSensor);
	}
}


void AWLQtDemo::on_sensorDepthSpin_editingFinished()
{
	double depth = ui.sensorDepthSpinBox->value();

	if (fusedCloudViewer) 
	{
		if (fusedCloudViewer->viewers.size() >= 1)
		{
		
			fusedCloudViewer->viewers[0]->SetSensorDepth(depth);
		}
	}

	if (m2DScan && !m2DScan->isHidden())
	{
	    mCfgSensor.sensorDepth = 0 - depth;
		m2DScan->slotConfigChanged(&mCfgSensor);
	}
}


void AWLQtDemo::on_measurementOffsetSpin_editingFinished()
{
	if (receiverCapture) 
	{
		double offset = ui.measurementOffsetSpinBox->value();
		receiverCapture->SetMeasurementOffset(offset);
	}
}


void AWLQtDemo::on_calibratePushButton_clicked()

{
	uint8_t frameQty = ui.calibrationFrameQtySpinBox->value();
	float   beta = ui.calibrationBetaDoubleSpinBox->value();
	ReceiverCapture::ChannelMask channelMask;

	channelMask.bitFieldData.channel0 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel1 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel2 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel3 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel4 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel5 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.channel6 = ui.calibrationChannel1CheckBox->isChecked();
	channelMask.bitFieldData.unused = 0;

	if (receiverCapture) 
	{
		receiverCapture->StartCalibration(frameQty, beta, channelMask);
	}

	// Update the state of buttons
	DisplayReceiverStatus();
}


void AWLQtDemo::on_timerTimeout()
{
	myTimer->stop();

	bool bContinue = true;
	if (videoCapture->WasStopped()) 
	{
		bContinue = false;
	}
#if 0  // Closing the videoviewer does not stop the application anymore
	else if (videoViewer->WasStopped())
	{
		bContinue = false;
	}
#endif
	else if (receiver->WasStopped())
	{
		bContinue = false;
	}
#if 0  // closing the fused viewer windows does not stop the application anymore
	else if (fusedCloudViewer->WasStopped())
	{
		bContinue = false;
	}
#endif
	if (bContinue && receiverCapture)
	{
		// Use the frame snapped by the  as the current frame
		// all displays will reference to.
		uint32_t lastDisplayedFrame = receiverCapture->SnapSnapshotFrameID();

		// Update the status information
		if (receiverCapture->receiverStatus.bUpdated) 
		{
			DisplayReceiverStatus();
		}
	}

	// Uopdate the 3D display

	if (bContinue && receiver) 
	{
		receiver->DoThreadIteration();

	if (receiver->WasStopped()) bContinue = false;
	}


	if (bContinue) 
	{
		DisplayReceiverValues();
		DisplayReceiverValuesTo2DScanView();
	}


	if (bContinue && fusedCloudViewer && !fusedCloudViewer->WasStopped()) 
	{
		if (fusedCloudViewer->viewers.size()>=1) 
		{
			fusedCloudViewer->viewers[0]->DoThreadIteration();
#if 0 // closing the fused viewer windows does not stop the application anymore
			if (fusedCloudViewer->WasStopped()) bContinue = false;
#endif
		}
	}

	// Update the menus for the 3D view and camera view, since we do not get any notifiocation from them
	if (ui.action3D_View->isChecked() && (!fusedCloudViewer || fusedCloudViewer->WasStopped())) 
	{
		ui.action3D_View->toggle();
	}


#if 0 // closing the fused viewer windows does not stop the application anymore
	if (bContinue  && fusedCloudViewer && !fusedCloudViewer->WasStopped())
#else
	if (bContinue)
#endif
	{
		myTimer->start(LOOP_RATE);
	}
	else 
	{
		this->close();
	}
}

void AWLQtDemo::DisplayReceiverValuesTo2DScanView()
{
#if 0
	// Force update of the frame displayed by user interfaces to the currentframe
	uint32_t lastDisplayedFrame = receiverCapture->SnapSnapshotFrameID();
#else
	// Use the frame snapped by the main display timer as the current frame
	// display will �
	uint32_t lastDisplayedFrame = receiverCapture->GetSnapshotFrameID();
#endif
	DetectionDataVect vect;
	DetectionData detect;
	float currentAngle = 0;

	for (int channelID = 0; channelID < channelQty; channelID++) 
	{
		switch(channelID)
		{
		case 0: currentAngle = -15; break;
		case 1: currentAngle =  -5; break;
		case 2: currentAngle = 5; break;
		case 3: currentAngle = 15; break;
		case 4: currentAngle = -4.6; break;
		case 5: currentAngle = 0.0; break;
		case 6: currentAngle = 4.6; break;
		default: currentAngle = 0.0; break;
		}

		if (channelID < receiverCapture->GetChannelQty())
		{
			if (receiverCapture->GetFrameQty()) 
			{

				ChannelFrame::Ptr channelFrame(new ChannelFrame(channelID));

				// Thread safe
				// The UI thread "Snaps" the frame ID for all other interface objects to display
				if (receiverCapture->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, receiverCaptureSubscriberID)) 
				{

					int detectionQty = channelFrame->detections.size();
					int detectionIndex = 0;
					for (int i = 0; i < detectionQty; i++)
					{
						Detection::Ptr detection = channelFrame->detections.at(i);
						if ((detection->distance >= receiverCapture->GetMinDistance()) && 
							(detection->distance <= receiverCapture->GetMaxDistance())) 
						{
							detect.distanceRadial = detection->distance;
							detect.id = detection->detectionID;
							detect.fromChannel =  detection->channelID;
							detect.angle = currentAngle;
							detect.angleWidth = ((channelID > 4) ? 4.3  : 9.0);
							detect.distanceLongitudinal = (-(detect.distanceRadial*cosf(DEG2RAD(detect.angle+180))))-mCfgSensor.sensorDepth;
							vect.append(detect);

							//AddDistanceToText(detectionIndex++, tableWidgets[channelID], detection);
						}
					}
				}
			}
		}
	}

	m2DScan->slotDetectionDataChanged(&vect);

}

void AWLQtDemo::DisplayReceiverStatus()

{
	bool bEnableButtons = true;

	if (receiverCapture) 
	{
		bEnableButtons = true;

		boost::mutex::scoped_lock rawLock(receiverCapture->currentReceiverCaptureSubscriptions->GetMutex());


		receiverCapture->receiverStatus.bUpdated = false;
		ReceiverStatus status = receiverCapture->receiverStatus;
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

		formattedString.sprintf("%X", status.fpgaRegisterValueRead);
		ui.registerFPGAValueGetLineEdit->setText(formattedString);

		formattedString.sprintf("%04X", status.adcRegisterAddressRead);
		ui.registerADCAddressGetLineEdit->setText(formattedString);

		formattedString.sprintf("%X", status.adcRegisterValueRead);
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
	}
	else  /* ! receiverCapture */ 
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

void AWLQtDemo::PrepareTableViews()

{
	QTableWidget *tableWidgets[channelQty];

	tableWidgets[0] = ui.distanceTable1;
	tableWidgets[1] = ui.distanceTable2;
	tableWidgets[2] = ui.distanceTable3;
	tableWidgets[3] = ui.distanceTable4;
	tableWidgets[4] = ui.distanceTable5;
	tableWidgets[5] = ui.distanceTable6;
	tableWidgets[6] = ui.distanceTable7;

	for (int sensor = 0; sensor < channelQty; sensor++) 
	{
		for (int row = 0; row < tableWidgets[sensor]->rowCount(); row++) 
		{
			for (int column = 0; column < tableWidgets[sensor]->columnCount(); column++)
			{
			QTableWidgetItem *newItem = new QTableWidgetItem("");
			tableWidgets[sensor]->setItem(row, column, newItem);
			}
		}
	}
}



void AWLQtDemo::on_algo1RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	receiverCapture->SetAlgorithm(1);
	PrepareParametersView();
}

void AWLQtDemo::on_algo2RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	receiverCapture->SetAlgorithm(2);
	PrepareParametersView();
}

void AWLQtDemo::on_algo3RadioButton_setChecked(bool bChecked)
{
	if (!bChecked) return;

	receiverCapture->SetAlgorithm(3);
	PrepareParametersView();
}



void AWLQtDemo::PrepareParametersView()

{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = settingsPtr->defaultAlgo;

	if (receiverCapture) 
	{
		currentAlgo = receiverCapture->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->defaultAlgo;
	}

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];
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
		QTableWidgetItem *newItem = new QTableWidgetItem(algoParameters[row].sIndex);
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		newItem->setCheckState(Qt::Unchecked);
		ui.parametersTable->setItem(row, eParameterCheckColumn, newItem);

		// Column 1: Is Description .  Not editable text
		newItem = new QTableWidgetItem(algoParameters[row].sDescription);
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

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = settingsPtr->defaultAlgo;

	if (receiverCapture) 
	{
		currentAlgo = receiverCapture->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->defaultAlgo;
	}

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];

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
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = settingsPtr->defaultAlgo;

	if (receiverCapture) 
	{
		currentAlgo = receiverCapture->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->defaultAlgo;
	}

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];

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

			receiverCapture->SetAlgoParameter(settingsPtr->parametersAlgos[currentAlgo], parameterAddress, parameterValue); 
		} // if checked
	} // for 
}

void AWLQtDemo::on_algoParametersGetPushButton_clicked()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = settingsPtr->defaultAlgo;

	if (receiverCapture) 
	{
		currentAlgo = receiverCapture->receiverStatus.currentAlgo;
		if (currentAlgo > ALGO_QTY) currentAlgo = settingsPtr->defaultAlgo;
	}

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];

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
			receiverCapture->QueryAlgoParameter(settingsPtr->parametersAlgos[currentAlgo],  parameterAddress); 
		} // if checked
	} // for 
}



void AWLQtDemo::PrepareGlobalParametersView()

{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = 0;

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];
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
		QTableWidgetItem *newItem = new QTableWidgetItem(algoParameters[row].sIndex);
		newItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		newItem->setCheckState(Qt::Unchecked);
		ui.globalParametersTable->setItem(row, eParameterCheckColumn, newItem);

		// Column 1: Is Description .  Not editable text
		newItem = new QTableWidgetItem(algoParameters[row].sDescription);
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

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];

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

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];

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

			receiverCapture->SetGlobalAlgoParameter(settingsPtr->parametersAlgos[currentAlgo], parameterAddress, parameterValue); 
		} // if checked
	} // for 
}

void AWLQtDemo::on_globalParametersGetPushButton_clicked()
{
	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	int currentAlgo = 0;

	QList<AlgorithmParameters> algoParameters = settingsPtr->parametersAlgos[currentAlgo];

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
			receiverCapture->QueryGlobalAlgoParameter(settingsPtr->parametersAlgos[currentAlgo],  parameterAddress); 
		} // if checked
	} // for 
}


void AWLQtDemo::DisplayReceiverValues()

{
	QTableWidget *tableWidgets[channelQty];

	tableWidgets[0] = ui.distanceTable1;
	tableWidgets[1] = ui.distanceTable2;
	tableWidgets[2] = ui.distanceTable3;
	tableWidgets[3] = ui.distanceTable4;
	tableWidgets[4] = ui.distanceTable5;
	tableWidgets[5] = ui.distanceTable6;
	tableWidgets[6] = ui.distanceTable7;

#if 0
	// Force update of the frame displayed by user interfaces to the currentframe
	uint32_t lastDisplayedFrame = receiverCapture->SnapSnapshotFrameID();
#else
	// Use the frame snapped by the main display timer as the current frame
	// display will �
	uint32_t lastDisplayedFrame = receiverCapture->GetSnapshotFrameID();
#endif	
	for (int channelID = 0; channelID < channelQty; channelID++) 
	{
		if (channelID < receiverCapture->GetChannelQty())
		{
			if (receiverCapture->GetFrameQty()) 
			{

				ChannelFrame::Ptr channelFrame(new ChannelFrame(channelID));

				// Thread safe
				// The UI thread "Snaps" the frame ID for all other interface objects to display
				if (receiverCapture->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, receiverCaptureSubscriberID)) 
				{

					int detectionQty = channelFrame->detections.size();
					int detectionIndex = 0;
					for (int i = 0; i < detectionQty; i++)
					{
						Detection::Ptr detection = channelFrame->detections.at(i);
						if ((detection->distance >= receiverCapture->GetMinDistance()) && 
							(detection->distance <= receiverCapture->GetMaxDistance())) 
						{
							AddDistanceToText(detectionIndex++, tableWidgets[channelID], detection);
						}
					}
					for  (int i = detectionIndex; i < tableWidgets[channelID]->rowCount(); i++) 
					{
							AddDistanceToText(i, tableWidgets[channelID], 0.0f);
					}
				}
			}
		}
	}
}


void AWLQtDemo::AddDistanceToText(int detectionID, QTableWidget *pTable, Detection::Ptr &detection)

{
	QString str;
	QString velocityStr;

	if (detection->distance <= 0.0)
	{
		str.sprintf("");
		velocityStr.sprintf("");
	}
	else
	{
		str.sprintf("%f", detection->distance);
		velocityStr.sprintf("%.1f", detection->velocity);
	}

	if (pTable->isVisible())
	{
		pTable->item(detectionID, eRealTimeDistanceColumn)->setText(str);
		pTable->item(detectionID, eRealTimeVelocityColumn)->setText(velocityStr);
	}

}

void AWLQtDemo::AddDistanceToText(int detectionID, QTableWidget *pTable, float distance, 
								 TrackID trackID, 
								 Detection::ThreatLevel threatLevel, 
								float intensity,
								float velocity)

{
	QString distanceStr;
	QString trackStr;
	QString velocityStr;
	QString intensityStr;
	QString threatStr;


	if (distance <= 0.0)
	{
		distanceStr.sprintf("");
		trackStr.sprintf("");
		velocityStr.sprintf("");
		intensityStr.sprintf("");
		threatStr.sprintf("");
	}
	else
	{
		distanceStr.sprintf("%f", distance);

		if (trackID > 0) 
		{
			trackStr.sprintf("%d", trackID);
		}
		else 
		{
			trackStr.sprintf("");
		}


		if (velocity != AWL_FNAN) 
		{
			velocityStr.sprintf("%.1f", velocity);
		}
		else 
		{
			velocityStr.sprintf("");
		}

		if (intensity != AWL_FNAN)
		{
			intensityStr.sprintf("%.0f", intensity * 100);
		}
		else 
		{
			intensityStr.sprintf("");
		}


		switch(threatLevel)
		{
		case Detection::eThreatNone:
			{
				threatStr.sprintf("");
			}
			break;

		case Detection::eThreatLow:
			{
				threatStr.sprintf("Low");
			}
			break;

		case Detection::eThreatWarn:
			{
				threatStr.sprintf("Warn");
			}
			break;

		case Detection::eThreatCritical:
			{
				threatStr.sprintf("Critical");
			}
			break;

		default:
			{
			}

		}
	}

	if (pTable->isVisible())
	{
		pTable->item(detectionID, eRealTimeDistanceColumn)->setText(distanceStr);
		pTable->item(detectionID, eRealTimeVelocityColumn)->setText(velocityStr);
		pTable->item(detectionID, eRealTimeLevelColumn)->setText(intensityStr);
		pTable->item(detectionID, eRealTimeTrackColumn)->setText(trackStr);
	}
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

void AWLQtDemo::on_viewGraphActionToggled()
{
	if (ui.actionGraph->isChecked()) 
	{
		scopeWindow->show();
		scopeWindow->start(receiverCapture);
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
		videoViewer->Go();
	else
		videoViewer->Stop();
}

void AWLQtDemo::on_view2DClose()
{
	ui.action2D->setChecked(false);
}


void AWLQtDemo::on_viewGraphClose()
{
	ui.actionGraph->setChecked(false);
}


void AWLQtDemo::FillFPGAList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < settingsPtr->registersFPGA.count(); i++) 
	{
		QString sLabel = settingsPtr->registersFPGA[i].sDescription;
		sLabel = settingsPtr->registersFPGA[i].sIndex;
		sLabel += ": ";
		sLabel += settingsPtr->registersFPGA[i].sDescription;
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
	registerAddress = settingsPtr->registersFPGA[comboIndex].address;

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
	if (receiverCapture) 
	{
		receiverCapture->SetFPGARegister(registerAddress, registerValue);
	}

}

void AWLQtDemo::on_registerFPGAGetPushButton_clicked()
{
	uint16_t registerAddress;


	int comboIndex = ui.registerFPGAAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	registerAddress = settingsPtr->registersFPGA[comboIndex].address;

	// Now update user interface
	ui.registerFPGAAddressGetLineEdit->setText("");
	ui.registerFPGAValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCapture) 
	{
		receiverCapture->QueryFPGARegister(registerAddress);
	}
}

void AWLQtDemo::FillADCList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < settingsPtr->registersADC.count(); i++) 
	{
		QString sLabel = settingsPtr->registersADC[i].sDescription;
		sLabel = settingsPtr->registersADC[i].sIndex;
		sLabel += ": ";
		sLabel += settingsPtr->registersADC[i].sDescription;
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
	registerAddress = settingsPtr->registersADC[comboIndex].address;

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
	if (receiverCapture) 
	{
		receiverCapture->SetADCRegister(registerAddress, registerValue);
	}

}

void AWLQtDemo::on_registerADCGetPushButton_clicked()
{
	uint16_t registerAddress;


	int comboIndex = ui.registerADCAddressSetComboBox->currentIndex();
	if (comboIndex < 0) return;

	AWLSettings *settingsPtr = AWLSettings::GetGlobalSettings();
	registerAddress = settingsPtr->registersADC[comboIndex].address;

	// Now update user interface
	ui.registerADCAddressGetLineEdit->setText("");
	ui.registerADCValueGetLineEdit->setText("");

	// Send the command to the device
	if (receiverCapture) 
	{
		receiverCapture->QueryADCRegister(registerAddress);
	}
}


void AWLQtDemo::FillGPIOList(AWLSettings *settingsPtr)
{
	for (int i = 0; i < settingsPtr->registersGPIO.count(); i++) 
	{
		QString sLabel = settingsPtr->registersGPIO[i].sDescription;
		sLabel = settingsPtr->registersGPIO[i].sIndex;
		sLabel += ": ";
		sLabel += settingsPtr->registersGPIO[i].sDescription;
		if (settingsPtr->registersGPIO[i].pendingUpdates)
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

	for (int i = 0; i < settingsPtr->registersGPIO.count(); i++) 
	{
		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);

		QString sLabel = settingsPtr->registersGPIO[i].sDescription;
		sLabel = settingsPtr->registersGPIO[i].sIndex;
		sLabel += ": ";
		sLabel += settingsPtr->registersGPIO[i].sDescription;
		if (settingsPtr->registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}
	
		if (settingsPtr->registersGPIO[i].value) 
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
	for (int i = 0; i < settingsPtr->registersGPIO.count(); i++) 
	{
		uint16_t registerAddress = settingsPtr->registersGPIO[i].address;
		uint32_t registerValue = 0;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);
		Qt::CheckState checkState = listItem->checkState();
		if (checkState == Qt::Checked) 
		{
			registerValue = 1;
		}


		// Send the command to the device
		if (receiverCapture) 
		{
			receiverCapture->SetGPIORegister(registerAddress, registerValue);
		}

		// Update the user interface
		QString sLabel = settingsPtr->registersGPIO[i].sDescription;
		sLabel = settingsPtr->registersGPIO[i].sIndex;
		sLabel += ": ";
		sLabel += settingsPtr->registersGPIO[i].sDescription;
		if (settingsPtr->registersGPIO[i].pendingUpdates)
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
	for (int i = 0; i < settingsPtr->registersGPIO.count(); i++) 
	{
		uint16_t registerAddress = settingsPtr->registersGPIO[i].address;

		QListWidgetItem *listItem = ui.registerGPIOListWidget->item(i);
	
		// Send the command to the device
		if (receiverCapture) 
		{
			receiverCapture->QueryADCRegister(registerAddress);		
		}

		// Update the user interface
		QString sLabel = settingsPtr->registersGPIO[i].sDescription;
		sLabel = settingsPtr->registersGPIO[i].sIndex;
		sLabel += ": ";
		sLabel += settingsPtr->registersGPIO[i].sDescription;
		if (settingsPtr->registersGPIO[i].pendingUpdates)
		{
			sLabel += " -- UPDATING...";
		}		
		listItem->setText(sLabel);

	}
}

void AWLQtDemo::closeEvent(QCloseEvent * event)
{
	qApp->closeAllWindows();
}	

