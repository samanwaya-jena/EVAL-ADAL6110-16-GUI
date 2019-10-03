#ifndef AWLQTDEMO_H
#define AWLQTDEMO_H
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

#include <QtWidgets/QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QIcon>
#ifndef Q_MOC_RUN
#include <boost/container/vector.hpp>
#endif

#include "ui_awlqtdemo.h"

#include "AWLSettings.h"

#ifdef USE_OPENCV_VIDEO
#include "VideoCapture.h"
#include "VideoViewer.h"
#endif
#ifdef USE_AP_VIDEO
#include "APVideoCapture.h"
#include "APVideoViewer.h"
#endif

#include "ReceiverCapture.h"

#include "fov_2dscan.h"
#include "TableView.h"
#include "awlqtscope.h"
#include "awlplotscan.h"


namespace awl
{



class AWLQtDemo : public QMainWindow
{
	Q_OBJECT

//		public types and enums:
public:
	typedef enum ParameterColumn 
	{
	eParameterCheckColumn = 0,
	eParameterDescriptionColumn = 1,
	eParameterValueColumn = 2,
	eParameterConfirmColumn = 3
	}
	ParameterColumn;

public:
	AWLQtDemo(int argc, char *argv[]);
	~AWLQtDemo();

private slots:
	void on_recordPushButton_clicked();
	void on_playbackPushButton_clicked();
	void on_stopPushButton_clicked();

	void on_sensorHeightSpin_editingFinished();
	void on_sensorDepthSpin_editingFinished();
	void on_calibrationRangeMinSpin_editingFinished();
	void on_calibrationRangeMaxSpin_editingFinished();	

	void on_measurementOffsetSpin_editingFinished();

	void on_targetHintDistanceSpin_editingFinished();
	void on_targetHintAngleSpin_editingFinished();
	void on_distanceLogCheckBox_setChecked(bool  bChecked);

	void on_receiverCalibStorePushButton_clicked();
	void on_calibratePushButton_clicked();

	void on_registerFPGAAddressSetComboBox_indexChanged(int newIndex);

	void on_registerFPGASetPushButton_clicked();
	void on_registerFPGAGetPushButton_clicked();

	void on_registerADCAddressSetComboBox_indexChanged(int newIndex);

	void on_registerADCSetPushButton_clicked();
	void on_registerADCGetPushButton_clicked();
	void on_registerGPIOSetPushButton_clicked();
	void on_registerGPIOGetPushButton_clicked();
  void on_registerFPGASaveToFlash_clicked();
  void on_registerFPGARestoreFactoryDefaults_clicked();
  void on_registerADCSaveToFlash_clicked();
  void on_registerADCRestoreFactoryDefaults_clicked();

	void on_algoSelectComboBox_indexChanged(int newIndex);

	void on_algoParametersSetPushButton_clicked();
	void on_algoParametersGetPushButton_clicked();

	void on_trackerSelectComboBox_indexChanged(int newIndex);

	void on_trackerParametersSetPushButton_clicked();
	void on_trackerParametersGetPushButton_clicked();


	void on_globalParametersSetPushButton_clicked();
	void on_globalParametersGetPushButton_clicked();
	void on_pushButton_FR_clicked();

  void on_checkBoxAdvanceModeToggled();

  void on_comboBoxMaxRange_indexChanged(int newIndex);
  void on_checkBoxAutoScaleToggled();

  void on_checkBoxAscanSelToggled();
  void on_radioReceiverSelToggled();
  void on_pushButtonSwap_clicked();

  void on_pushButtonSelectAllAscan_clicked();
  void on_pushButtonSelectNoneAscan_clicked();

	void on_checkBoxMiscSystemSelToggled();
	void on_checkBoxMiscLaserSelToggled();
	void on_checkBoxMiscGainSelToggled();
	void on_checkBoxMiscDCSelToggled();

	void on_viewSettingsActionToggled();

	void on_view2DActionToggled();
	void on_viewTableViewActionToggled();
	void on_viewAScanViewActionToggled();
	void on_viewAboutActionTriggered();

#if defined (USE_OPENCV_VIDEO) || defined(USE_AP_VIDEO)
	void on_viewCameraActionToggled();
#endif
	void on_resizeActionToggled();

	void on_view2DClose();
	void on_viewTableViewClose();
	void on_viewAScanClose();

	void on_destroy();

	void on_timerTimeout();

protected:
	// Setup toolbar layout and events
	void SetupToolBar();

	// Setup Diplay Grid for Layout
	void SetupDisplayGrid();

	// Adjust the default displayed ranges depending on the sensor capabilities
	void AdjustDefaultDisplayedRanges();

	void PrepareAlgoParametersView();
	void UpdateAlgoParametersView();

	void PrepareTrackerParametersView();
	void UpdateTrackerParametersView();

	void PrepareGlobalParametersView();
	void UpdateGlobalParametersView();
	void DisplayReceiverStatus();
	void DisplayReceiverStatus(int receiverID);

	// Fil the detection data vector with the latest detection data.
	// Return true if the data has changed since last request.
	bool GetLatestDetections(Detection::Vector &detectionData);
	bool GetLatestAScans(AScan::Vector &aScanData);
	void closeEvent(QCloseEvent * event);

	void FillChannelSelectList();

	void FillFPGAList(AWLSettings *settingsPtr);
	void FillADCList(AWLSettings *settingsPtr);
	void FillGPIOList(AWLSettings *settingsPtr);

	void UpdateGPIOList();

	void ChangeRangeMax(int channelID, double range);

  //void  DoThreadLoop();

private:
	Ui::AWLQtDemoClass ui;
	QTimer *myTimer;

	FOV_2DScan* m2DScan;
	TableView * mTableView;
	AWLPlotScan* mAScanView;

#if 0
	QLabel	mLogoLabel;
#endif
	QAction *actionSettingsButton;
	QAction *action2DButton;
	QAction *actionTableButton;
	QAction *actionAScanButton;
	QAction *actionAboutButton;
#if defined (USE_OPENCV_VIDEO) || defined(USE_AP_VIDEO)
	QAction *actionCameraButton;
#endif
	QAction *actionResizeButton;
	QAction *actionQuitButton;
	QIcon *actionResizeMaximizeIcon;
	QIcon *actionResizeRestoreDownIcon;

	ReceiverCapture::List receiverCaptures;

#if defined (USE_OPENCV_VIDEO)
	VideoCapture::List videoCaptures;
	VideoViewer::List  videoViewers;
#endif
#if defined(USE_AP_VIDEO)
	APVideoCapture::List apVideoCaptures;
	APVideoViewer::List  apVideoViewers;
#endif

	QLabel * labelConnected;
	QLabel * labelFramerate;
	bool m_bConnected;
	int m_frameRate;

	/** \brief Our subscription identifier to access to lidar data. */
	boost::container::vector<Publisher::SubscriberID> receiverCaptureSubscriberIDs;
};

} // namespace AWL          

#endif // AWLQTDEMO_H
