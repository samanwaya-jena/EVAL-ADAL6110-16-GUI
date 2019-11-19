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
#ifndef AWLQTDEMO_H
#define AWLQTDEMO_H

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

#include "ReceiverCapture.h"

#include "fov_2dscan.h"
#include "TableView.h"
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
	void on_logFileSelectorButton_pressed();

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

#if defined (USE_OPENCV_VIDEO)
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
	bool GetLatestDetections(SensorCoreScope::Detection::Vector &detectionData);
	bool GetLatestAScans(SensorCoreScope::AScan::Vector &aScanData);
	void closeEvent(QCloseEvent * /*event*/);

	void FillVoxelSelectList();

	void FillFPGAList();
	void FillADCList();
	void FillGPIOList();

	void UpdateGPIOList();

	void ChangeRangeMax(int voxelID, float range);

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
#if defined (USE_OPENCV_VIDEO)
	QAction *actionCameraButton;
#endif
	QAction *actionResizeButton;
	QAction *actionQuitButton;
	QIcon *actionResizeMaximizeIcon;
	QIcon *actionResizeRestoreDownIcon;

	SensorCoreScope::ReceiverCapture::List receiverCaptures;

#if defined (USE_OPENCV_VIDEO)
	VideoCapture::List videoCaptures;
	VideoViewer::List  videoViewers;
#endif

	QLabel * labelConnected;
	QLabel * labelFramerate;
	bool m_bConnected;
	ReceiverFrameRate m_frameRate;

	/** \brief Our subscription identifier to access to lidar data. */
	boost::container::vector<Publisher::SubscriberID> receiverCaptureSubscriberIDs;
};

} // namespace AWL          

#endif // AWLQTDEMO_H
