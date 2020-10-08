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


/** \brief Main application Window.
*/
class AWLQtDemo : public QMainWindow
{
	Q_OBJECT

//		public types and enums:
public:
	/** \brief  Column numbers for parameter view - older version.
	 */
	typedef enum ParameterColumn 
	{
	eParameterCheckColumn = 0,
	eParameterDescriptionColumn = 1,
	eParameterValueColumn = 2,
	eParameterConfirmColumn = 3
	}
	ParameterColumn;

public:
	/** \brief  Constructor for main application window.
	 *
	 *	The constructor has many steps to go though:
	 * - Initialize UI.
	 * - Initialize thread priorities
	 * - Setup configuration file environment and adjust accordingly
	 * - Create devices (receivers, cameras)
	 * - Create display subwindows
	 * - Setup messaging and notification
	 * - Start device acquisition threads
	 */
	AWLQtDemo(int argc, char *argv[]);

	/** \brief  Destructor for main application window.
	 */
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

  void on_registerSaveToFlashPushButton_clicked();
  void on_registerRestoreFactoryDefaultsPushButton_clicked();

	void on_algoSelectComboBox_indexChanged(int newIndex);

	void on_algoParametersSetPushButton_clicked();
	void on_algoParametersGetPushButton_clicked();

	void on_trackerSelectComboBox_indexChanged(int newIndex);

	void on_trackerParametersSetPushButton_clicked();
	void on_trackerParametersGetPushButton_clicked();


	void on_globalParametersSetPushButton_clicked();
	void on_globalParametersGetPushButton_clicked();

  void on_checkBoxFPGAAdvanceModeToggled();
  void on_checkBoxADCAdvanceModeToggled();

  void on_comboBoxMaxRange_indexChanged(int newIndex);
  void on_checkBoxAutoScaleToggled();

  void on_checkBoxAscanSelToggled();
  void on_radioReceiverSelToggled();
  void on_pushButtonSwap_clicked();

  void on_pushButtonSelectAllAscan_clicked();
  void on_pushButtonSelectNoneAscan_clicked();

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

	/** \brief Main timer event used to control refresh of the UI elements
	 *
	 * \ Remark The application refreshes the UI elements based on an external timer.
	 *          This avoids overconsumption of the CPU by "pacing" refreshes. 
	*/
	void on_timerTimeout();

protected:
	/** \brief Setup toolbar layout and events.
	*/
	void SetupToolBar();

	/** \brief Setup Diplay Grid for Layout.
	*/
	void SetupDisplayGrid();

	/** \brief Adjust the default displayed ranges depending on the sensor capabilities.
	*/
	void AdjustDefaultDisplayedRanges();

	/** \brief Prepare the Algo Parameters boxes according to config file.
	*/
	void PrepareAlgoParametersView();

	/** \brief Update Algo Parameters boxes with current Receiver value.
	*/
	void UpdateAlgoParametersView();

	/** \brief Prepare the Tracker Parameters boxes according to config file.
	*/
	void PrepareTrackerParametersView();

	/** \brief Update Tracker Parameters boxes with current Receiver value.
	*/
	void UpdateTrackerParametersView();

	/** \brief Prepare the Global Parameters boxes according to config file.
	*/
	void PrepareGlobalParametersView();

	/** \brief Update Global Parameters boxes with current Receiver value.
	*/
	void UpdateGlobalParametersView();

	/** \brief Display status of the receiver acording to current receiver.
	*/
	void DisplayReceiverStatus();

	/** \brief Display status of the receiver according to  status of the supplied receiverID.
	*/
	void DisplayReceiverStatus(int receiverID);

	/** \brief Fill the detection data vector with the latest detection data.
	 * \return true if the data has changed since last request.
	 * 
	 * \Note  Since all processes are asynchonous, takes a "snapshot" of acquired receiver detections
	 *        For subsequent display
	*/

	bool GetLatestDetections(SensorCoreScope::Detection::Vector &detectionData);

	/** \brief Take a snapshot of AScan data for subsequent display.
	 * \return true if the data has changed since last request.
	 *
	 * \Note  Since all processes are asynchonous, takes a "snapshot" of acquired receiver waveforms
	 *        For subsequent display
	*/
	bool GetLatestAScans(SensorCoreScope::AScan::Vector &aScanData);
	
	void closeEvent(QCloseEvent * /*event*/);

	/** \brief Fill the list used to select Voxels from the Setttinns/Calibration tab
	*          according to the configuration of the Receivers.
	*/
		void FillVoxelSelectList();

	/** \brief Fill the list of FPGA Registers according to register description from the config file.
	*/
	void FillFPGAList();

	/** \brief Fill the list of ADC Registers according to register description from the config file.
	*/
	void FillADCList();
	
	/** \brief Fill the list of GPIO Registers according to register description from the config file.
    */
	void FillGPIOList();

	/** \brief Update teh list of GPIO registers to reflect changes made by the user, or arrival of new sensor.
	*/
	void UpdateGPIOList();

	/** \brief Modify the maximum diaplayed range, used in the 2D view.*/
	void ChangeRangeMax(int voxelID, float range);

 
private:
	Ui::AWLQtDemoClass ui;

	/** \brief Main timer used to trigger periodic refreshes of the UI.*/
	QTimer *myTimer;

	/** \brief  2D View subwindow.
	*/
	FOV_2DScan* m2DScan;
	/** \brief  Table view Subwindow.
	*/
	TableView * mTableView;
	/** \brief  A-Scan (Waveform) View subwindow.
	*/
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

	/** \brief  List holding all of the Lidar Receivers used by the  application.
	*/
	SensorCoreScope::ReceiverCapture::List receiverCaptures;

#if defined (USE_OPENCV_VIDEO)
	/** \brief  List holding all of the video capture devices.
	*/
	VideoCapture::List videoCaptures;

	/** \brief  List holding all of the video viewers.
	*/
	VideoViewer::List  videoViewers;
#endif

	/** \brief  Label used to indicate the connexion state of the application.
	*/
	QLabel * labelConnected;
	/** \brief  Label used to display the frame rate.
	*/
	QLabel * labelFramerate;
	/** \brief  Global application indicator to know if at least one receiver is connected..
	*/
	bool m_bConnected;

	/** \brief Our subscription identifier to access to lidar data. */
	boost::container::vector<Publisher::SubscriberID> receiverCaptureSubscriberIDs;
};

} // namespace AWL          

#endif // AWLQTDEMO_H
