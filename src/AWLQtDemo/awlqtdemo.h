#ifndef AWLQTDEMO_H
#define AWLQTDEMO_H

#include <QtWidgets/QMainWindow>
#include <QTimer>

#ifndef Q_MOC_RUN
#include <boost/container/vector.hpp>
#endif

#include "ui_awlqtdemo.h"

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "ReceiverCapture.h"
#include "Sensor.h"
#include "VideoViewer.h"
#include "FusedCloudViewer.h"
#include "FOV_2DScan.h"
#include "TableView.h"
#include "..\awlqtscope\awlqtscope.h"


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
	void on_colorImageRadioButton_setChecked(bool bChecked);
	void on_rangeImageRadioButton_setChecked(bool bChecked);

	void on_viewSidePushButton_pressed();
	void on_viewTopPushButton_pressed();
	void on_viewFrontPushButton_pressed();
	void on_viewIsoPushButton_pressed();
	void on_viewZoomPushButton_pressed();

	void on_recordPushButton_clicked();
	void on_playbackPushButton_clicked();
	void on_stopPushButton_clicked();

	void on_simulatedDataInjectCheckBox_setChecked(bool  bChecked);

	void on_sensorHeightSpin_editingFinished();
	void on_sensorDepthSpin_editingFinished();
	void on_calibrationRangeMinSpin_editingFinished();
	void on_calibrationRangeMax0Spin_editingFinished();	
	void on_calibrationRangeMax1Spin_editingFinished();
	void on_calibrationRangeMax2Spin_editingFinished();
	void on_calibrationRangeMax3Spin_editingFinished();
	void on_calibrationRangeMax4Spin_editingFinished();
	void on_calibrationRangeMax5Spin_editingFinished();
	void on_calibrationRangeMax6Spin_editingFinished();
	void on_measurementOffsetSpin_editingFinished();
	void on_decimationSpin_editingFinished();
	void on_pixelSizeSpin_editingFinished();
	void on_targetHintDistanceSpin_editingFinished();
	void on_targetHintAngleSpin_editingFinished();
	void on_distanceLogCheckBox_setChecked(bool  bChecked);

	void on_calibratePushButton_clicked();

	void on_registerFPGASetPushButton_clicked();
	void on_registerFPGAGetPushButton_clicked();
	void on_registerADCSetPushButton_clicked();
	void on_registerADCGetPushButton_clicked();
	void on_registerGPIOSetPushButton_clicked();
	void on_registerGPIOGetPushButton_clicked();

	void on_algo1RadioButton_setChecked(bool bChecked);
	void on_algo2RadioButton_setChecked(bool bChecked);
	void on_algo3RadioButton_setChecked(bool bChecked);
	void on_algo4RadioButton_setChecked(bool bChecked);


	void on_algoParametersSetPushButton_clicked();
	void on_algoParametersGetPushButton_clicked();

	void on_globalParametersSetPushButton_clicked();
	void on_globalParametersGetPushButton_clicked();

	void on_view3DActionToggled();
	void on_view2DActionToggled();
	void on_viewTableViewActionToggled();
	void on_viewGraphActionToggled();
	void on_viewCameraActionToggled();

	void on_view2DClose();
	void on_viewTableViewClose();
	void on_viewGraphClose();


	void on_destroy();

	void on_timerTimeout();

protected:
	// Adjust the default displayed ranges depending on the sensor capabilities
	void AdjustDefaultDisplayedRanges();

	void PrepareParametersView();
	void UpdateParametersView();

	void PrepareGlobalParametersView();
	void UpdateGlobalParametersView();
	void DisplayReceiverStatus();
	void DisplayReceiverStatus(int receiverID);
	void GetLatestDetections(Detection::Vector &detectionData);
	void closeEvent(QCloseEvent * event);

	void FillFPGAList(AWLSettings *settingsPtr);
	void FillADCList(AWLSettings *settingsPtr);
	void FillGPIOList(AWLSettings *settingsPtr);

	void UpdateGPIOList();

	void AWLQtDemo::ChangeRangeMax(int channelID, double range);

private:
	Ui::AWLQtDemoClass ui;
	QTimer *myTimer;

	FOV_2DScan* m2DScan;
	TableView * mTableView;
	ConfigSensor mCfgSensor;
	AWLQtScope* scopeWindow;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloud;


	ReceiverCapture::List receiverCaptures;
	VideoCapture::List videoCaptures;
	VideoViewer::List  videoViewers;

	ReceiverProjector::Ptr receiver3DProjector;
	FusedCloudViewer::Ptr fusedCloudViewer;



	/** \brief Our subscription identifier to access to lidar data. */
	boost::container::vector<Publisher::SubscriberID> receiverCaptureSubscriberIDs;
};

} // namespace AWL          

#endif // AWLQTDEMO_H
