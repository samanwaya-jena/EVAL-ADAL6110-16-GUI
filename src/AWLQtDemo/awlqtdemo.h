#ifndef AWLQTDEMO_H
#define AWLQTDEMO_H

#include <QtWidgets/QMainWindow>
#include <QTimer>

#include "ui_awlqtdemo.h"

#include "VideoCapture.h"
#include "ReceiverCapture.h"
#include "ReceiverFileCapture.h"
#include "Sensor.h"
#include "VideoViewer.h"
#include "FusedCloudViewer.h"
#include "FOV_2DScan.h"

#include "AWLSettings.h"
#include "..\awlqtscope\awlqtscope.h"

using namespace pcl;


namespace awl
{

class AWLQtDemo : public QMainWindow
{
	Q_OBJECT

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
	void on_measurementOffsetSpin_editingFinished();
	void on_decimationSpin_editingFinished();
	void on_pixelSizeSpin_editingFinished();

	void on_calibratePushButton_clicked();

	void on_registerFPGASetPushButton_clicked();
	void on_registerFPGAGetPushButton_clicked();
	void on_registerADCSetPushButton_clicked();
	void on_registerADCGetPushButton_clicked();

	void on_view3DActionToggled();
	void on_view2DActionToggled();
	void on_viewGraphActionToggled();
	void on_viewCameraActionToggled();

	void on_view2DClose();
	void on_viewGraphClose();


	void on_destroy();

	void on_timerTimeout();

protected:
	void PrepareTableViews();
	void DisplayReceiverValues();
	void AddDistanceToText(int detectionID,  QTableWidget *pTable , Detection::Ptr &detection);
	void AddDistanceToText(int detectionID, QTableWidget *pTable, float distance);
	void DisplayReceiverStatus();
	void DisplayReceiverValuesTo2DScanView();


	void FillFPGAList(AWLSettings *settingsPtr);
	void FillADCList(AWLSettings *settingsPtr);

private:
	Ui::AWLQtDemoClass ui;
	QTimer *myTimer;

	FOV_2DScan* m2DScan;
	ConfigSensor mCfgSensor;
	AWLQtScope* scopeWindow;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloud;
	VideoCapture::Ptr videoCapture;

	ReceiverCapture::Ptr receiverCapture;

	ReceiverProjector::Ptr receiver;
	VideoViewer::Ptr  videoViewer;
	FusedCloudViewer::Ptr fusedCloudViewer;

	/** \brief Our subscription identifier to access to lidar data. */
	Subscription::SubscriberID receiverCaptureSubscriberID;
};

} // namespace AWL          

#endif // AWLQTDEMO_H
