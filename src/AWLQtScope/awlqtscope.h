#ifndef AWLQTSCOPE_H
#define AWLQTSCOPE_H

#include <QtWidgets/QWidget>
#include <qwt_plot_curve.h>

#include "ui_awlqtscope.h"

#include "awlscopeplot.h"
#include "curvedata.h"

#include "ReceiverCapture.h"
#include "Tracker.h"



using namespace std;
using namespace awl;


class Knob;
class WheelBox;

class AWLQtScope : public QWidget
{
	Q_OBJECT

public:
	AWLQtScope(QWidget *parent = 0);
	~AWLQtScope();

	void start(ReceiverCapture::Ptr inReceiverCapture);
	void stop();

	CurveData::Array *getDistanceCurveData() {return(&d_distanceCurveDataArray);};
	CurveData::Vector *getDistanceCurveData(int channel) {return(d_distanceCurveDataArray[channel]);};
	CurveData::Ptr getDistanceCurveData(int channel, int detection) {return((*d_distanceCurveDataArray[channel])[detection]);};


	AWLScopePlot *getPlot(int inChannel) {return(d_plot[inChannel]);};
	
Q_SIGNALS:
	// No signals yet.
   void closed();

private slots:
	void on_scopeCurveStyleDots_setChecked(bool bChecked);
	void on_scopeCurveStyleLines_setChecked(bool bChecked);

	void on_scopeDisplayDistance_toggled(bool bChecked);
	void on_scopeDisplayVelocity_toggled(bool bChecked);

public Q_SLOTS:
    virtual void timerEvent( QTimerEvent * );

protected:
	void	closeEvent(QCloseEvent * event);

	void updateCurveDataRaw();

private:
	QwtPlotCurve::CurveStyle curveStyle;
	AcquisitionSequence::TrackingMode trackingMode;


	Ui::AWLQtScopeClass ui; 

    WheelBox *d_intervalWheel;

	QVector<AWLScopePlot *> d_plot;
	CurveData::Array d_distanceCurveDataArray;	
	CurveData::Array d_velocityCurveDataArray;
	int d_timerId;

	ReceiverCapture::Ptr d_receiverCapture;
	/** \brief Our subscription identifier to access to lidar data. */
	Subscription::SubscriberID d_receiverCaptureSubscriberID;
	uint32_t d_lastFrameID;
};

#endif // AWLQTSCOPE_H
