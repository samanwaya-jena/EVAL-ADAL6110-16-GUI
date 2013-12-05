#ifndef AWL_SCOPEPLOT_H
#define AWL_SCOPEPLOT_H

#include <qwt_plot.h>
#include <qwt_interval.h>
#include <qwt_system_clock.h>
#include <qwt_plot_curve.h>

#include "curvedata.h"

#include "ReceiverCapture.h"


using namespace std;
using namespace awl;


class QwtPlotCurve;
class QwtPlotMarker;
class QwtPlotDirectPainter;

class AWLScopePlot: public QwtPlot
{
    Q_OBJECT

public:
	typedef QVector<QwtPlotCurve *> CurveVector;

public:
    AWLScopePlot( QWidget * = NULL );
    virtual ~AWLScopePlot();

	void start(ReceiverCapture::Ptr inReceiverCapture, int inChannelID);
    virtual void replot();

    virtual bool eventFilter( QObject *, QEvent * );

	// Adjust display of curves depending on settings
	void adjustDisplayedCurves();

	CurveData::Vector * getDistanceCurveData() {return(&d_distanceCurveData);};
	CurveData::Ptr getDistanceCurveData(int i) {return(d_distanceCurveData[i]);};

	CurveData::Vector * getVelocityCurveData() {return(&d_velocityCurveData);};
	CurveData::Ptr getVelocityCurveData(int i) {return(d_velocityCurveData[i]);};

	QwtPlotCurve::CurveStyle getDistanceCurveStyle();
	QwtPlotCurve::CurveStyle setDistanceCurveStyle(QwtPlotCurve::CurveStyle inCurveStyle);

	QwtPlotCurve::CurveStyle getVelocityCurveStyle();
	QwtPlotCurve::CurveStyle setVelocityCurveStyle(QwtPlotCurve::CurveStyle inCurveStyle);

	bool doTimeUpdate();

public Q_SLOTS:
    void setIntervalLength( double );

protected:

	
    virtual void showEvent( QShowEvent * );
    virtual void resizeEvent( QResizeEvent * );

private:
	void setupInterface();

	void updateCurve();
    void incrementInterval();

    QwtPlotMarker *d_origin;
    CurveVector d_distanceCurve;
	CurveData::Vector d_distanceCurveData;

    CurveVector d_velocityCurve;
	CurveData::Vector d_velocityCurveData;

    QVector<int> d_paintedPoints;

    QwtPlotDirectPainter *d_directPainter;

    QwtInterval d_interval;

	ReceiverCapture::Ptr d_receiverCapture;
	int d_channelID;
	uint32_t d_lastFrameID;

	/** \brief Our subscription identifier to access to lidar data. */
	Subscription::SubscriberID d_currentReceiverCaptureSubscriberID;
};

#endif