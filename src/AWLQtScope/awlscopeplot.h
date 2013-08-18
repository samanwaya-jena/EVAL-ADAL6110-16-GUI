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

	CurveData::Vector * getCurveData() {return(&d_curveData);};
	CurveData::Ptr getCurveData(int i) {return(d_curveData[i]);};

	QwtPlotCurve::CurveStyle getCurveStyle();
	QwtPlotCurve::CurveStyle setCurveStyle(QwtPlotCurve::CurveStyle inCurveStyle);


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
	void updateCurveDataRaw();

    QwtPlotMarker *d_origin;
    CurveVector d_curve;
	CurveData::Vector d_curveData;
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