#ifndef AWL_SCOPEPLOT_H
#define AWL_SCOPEPLOT_H

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
};

#endif