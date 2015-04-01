#ifndef AWLQTSCOPE_H
#define AWLQTSCOPE_H

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

#include <QtWidgets/QWidget>
#include <qwt_plot_curve.h>

#include "ui_awlqtscope.h"

#include "awlscopeplot.h"
#include "curvedata.h"

#include "ReceiverCapture.h"
#include "ReceiverPostProcessor.h"



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

	Ui::AWLQtScopeClass ui; 

    WheelBox *d_intervalWheel;

	QVector<AWLScopePlot *> d_plot;
	CurveData::Array d_distanceCurveDataArray;	
	CurveData::Array d_velocityCurveDataArray;
	int d_timerId;

	ReceiverCapture::Ptr d_receiverCapture;
	/** \brief Our subscription identifier to access to lidar data. */
	Publisher::SubscriberID d_receiverCaptureSubscriberID;
};

#endif // AWLQTSCOPE_H
