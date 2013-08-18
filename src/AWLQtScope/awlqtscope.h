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
	typedef enum 
	{
		eScopeRaw = 0,
		eScopeTrackDistance = 1,
		eScopeTrackSingleChannel = 2
	}
	ScopeMode;

public:
	AWLQtScope(QWidget *parent = 0);
	~AWLQtScope();

	void start(ReceiverCapture::Ptr inReceiverCapture);
	void stop();

	CurveData::Array *getCurveData() {return(&d_curveDataArray);};
	CurveData::Vector *getCurveData(int channel) {return(d_curveDataArray[channel]);};
	CurveData::Ptr getCurveData(int channel, int detection) {return((*d_curveDataArray[channel])[detection]);};


	AWLScopePlot *getPlot(int inChannel) {return(d_plot[inChannel]);};
	AWLQtScope::ScopeMode GetScopeMode() {return(scopeMode);};
	AWLQtScope::ScopeMode SetScopeMode(AWLQtScope::ScopeMode inMode) {return(scopeMode=inMode);};
	
Q_SIGNALS:
	// No signals yet.
   void closed();

private slots:
	void on_scopeModeRaw_setChecked(bool bChecked);
	void on_scopeModeTrackDistance_setChecked(bool bChecked);
	void on_scopeModeTrackSingleChannel_setChecked(bool bChecked);
	void on_scopeCurveStyleDots_setChecked(bool bChecked);
	void on_scopeCurveStyleLines_setChecked(bool bChecked);

public Q_SLOTS:
    virtual void timerEvent( QTimerEvent * );

protected:
	void	closeEvent(QCloseEvent * event);

	void updateCurveDataRaw();
	void updateCurveDataTrackDistance();

private:
	ScopeMode	scopeMode;
	QwtPlotCurve::CurveStyle curveStyle;
	AcquisitionSequence::TrackingMode trackingMode;


	Ui::AWLQtScopeClass ui; 

    WheelBox *d_intervalWheel;

	QVector<AWLScopePlot *> d_plot;
	CurveData::Array d_curveDataArray;
	int d_timerId;

	ReceiverCapture::Ptr d_receiverCapture;
	uint32_t d_lastFrameID;
};

#endif // AWLQTSCOPE_H
