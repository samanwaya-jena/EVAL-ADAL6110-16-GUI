#include "awlqtscope.h"
#include "AWLScopePlot.h"
#include "knob.h"
#include "wheelbox.h"

#include <QDesktopWidget>
#include <QApplication>
#include <qwt_scale_engine.h>
#include <qwt_plot_curve.h>
#include <qlabel.h>
#include <qlayout.h>

#include "AWLSettings.h"
#include "DebugPrintf.h"

int timerInterval = 30; // In ms.  So 30FPS

const AWLQtScope::ScopeMode defaultScopeMode = AWLQtScope::eScopeRaw;
const QwtPlotCurve::CurveStyle defaultCurveStyle = QwtPlotCurve::Dots;
const AcquisitionSequence::TrackingMode defaultTrackingMode = AcquisitionSequence::eTrackAllChannels;


const double intervalLength = 15.0; // seconds

//const AWLQtScope::ScopeMode defaultScopeMode = AWLQtScope::eScopeTrackDistance;

AWLQtScope::AWLQtScope(QWidget *parent)
	: QWidget(parent),
	d_lastFrameID(0),
	d_timerId(0),
	scopeMode(defaultScopeMode),
	curveStyle(defaultCurveStyle),
	trackingMode(defaultTrackingMode)
{
	ui.setupUi(this);

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	timerInterval = globalSettings->scopeTimerInterval;

	// In demo mode, change the title of the Window
	if (globalSettings->bEnableDemo)
	{
		this->setWindowTitle(this->windowTitle() + " [DEMO Mode]");
	}

	// Position the widget on the top left corner
	QRect scr = QApplication::desktop()->screenGeometry();
	move(scr.left(), scr.top()+5); 

	// Append the individual scope windows
	d_plot.append(ui.channel1PlotFrame);
	d_plot.append(ui.channel2PlotFrame);
	d_plot.append(ui.channel3PlotFrame);
	d_plot.append(ui.channel4PlotFrame);
	d_plot.append(ui.channel5PlotFrame);
	d_plot.append(ui.channel6PlotFrame);
	d_plot.append(ui.channel7PlotFrame);
	
	for (int i = 0; i < d_plot.size(); i++)
	{ 
		d_curveDataArray.append(d_plot[i]->getCurveData());

		// Set the curve style
		d_plot[i]->setCurveStyle(curveStyle);

		// Set the individual interval length
		d_plot[i]->setIntervalLength( intervalLength );
	}

	d_plot[0]->setAxisScale( QwtPlot::yLeft, 0, 50.0);
	d_plot[1]->setAxisScale( QwtPlot::yLeft, 0, 50.0);
	d_plot[2]->setAxisScale( QwtPlot::yLeft, 0, 50.0);
	d_plot[3]->setAxisScale( QwtPlot::yLeft, 0, 50.0);
	d_plot[4]->setAxisScale( QwtPlot::yLeft, 0, 50.0);
	d_plot[5]->setAxisScale( QwtPlot::yLeft, 0, 50.0);
	d_plot[6]->setAxisScale( QwtPlot::yLeft, 0, 50.0);



	// Place the windget used to set the display interval
	d_intervalWheel = new WheelBox( "Displayed [s]", 1.0, 100.0, 1.0, this );
	d_intervalWheel->setValue( intervalLength );

	// layout the interval wheel within the window
#if 1
	QVBoxLayout* vLayout1 = new QVBoxLayout();
	vLayout1->addWidget( d_intervalWheel );
	vLayout1->addWidget( ui.scopeModeGroupBox);	
	vLayout1->addWidget( ui.scopeCurveStyleGroupBox);
	vLayout1->addStretch( 10 );

	QHBoxLayout *layout = new QHBoxLayout( this );
	layout->addWidget( ui.scopeFrame, 10);
	layout->addLayout( vLayout1 );

#endif

	// Set the default value foor the scope mode check box
	ui.scopeModeRawRadioButton->setChecked(scopeMode ==  AWLQtScope::eScopeRaw);
	ui.scopeModeTrackRadioButton->setChecked(scopeMode==  AWLQtScope::eScopeTrackDistance);
	ui.scopeModeTrackSingleChannelRadioButton->setChecked(scopeMode ==  AWLQtScope::eScopeTrackSingleChannel);

	// Set the default value foor the curveStyle check box
	ui.scopeCurveStyleDotsRadioButton->setChecked(curveStyle ==  QwtPlotCurve::Dots);
	ui.scopeCurveStyleLinesRadioButton->setChecked(curveStyle==  QwtPlotCurve::Lines);

	// Start the plot
	for (int i = 0; i < d_plot.size(); i++) 
	{
		connect( d_intervalWheel, SIGNAL( valueChanged( double ) ),
			d_plot[i], SLOT( setIntervalLength( double ) ) );
	}
}


AWLQtScope::~AWLQtScope()
{
	stop();
	delete d_intervalWheel;
}


void AWLQtScope::start(ReceiverCapture::Ptr inReceiverCapture)
{
	d_receiverCapture = inReceiverCapture;

	// Set the tracking mode
	if (d_receiverCapture && d_receiverCapture->acquisitionSequence)
	{
		d_receiverCapture->acquisitionSequence->SetTrackingMode(trackingMode);
	} 

	for (int i = 0; i < d_plot.size(); i++) 
	{
		d_plot[i]->start(d_receiverCapture, i);
	}

	if (d_timerId == 0) d_timerId = startTimer( timerInterval );
}


void AWLQtScope::stop()
{
	for (int i = 0; i < d_plot.size(); i++) 
	{
	//	d_plot[i]->stop();
	}

	if (d_timerId != 0)
	{
		killTimer(d_timerId);
		d_timerId = -1;
	}
}

void AWLQtScope::timerEvent( QTimerEvent *event )
{

    if ( event->timerId() == d_timerId )
    {
		switch (scopeMode) 
		{
		case AWLQtScope::eScopeRaw:
			{
			updateCurveDataRaw();
			}
			break;
		case AWLQtScope::eScopeTrackDistance:
			{
			updateCurveDataTrackDistance();
			}
			break;
		case AWLQtScope::eScopeTrackSingleChannel:
			{
			updateCurveDataTrackDistance();
			}
			break;

		default:
			{
			updateCurveDataRaw();
			}
			break;
		}

		
		for (int i = 0; i < d_plot.size(); i++)
		{
			d_plot[i]->doTimeUpdate();
		}

        return;
    }

    QWidget::timerEvent( event );
}

void AWLQtScope::updateCurveDataRaw()

{
	if (!d_receiverCapture->GetFrameQty()) return;   // No frame yet produced

	boost::mutex::scoped_lock updateLock( d_receiverCapture->currentReceiverCaptureSubscriptions->GetMutex());

	// Determine which frames need to be updated
	int startFrame = d_receiverCapture->acquisitionSequence->FindFrameIndex(d_lastFrameID)+1;
	int lastFrame = d_receiverCapture->GetFrameQty()-1;
	d_lastFrameID = d_receiverCapture->GetLastFrameID();  // Mark the last frame for posterity

	// Add the data from all the new frames to the scope
	for (int frameIndex = startFrame; frameIndex <= lastFrame; frameIndex++) 
	{
		double elapsed = d_receiverCapture->GetFrameTimeAtIndex(frameIndex);
		// Note that elpased in in millisec and our curves expect seconds.
		elapsed /= 1000;

		int channelQty = d_curveDataArray.size();

		for (int channelID = 0; channelID < channelQty; channelID++)
		{
			// Thread safe
			int detectionQty = d_receiverCapture->GetDetectionQtyPerChannel();
			int detectionIndex = 0;
			int maxDetections = d_curveDataArray[channelID]->size();
			
			for (int i = 0; (i < detectionQty) && (i < maxDetections); i++)
			{
				Detection::Ptr detection = Detection::Ptr(new Detection(channelID, i));

				d_receiverCapture->GetDetection(frameIndex, channelID, i, detection, -1);

				if ((detection->distance >= d_receiverCapture->GetMinDistance()) && 
					(detection->distance <= d_receiverCapture->GetMaxDistance())) 
				{
					// Replace the new point to the end, with detected value
					const QPointF s(elapsed,  detection->distance);
					d_curveDataArray[channelID]->at(detectionIndex++)->addValue(s);
				} 
			} // For i;

			// Add empty values to the remaining empty tracks
			for  (int i = detectionIndex; i < maxDetections; i++) 
			{
				const QPointF s(elapsed, 0.0);
					d_curveDataArray[channelID]->at(i)->addValue(s);
			} // For i;

		} // for channelID
	} // For frameIndex

	updateLock.unlock();
}

void AWLQtScope::updateCurveDataTrackDistance()

{
	if (!d_receiverCapture->GetFrameQty()) return;   // No frame yet produced

	boost::mutex::scoped_lock updateLock( d_receiverCapture->currentReceiverCaptureSubscriptions->GetMutex());

	// Get the currentTracking info 
	Track::Vector &tracks = d_receiverCapture->acquisitionSequence->GetTracks();
	int trackQty = tracks.size();


	// Note the date and time;
	int lastFrame = d_receiverCapture->GetFrameQty()-1;
	double elapsed = d_receiverCapture->GetFrameTimeAtIndex(lastFrame);

	// Note that elapsed in in millisec and our curves expect seconds.
	elapsed /= 1000;

	// Add the data from all the the latest track info to the scope
	int channelQty = d_curveDataArray.size();
	for (int channelID = 0; channelID < channelQty; channelID++)
	{
		// Thread safe
		int trackQty = tracks.size();
		int trackIndex = 0;
		int maxTracks = d_curveDataArray[channelID]->size();
			
		for (int i = 0; i < trackQty; i++)
		{
			Track::Ptr track = tracks.at(i);
			double deltaTime = track->timeStamp - track->firstTimeStamp;

			if ((track->Contains(channelID)) && 
				(track->distance >= d_receiverCapture->GetMinDistance()) && 
				(track->distance <= d_receiverCapture->GetMaxDistance()) &&
				(track->IsProbable()) &&
				(trackIndex < d_curveDataArray[channelID]->size())) 
			{
				// Replace the new point to the end, with detected value
				const QPointF s(elapsed,  track->distance);
				d_curveDataArray[channelID]->at(trackIndex++)->addValue(s);
			} 
		} // For i;

		// Add empty values to the remaining empty tracks
		for  (int i = trackIndex; i < maxTracks; i++) 
		{
			const QPointF s(elapsed, 0.0);
				d_curveDataArray[channelID]->at(i)->addValue(s);
		} // For i;

	} // for channelID

	updateLock.unlock();
}


void AWLQtScope::on_scopeModeRaw_setChecked(bool bChecked)
{
	if (bChecked) 
	{
		scopeMode = AWLQtScope::eScopeRaw;

		// Set the tracking mode
		if (d_receiverCapture && d_receiverCapture->acquisitionSequence)
		{
			trackingMode = AcquisitionSequence::eTrackAllChannels;
			d_receiverCapture->acquisitionSequence->SetTrackingMode(AcquisitionSequence::eTrackAllChannels);
		} 
	}

}

void AWLQtScope::on_scopeModeTrackDistance_setChecked(bool bChecked)
{
	if (bChecked) 
	{
		scopeMode = AWLQtScope::eScopeTrackDistance;
		// Set the tracking mode
		trackingMode = AcquisitionSequence::eTrackAllChannels;
		if (d_receiverCapture && d_receiverCapture->acquisitionSequence)
		{

			d_receiverCapture->acquisitionSequence->SetTrackingMode(trackingMode);
		} 
	}
}

void AWLQtScope::on_scopeModeTrackSingleChannel_setChecked(bool bChecked)
{
	if (bChecked) 
	{
		scopeMode = AWLQtScope::eScopeTrackSingleChannel;
		// Set the tracking mode
		trackingMode = AcquisitionSequence::eTrackSingleChannel;
		if (d_receiverCapture && d_receiverCapture->acquisitionSequence)
		{

			d_receiverCapture->acquisitionSequence->SetTrackingMode(trackingMode);
		} 
	}
}


void AWLQtScope::on_scopeCurveStyleDots_setChecked(bool bChecked)
{
	if (bChecked) 
	{
		curveStyle = QwtPlotCurve::Dots;
		for (int i = 0; i < d_plot.size(); i++)
		{ 
			// Set the curve style
			d_plot[i]->setCurveStyle(curveStyle);
		}
	}

}

void AWLQtScope::on_scopeCurveStyleLines_setChecked(bool bChecked)
{
	if (bChecked) 
	{
		curveStyle = QwtPlotCurve::Lines;
		for (int i = 0; i < d_plot.size(); i++)
		{ 
			// Set the curve style
			d_plot[i]->setCurveStyle(curveStyle);
		}
	}
}