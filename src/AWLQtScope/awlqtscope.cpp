#include <QDesktopWidget>
#include <QApplication>
#include <qwt_plot_curve.h>
#include <qlabel.h>
#include <qlayout.h>

#include "awlqtscope.h"
#include "AWLScopePlot.h"
#include "knob.h"
#include "wheelbox.h"

#include "AWLSettings.h"
#include "DebugPrintf.h"
#include "tracker.h"


int timerInterval = 30; // In ms.  So 30FPS

const QwtPlotCurve::CurveStyle defaultCurveStyle = QwtPlotCurve::Dots;

const double intervalLength = 15.0; // seconds

AWLQtScope::AWLQtScope(QWidget *parent)
	: QWidget(parent),
	d_timerId(0),
	curveStyle(defaultCurveStyle)
{
	ui.setupUi(this);

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	timerInterval = globalSettings->scopeTimerInterval;

	this->setWindowTitle(this->windowTitle() + " Scope view");

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

		//
		// Distance curve
		d_distanceCurveDataArray.append(d_plot[i]->getDistanceCurveData());

		// Set the curve style
		d_plot[i]->setDistanceCurveStyle(curveStyle);

		//
		// Velocity curve
		d_velocityCurveDataArray.append(d_plot[i]->getVelocityCurveData());

		// Set the curve style
		d_plot[i]->setVelocityCurveStyle(curveStyle);

		// Set the individual interval length: common to both distance and velocity curves
		d_plot[i]->setIntervalLength( intervalLength);
	}

	// Place the windget used to set the display interval
	d_intervalWheel = new WheelBox( "Displayed [s]", 1.0, 100.0, 1.0, this );
	d_intervalWheel->setValue( intervalLength );

	// layout the interval wheel within the window
	QVBoxLayout* vLayout1 = new QVBoxLayout();
	vLayout1->addWidget( d_intervalWheel );
	vLayout1->addWidget( ui.scopeDisplayGroupBox);
	vLayout1->addStretch( 10 );
	vLayout1->addWidget( ui.scopeCurveStyleGroupBox);
	vLayout1->addStretch( 10 );

	QHBoxLayout *layout = new QHBoxLayout( this );
	layout->addWidget( ui.scopeFrame, 10);
	layout->addLayout( vLayout1 );

	// Set the default value for the displayed curves check boxes
	ui.scopeDisplayDistanceCheckBox->setChecked(globalSettings->bDisplayScopeDistance);
	ui.scopeDisplayVelocityCheckBox->setChecked(globalSettings->bDisplayScopeVelocity);

	// Set the default value foor the curveStyle check box
	ui.scopeCurveStyleDotsRadioButton->setChecked(curveStyle ==  QwtPlotCurve::Dots);
	ui.scopeCurveStyleLinesRadioButton->setChecked(curveStyle==  QwtPlotCurve::Lines);

	// Adjust the display units on the labels
	QString velocityText = " (m/s)";
	if (globalSettings->velocityUnits != eVelocityUnitsMS)
		velocityText = " (km/h)";
	ui.scopeDisplayVelocityCheckBox->setText(ui.scopeDisplayVelocityCheckBox->text() + velocityText);


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
	d_receiverCaptureSubscriberID = d_receiverCapture->Subscribe();

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
		d_timerId = 0;
	}
}

void AWLQtScope::timerEvent( QTimerEvent *event )
{

    if ( event->timerId() == d_timerId )
    {
		updateCurveDataRaw();
		
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

	if (!d_receiverCapture->HasNews(d_receiverCaptureSubscriberID)) return;


	AWLSettings *settings = AWLSettings::GetGlobalSettings();
	float minDistance = settings->receiverSettings[d_receiverCapture->receiverID].displayedRangeMin;
	// y Scale for velocities
	float maxVelocity =  settings->maxVelocity2D;
	if (settings->velocityUnits != eVelocityUnitsMS)
		maxVelocity = VelocityToKmH(maxVelocity);

	// Get the pointer to the acquisitionSequence
	AcquisitionSequence::Ptr acquisitionSequence = d_receiverCapture->acquisitionSequence;

	// Determine which frames need to be updated
	Publisher::IssueID requestedFrameID = d_receiverCapture->GetConsumedIssueID(d_receiverCaptureSubscriberID);
	Publisher::IssueID lastFrameID = d_receiverCapture->GetCurrentIssueID(d_receiverCaptureSubscriberID);

	// Process all of the back issues 
	do 
	{
		requestedFrameID++;  // Request the next issue.
		if (d_receiverCapture->LockNews(d_receiverCaptureSubscriberID, requestedFrameID)) // Informs the publisher and locks the mutex; 
		{
			SensorFrame::Ptr sensorFrame;
			if (acquisitionSequence->FindSensorFrame(requestedFrameID, sensorFrame))
			{

				// Get the frame time
				// Note that elapsed in in millisec and our curves expect seconds.
				double elapsed = sensorFrame->timeStamp;
				elapsed /= 1000;

				int channelQty = d_distanceCurveDataArray.size();
				for (int channelID = 0; channelID < channelQty; channelID++)
				{
					float maxDistance =settings->receiverSettings[d_receiverCapture->receiverID].channelsConfig[channelID].maxRange;
					ChannelFrame::Ptr channelFrame = sensorFrame->channelFrames.at(channelID);

					// Thread safe
					int detectionQty = channelFrame->detections.size();
					int detectionIndex = 0;
					int maxDetections = d_distanceCurveDataArray[channelID]->size();

					for (int i = 0; (i < detectionQty) && (i < maxDetections); i++)
					{
						Detection::Ptr detection = channelFrame->detections.at(i);
						if ((detection->distance >= minDistance) && 
							(detection->distance <= maxDistance)) 
						{
							// Replace the new point to the end, with detected value
							const QPointF distancePoint(elapsed,  detection->distance);
							d_distanceCurveDataArray[channelID]->at(detectionIndex++)->addValue(distancePoint);

							float velocity = detection->velocity;
							if (settings->velocityUnits != eVelocityUnitsMS)
								velocity = VelocityToKmH(velocity);

							if (velocity > maxVelocity) velocity = maxVelocity;
							if (velocity < -maxVelocity) velocity = -maxVelocity;

							const QPointF velocityPoint(elapsed,  velocity);
							d_velocityCurveDataArray[channelID]->at(detectionIndex++)->addValue(velocityPoint);
						} // If  
					} // For i;
				} // for channelID
			} // if (acquisitionSequence->FindSensorFrame

			d_receiverCapture->UnlockNews(d_receiverCaptureSubscriberID);
		} //  if (d_receiverCapture->LockNews

	} while (requestedFrameID!= lastFrameID);
}

void AWLQtScope::on_scopeCurveStyleDots_setChecked(bool bChecked)
{
	if (bChecked) 
	{
		curveStyle = QwtPlotCurve::Dots;
		for (int i = 0; i < d_plot.size(); i++)
		{ 
			// Set the curve style
			d_plot[i]->setDistanceCurveStyle(curveStyle);
			d_plot[i]->setVelocityCurveStyle(curveStyle);
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
			d_plot[i]->setDistanceCurveStyle(curveStyle);
			d_plot[i]->setVelocityCurveStyle(curveStyle);
		}
	}
}

void AWLQtScope::on_scopeDisplayDistance_toggled(bool bChecked)
{
	AWLSettings::GetGlobalSettings()->bDisplayScopeDistance = bChecked;
	for (int i = 0; i < d_plot.size(); i++)
	{ 
		// Set the curve style
		d_plot[i]->adjustDisplayedCurves();
	}
}


void AWLQtScope::on_scopeDisplayVelocity_toggled(bool bChecked)
{
	AWLSettings::GetGlobalSettings()->bDisplayScopeVelocity = bChecked;
	for (int i = 0; i < d_plot.size(); i++)
	{ 
		// Set the curve style
		d_plot[i]->adjustDisplayedCurves();
	}
}


void AWLQtScope::closeEvent(QCloseEvent * event)
{
	stop();
	emit closed();
}
