#include "AWLScopePlot.h"
#include "curvedata.h"
#include "signaldata.h"
#include <qwt_plot_grid.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_directpainter.h>
#include <qwt_curve_fitter.h>
#include <qwt_painter.h>
#include <QEvent>

#include "ReceiverCapture.h"


using namespace std;
using namespace awl;

const int detectionQty = 8;

class Canvas: public QwtPlotCanvas
{
public:
    Canvas( QwtPlot *plot = NULL ):
        QwtPlotCanvas( plot )
    {
 		setBorderRadius( 10 );
        setupPalette();
    }

private:
    void setupPalette()
    {
        QPalette pal = palette();


#if QT_VERSION >= 0x040400
        QLinearGradient gradient;
        gradient.setCoordinateMode( QGradient::StretchToDeviceMode );
        gradient.setColorAt( 0.0, QColor( 0, 49, 110 ) );
        gradient.setColorAt( 1.0, QColor( 0, 87, 174 ) );

        pal.setBrush( QPalette::Window, QBrush( gradient ) );
#else
        pal.setBrush( QPalette::Window, QBrush( color ) );
#endif

        // QPalette::WindowText is used for the curve color
 //       pal.setColor( QPalette::WindowText, Qt::green );
		pal.setColor(QPalette::WindowText, QColor(255, 173, 0));

        setPalette( pal );

    }
};

AWLScopePlot::AWLScopePlot( QWidget *parent):
    QwtPlot( parent ),
    d_interval( 0.0, 10.0 ),
	d_channelID(-1),
	d_lastFrameID(0),
	d_currentReceiverCaptureSubscriberID(-1)

{
	setupInterface();
}

AWLScopePlot::~AWLScopePlot()
{
	delete d_clock;
	delete d_directPainter;
}

void AWLScopePlot::setupInterface()

{
	d_clock = new QwtSystemClock();
	d_interval = QwtInterval(0.0, 10.0);
    d_directPainter = new QwtPlotDirectPainter();

	setAutoReplot( false );
    setCanvas( new Canvas() );

    plotLayout()->setAlignCanvasToScales( true );

 //   setAxisTitle( QwtPlot::xBottom, "Time [s]" );

    setAxisScale( QwtPlot::xBottom, d_interval.minValue(), d_interval.maxValue());

	enableAxis(QwtPlot::xBottom, true);
	enableAxis(QwtPlot::yLeft, true);
	enableAxis(QwtPlot::xTop, false);
	enableAxis(QwtPlot::yRight, false);


    QwtPlotGrid *grid = new QwtPlotGrid();
    grid->setPen( Qt::gray, 0.0, Qt::DotLine );
    grid->enableX( true );
    grid->enableXMin( true );
    grid->enableY( true );
    grid->enableYMin( false );
    grid->attach( this );

    d_origin = new QwtPlotMarker();
    d_origin->setLineStyle( QwtPlotMarker::Cross );
    d_origin->setValue( d_interval.minValue() + d_interval.width() / 2.0, 0.0 );
    d_origin->setLinePen( Qt::gray, 0.0, Qt::DashLine );
    d_origin->attach( this );

    for (int i = 0; i < detectionQty; i++) 
	{
		QwtPlotCurve * pCurve = new QwtPlotCurve();
		CurveData::Ptr pCurveData = new CurveData();
#if 0
		pCurve->setStyle( QwtPlotCurve::Lines );
#else
		pCurve->setStyle( QwtPlotCurve::Dots );
#endif
		pCurve->setPen( canvas()->palette().color( QPalette::WindowText ), 2 );
		pCurve->setRenderHint( QwtPlotItem::RenderAntialiased, true );
		pCurve->setPaintAttribute( QwtPlotCurve::ClipPolygons, false );
//		pCurve->setPaintAttribute( QwtPlotCurve::ClipPolygons || QwtPlotCurve::FilterPoints, false );
		pCurve->setData( pCurveData );
		pCurve->attach( this );

		d_curve.append(pCurve);
		d_curveData.append(pCurveData);
		d_paintedPoints.append(0);
	}
}


void AWLScopePlot::start(ReceiverCapture::Ptr inReceiverCapture, int inChannelID)
{
 	d_channelID = inChannelID;
	d_receiverCapture = inReceiverCapture;
	d_currentReceiverCaptureSubscriberID = d_receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();
    d_clock->start();
}

void AWLScopePlot::replot()
{
	for (int i = 0; i < d_curve.size(); i++) 
	{
		getCurveData(i)->values().lock();
	}

    QwtPlot::replot();


	for (int i = 0; i < d_curve.size(); i++) 
	{
		d_paintedPoints[i] = getCurveData(i)->size();
		getCurveData(i)->values().unlock();
	}
}

void AWLScopePlot::setIntervalLength( double interval )
{
    if ( interval > 0.0 && interval != d_interval.width() )
    {
        d_interval.setMaxValue( d_interval.minValue() + interval );

        setAxisScale( QwtPlot::xBottom,
            d_interval.minValue(), d_interval.maxValue() );

        replot();
    }
}

bool AWLScopePlot::doTimeUpdate()

{
    const double elapsed = d_clock->elapsed() / 1000.0;

    if ( elapsed > d_interval.maxValue() ) 
	{
        incrementInterval();
	}
	else 
	{
		replot();
//      updateCurve();
	}

	return(false);
}

void AWLScopePlot::updateCurve()
{
	for (int i = 0; i < d_curve.size(); i++)
	{
		CurveData *data = getCurveData(i);
		data->values().lock();

		const int numPoints = data->size();
		if ( numPoints > d_paintedPoints[i] )
		{
			const bool doClip = !canvas()->testAttribute( Qt::WA_PaintOnScreen );
			if ( doClip )
			{
				/*
				Depending on the platform setting a clip might be an important
				performance issue. F.e. for Qt Embedded this reduces the
				part of the backing store that has to be copied out - maybe
				to an unaccelerated frame buffer device.
				*/

				const QwtScaleMap xMap = canvasMap( d_curve[i]->xAxis() );
				const QwtScaleMap yMap = canvasMap( d_curve[i]->yAxis() );

				QRectF br = qwtBoundingRect( *data,
					d_paintedPoints[i]  - 1, numPoints - 1 );

				const QRect clipRect = QwtScaleMap::transform( xMap, yMap, br ).toRect();
				d_directPainter->setClipRegion( clipRect );
			}

			d_directPainter->drawSeries( d_curve[i],
				d_paintedPoints[i] - 1, numPoints - 1 );
			d_paintedPoints[i] = numPoints;
		}

		data->values().unlock();
	}
}

void AWLScopePlot::incrementInterval()
{
    double elapsed = d_clock->elapsed() / 1000.0;
	elapsed = ceil(elapsed);

	double minValue = ((elapsed - d_interval.width()));

	if (minValue <0.0) minValue = 0.0;
	double maxValue = minValue + d_interval.width();
    d_interval = QwtInterval( minValue, maxValue);


    for (int i = 0; i < d_curveData.size(); i++) 
	{
		CurveData *data = getCurveData(i);
		data->values().clearStaleValues( d_interval.minValue() );
	}



    setAxisScale( QwtPlot::xBottom, d_interval.minValue(), d_interval.maxValue());

#if 0
    d_origin->setValue( d_interval.minValue() + d_interval.width() / 2.0, 0.0 );
#endif

    replot();

}

void AWLScopePlot::resizeEvent( QResizeEvent *event )
{
    d_directPainter->reset();
    QwtPlot::resizeEvent( event );
}

void AWLScopePlot::showEvent( QShowEvent * )
{
    replot();
}

bool AWLScopePlot::eventFilter( QObject *object, QEvent *event )
{
    if ( object == canvas() && 
        event->type() == QEvent::PaletteChange )
    {
		for (int i = 0; i < d_curve.size(); i++) 
		{
			d_curve[i]->setPen( canvas()->palette().color( QPalette::WindowText ) );
		}
    }

    return QwtPlot::eventFilter( object, event );
}

void AWLScopePlot::updateCurveDataRaw()

{
	if (d_channelID < 0) return;  // Receiver not assigned correctly yet
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

		// Thread safe
		int detectionQty = d_receiverCapture->GetDetectionQtyPerChannel();
		int detectionIndex = 0;
		int maxDetections = d_curveData.size();
		for (int i = 0; (i < detectionQty) && (i < maxDetections); i++)
		{
			Detection::Ptr detection = Detection::Ptr(new Detection(d_channelID, i));

			d_receiverCapture->GetDetection(frameIndex, d_channelID, i, detection, -1);

			if ((detection->distance >= d_receiverCapture->GetMinDistance()) && 
				(detection->distance <= d_receiverCapture->GetMaxDistance())) 
			{
				// Replace the new point to the end, with detected value
				const QPointF s(elapsed,  detection->distance);
				d_curveData[detectionIndex++]->addValue(s);
			} 
		} // For i;

		// Add empty values to the remaining empty tracks
		for  (int i = detectionIndex; i < maxDetections; i++) 
		{
			const QPointF s(elapsed, 0.0);
			d_curveData[i]->addValue(s);
		} // For i;
	} // For frameIndex

	updateLock.unlock();
}

QwtPlotCurve::CurveStyle AWLScopePlot::getCurveStyle()

{
	if (d_curve.size())
	{
		return(d_curve[0]->style());
	}
	else
	{
		return( QwtPlotCurve::Lines);
	}

}

QwtPlotCurve::CurveStyle AWLScopePlot::setCurveStyle(QwtPlotCurve::CurveStyle inCurveStyle)
{
	for (int i = 0; i < detectionQty; i++) 
	{
		QwtPlotCurve * pCurve = d_curve[i];
		pCurve->setStyle( inCurveStyle );
	}

	return(inCurveStyle);
}

	