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
QColor speedColor( QColor(0xFF, 0x00, 0xFF)); // Light pink
//QColor speedColor( QColor(0x30, 0xFF, 0x30)); // Semi light green
QColor distanceColor( QColor(255, 170, 0));  // Orange

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

        // QPalette::WindowText was used for the distance curve color
		pal.setColor(QPalette::WindowText, distanceColor);

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
	delete d_directPainter;
}

void AWLScopePlot::setupInterface()

{
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
	enableAxis(QwtPlot::yRight, true);

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
		// Distance curve
		QwtPlotCurve * pDistanceCurve = new QwtPlotCurve();
		CurveData::Ptr pDistanceCurveData = new CurveData();

		pDistanceCurve->setStyle( QwtPlotCurve::Dots );

		pDistanceCurve->setPen(distanceColor, 2 );
		pDistanceCurve->setRenderHint( QwtPlotItem::RenderAntialiased, true );
		pDistanceCurve->setPaintAttribute( QwtPlotCurve::ClipPolygons, false );
		pDistanceCurve->setAxes(QwtPlot::xBottom, QwtPlot::yLeft);
		pDistanceCurve->setData( pDistanceCurveData );
		pDistanceCurve->attach( this );

		d_distanceCurve.append(pDistanceCurve);
		d_distanceCurveData.append(pDistanceCurveData);

		// Velocity curve
		QwtPlotCurve * pVelocityCurve = new QwtPlotCurve();
		CurveData::Ptr pVelocityCurveData = new CurveData();

		pVelocityCurve->setStyle( QwtPlotCurve::Dots );

		pVelocityCurve->setPen( speedColor, 2 );
		pVelocityCurve->setRenderHint( QwtPlotItem::RenderAntialiased, true );
		pVelocityCurve->setPaintAttribute( QwtPlotCurve::ClipPolygons, false );

		pVelocityCurve->setAxes(QwtPlot::xBottom, QwtPlot::yRight);
		pVelocityCurve->setData( pVelocityCurveData );
		pVelocityCurve->attach( this );

		d_velocityCurve.append(pVelocityCurve);
		d_velocityCurveData.append(pVelocityCurveData);
		d_paintedPoints.append(0);
	}

	adjustDisplayedCurves();
}

void AWLScopePlot::adjustDisplayedCurves()

{
	AWLSettings *settings = AWLSettings::GetGlobalSettings();
	// y Scale for velocities
	float maxVelocity =  settings->maxVelocity2D;
	if (settings->velocityUnits != eVelocityUnitsMS)
		maxVelocity = VelocityToKmH(maxVelocity);

	QwtPlot::Axis distanceAxis = QwtPlot::yLeft;
	QwtPlot::Axis velocityAxis = QwtPlot::yRight;
	bool leftAxisVisible = true;
	bool rightAxisVisible = true;

	if (settings->bDisplayScopeDistance && settings->bDisplayScopeVelocity)
	{
		leftAxisVisible = true;
		rightAxisVisible = true;
		distanceAxis = QwtPlot::yLeft;
		velocityAxis = QwtPlot::yRight;
	}
	else if (settings->bDisplayScopeDistance)
	{
		leftAxisVisible = true;
		rightAxisVisible = false;
		distanceAxis = QwtPlot::yLeft;
		velocityAxis = QwtPlot::yRight;
	}
	else if (settings->bDisplayScopeVelocity)
	{
		leftAxisVisible = true;
		rightAxisVisible = false;
		distanceAxis = QwtPlot::yRight;
		velocityAxis = QwtPlot::yLeft;
	}
	else
	{
		leftAxisVisible = false;
		rightAxisVisible = false;
		distanceAxis = QwtPlot::yLeft;
		velocityAxis = QwtPlot::yRight;
	}

	for (int curveIndex = 0; curveIndex < d_distanceCurve.size(); curveIndex++)
	{
		d_distanceCurve.at(curveIndex)->setVisible(settings->bDisplayScopeDistance);
		d_distanceCurve.at(curveIndex)->setAxes(QwtPlot::xBottom, distanceAxis);
	}
		
	// Y Scale for distances
	setAxisScale(distanceAxis, 0, settings->displayedRangeMax);
	
	for (int curveIndex = 0; curveIndex < d_velocityCurve.size(); curveIndex++)
	{
		d_velocityCurve.at(curveIndex)->setVisible(settings->bDisplayScopeVelocity);
		d_velocityCurve.at(curveIndex)->setAxes(QwtPlot::xBottom, velocityAxis);
	}
		
	// Y Scale for velocities
	setAxisScale(velocityAxis, -maxVelocity, maxVelocity);

	enableAxis(QwtPlot::yLeft, leftAxisVisible);
	enableAxis(QwtPlot::yRight,rightAxisVisible);
}


void AWLScopePlot::start(ReceiverCapture::Ptr inReceiverCapture, int inChannelID)
{
 	d_channelID = inChannelID;
	d_receiverCapture = inReceiverCapture;
	d_currentReceiverCaptureSubscriberID = d_receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();

	// Update the plot according to the clock;
	incrementInterval();
}

void AWLScopePlot::replot()
{
	for (int i = 0; i < d_distanceCurve.size(); i++) 
	{
		getDistanceCurveData(i)->values().lock();
	}

	for (int i = 0; i < d_velocityCurve.size(); i++) 
	{
		getVelocityCurveData(i)->values().lock();
	}


	QwtPlot::replot();


	for (int i = 0; i < d_distanceCurve.size(); i++) 
	{
		d_paintedPoints[i] = getDistanceCurveData(i)->size();
		getDistanceCurveData(i)->values().unlock();
	}

	for (int i = 0; i < d_velocityCurve.size(); i++) 
	{
		getVelocityCurveData(i)->values().unlock();
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
    const double elapsed = d_receiverCapture->GetElapsed() / 1000.0;

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
	for (int i = 0; i < d_distanceCurve.size(); i++)
	{
		CurveData *data = getDistanceCurveData(i);
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

				const QwtScaleMap xMap = canvasMap( d_distanceCurve[i]->xAxis() );
				const QwtScaleMap yMap = canvasMap( d_distanceCurve[i]->yAxis() );

				QRectF br = qwtBoundingRect( *data,
					d_paintedPoints[i]  - 1, numPoints - 1 );

				const QRect clipRect = QwtScaleMap::transform( xMap, yMap, br ).toRect();
				d_directPainter->setClipRegion( clipRect );
			}

			d_directPainter->drawSeries( d_distanceCurve[i],
				d_paintedPoints[i] - 1, numPoints - 1 );

			d_directPainter->drawSeries( d_velocityCurve[i],
				d_paintedPoints[i] - 1, numPoints - 1 );


			d_paintedPoints[i] = numPoints;
		}

		data->values().unlock();
	}
}

void AWLScopePlot::incrementInterval()
{
    double elapsed = d_receiverCapture->GetElapsed() / 1000.0;
	elapsed = ceil(elapsed);

	double minValue = ((elapsed - d_interval.width()));

	if (minValue <0.0) minValue = 0.0;
	double maxValue = minValue + d_interval.width();
    d_interval = QwtInterval( minValue, maxValue);


    for (int i = 0; i < d_distanceCurveData.size(); i++) 
	{
		CurveData *data = getDistanceCurveData(i);
		data->values().clearStaleValues( d_interval.minValue() );
	}

    for (int i = 0; i < d_velocityCurveData.size(); i++) 
	{
		CurveData *data = getVelocityCurveData(i);
		data->values().clearStaleValues( d_interval.minValue() );
	}

    setAxisScale( QwtPlot::xBottom, d_interval.minValue(), d_interval.maxValue());
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
		for (int i = 0; i < d_distanceCurve.size(); i++) 
		{
			d_distanceCurve[i]->setPen( distanceColor );
		}

		for (int i = 0; i < d_velocityCurve.size(); i++) 
		{
			d_velocityCurve[i]->setPen( speedColor );
		}
    }

    return QwtPlot::eventFilter( object, event );
}


QwtPlotCurve::CurveStyle AWLScopePlot::getDistanceCurveStyle()

{
	if (d_distanceCurve.size())
	{
		return(d_distanceCurve[0]->style());
	}
	else
	{
		return( QwtPlotCurve::Lines);
	}
}

QwtPlotCurve::CurveStyle AWLScopePlot::setDistanceCurveStyle(QwtPlotCurve::CurveStyle inCurveStyle)
{
	for (int i = 0; i < detectionQty; i++) 
	{
		 d_distanceCurve[i]->setStyle( inCurveStyle );
	}

	return(inCurveStyle);
}


QwtPlotCurve::CurveStyle AWLScopePlot::getVelocityCurveStyle()

{
	if (d_velocityCurve.size())
	{
		return(d_velocityCurve[0]->style());
	}
	else
	{
		return( QwtPlotCurve::Lines);
	}
}

QwtPlotCurve::CurveStyle AWLScopePlot::setVelocityCurveStyle(QwtPlotCurve::CurveStyle inCurveStyle)
{
	for (int i = 0; i < detectionQty; i++) 
	{
		 d_velocityCurve[i]->setStyle( inCurveStyle );
	}

	return(inCurveStyle);
}

#if 0
QwtPlotCurve::SetDistanceVisible(bool bVisible)

{
}

QwtPlotCurve::SetVelocityVisible(bool bVisible)
{
}
#endif