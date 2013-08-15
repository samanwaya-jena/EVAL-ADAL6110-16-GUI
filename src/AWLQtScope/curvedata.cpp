#include "curvedata.h"
#include "signaldata.h"

CurveData::CurveData():
QwtSeriesData<QPointF>()

{
	signalData = new SignalData();
}

const SignalData &CurveData::values() const
{
    return *signalData;
}

SignalData &CurveData::values()
{
    return *signalData;
}

QPointF CurveData::sample( size_t i ) const
{
    return signalData->value( i );
}

size_t CurveData::size() const
{
    return signalData->size();
}

QRectF CurveData::boundingRect() const
{
    return signalData->boundingRect();
}

QPointF CurveData::addValue(double x, double y)

{
	    const QPointF s(x, y );
        signalData->append( s );
		return(s);
}

QPointF CurveData::addValue(const QPointF &point)

{
        signalData->append( point );
		return(point);
}
