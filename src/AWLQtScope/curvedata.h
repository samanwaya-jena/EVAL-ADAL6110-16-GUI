#ifndef AWLCURVEDATA_H
#define AWLCURVEDATA_H

#include <qwt_series_data.h>

class SignalData;

class CurveData: public QwtSeriesData<QPointF>
{
public:
	typedef CurveData * Ptr;
	typedef QVector<CurveData::Ptr> Vector;
	typedef QVector<CurveData::Vector *> Array;

public:
	CurveData();

    const SignalData &values() const;
    SignalData &values();

    virtual QPointF sample( size_t i ) const;
    virtual size_t size() const;

    virtual QRectF boundingRect() const;

	QPointF CurveData::addValue(double x, double y);
	QPointF CurveData::addValue(const QPointF &point);

protected:
	SignalData	*signalData;
};

#endif