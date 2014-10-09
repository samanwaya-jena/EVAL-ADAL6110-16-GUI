#ifndef AWLCURVEDATA_H
#define AWLCURVEDATA_H
/*
	Copyright 2014 Aerostar R&D Canada Inc.

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