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
