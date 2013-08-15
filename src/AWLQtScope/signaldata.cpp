#include "signaldata.h"
#include <qvector.h>
#include <qmutex.h>
#include <qreadwritelock.h>

class SignalData::PrivateData
{
public:
    PrivateData():
        boundingRect( 1.0, 1.0, -2.0, -2.0 ) // invalid
    {
        values.reserve( 1000 );
    }

    inline void append( const QPointF &sample )
    {
        values.append( sample );

        // adjust the bounding rectangle

        if ( boundingRect.width() < 0 || boundingRect.height() < 0 )
        {
            boundingRect.setRect( sample.x(), sample.y(), 0.0, 0.0 );
        }
        else
        {
            boundingRect.setRight( sample.x() );

            if ( sample.y() > boundingRect.bottom() )
                boundingRect.setBottom( sample.y() );

            if ( sample.y() < boundingRect.top() )
                boundingRect.setTop( sample.y() );
        }
    }

    QReadWriteLock lock;

    QVector<QPointF> values;
    QRectF boundingRect;

    QMutex mutex; // protecting pendingValues
    QVector<QPointF> pendingValues;
};

SignalData::SignalData()
{
    d_data = new PrivateData();
}

SignalData::~SignalData()
{
    delete d_data;
}

int SignalData::size() const
{
    return d_data->values.size();
}

QPointF SignalData::value( int index ) const
{
    return d_data->values[index];
}

QRectF SignalData::boundingRect() const
{
    return d_data->boundingRect;
}

void SignalData::lock()
{
    d_data->lock.lockForRead();
}

void SignalData::unlock()
{
    d_data->lock.unlock();
}

void SignalData::append( const QPointF &sample )
{
    d_data->mutex.lock();
    d_data->pendingValues += sample;

    const bool isLocked = d_data->lock.tryLockForWrite();
    if ( isLocked )
    {
        const int numValues = d_data->pendingValues.size();
        const QPointF *pendingValues = d_data->pendingValues.data();

        for ( int i = 0; i < numValues; i++ )
            d_data->append( pendingValues[i] );

        d_data->pendingValues.clear();

        d_data->lock.unlock();
    }

    d_data->mutex.unlock();
}

void SignalData::clearStaleValues( double limit )
{
	 d_data->mutex.lock();
 
    d_data->lock.lockForWrite();
    d_data->boundingRect = QRectF( 1.0, 1.0, -2.0, -2.0 ); // invalid

    int index;
    for ( index = d_data->values.size() - 1; index >= 0; index-- )
    {
        if ( d_data->values[index].x() < limit )
            break;
    }

	// If we have index == -1: Whole array, do nothing
	if (index >= 0) 
	{
		int destOffset = 0;
		index++;
		while ( index <= d_data->values.size() - 1 ) 
		{
			d_data->values[destOffset] = d_data->values[index++];
			// Adjust the bounding rect yValues
			qreal y = d_data->values[destOffset].y();
			if ( y > d_data->boundingRect.bottom() )
                d_data->boundingRect.setBottom(y);

            if ( y < d_data->boundingRect.top() )
                d_data->boundingRect.setTop( y );

			destOffset++;
		}

		// Rezize the array 
		d_data->values.resize(destOffset);


		// If array not empty, adjust the left - right values for bounding rect
		if (destOffset > 0)
		{
			d_data->boundingRect.setRight(d_data->values[d_data->values.size() - 1].x() );
			d_data->boundingRect.setLeft(d_data->values[0].x() );
        }

	}

  d_data->lock.unlock();
  d_data->mutex.unlock();
}


