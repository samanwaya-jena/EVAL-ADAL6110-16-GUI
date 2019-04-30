#include "samplingthread.h"
#include "signaldata.h"
#include "curvedata.h"
#include "ReceiverCapture.h"
#include "Tracker.h"

#include <qwt_math.h>
#include <math.h>

using namespace std;
using namespace awl;

#if QT_VERSION < 0x040600
#define qFastSin(x) ::sin(x)
#endif


SamplingThread::SamplingThread( QObject *parent, CurveData::Array *curveData, ReceiverCapture::Ptr receiverCapture):
    QwtSamplingThread( parent ),
	d_curveData(curveData),
	d_receiverCapture(receiverCapture),
    d_frequency( 5.0 ),
    d_amplitude( 20.0 ),
	lastFrameID(-1)
{
	currentReceiverCaptureSubscriberID = d_receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();
}

void SamplingThread::setFrequency( double frequency )
{
    d_frequency = frequency;
}

double SamplingThread::frequency() const
{
    return d_frequency;
}

void SamplingThread::setAmplitude( double amplitude )
{
    d_amplitude = amplitude;
}

void SamplingThread::setIntervalDelay( double intervalInMillisec )
{
    setInterval(intervalInMillisec);
}

double SamplingThread::amplitude() const
{
    return d_amplitude;
}

void SamplingThread::sample( double elapsed )
{
    if ( d_frequency > 0.0 )
    {
		updateSamples();
    }
}

void SamplingThread::updateSamples()

{
	uint32_t currentFrame = d_receiverCapture->GetFrameID();
	if (!d_receiverCapture->GetFrameQty()) return; 
	if (currentFrame == lastFrameID) return;

	for (int channelID = 0; channelID < d_receiverCapture->GetChannelQty(); channelID++) 
	{
		ChannelFrame::Ptr channelFrame(new ChannelFrame(channelID));

		// Thread safe
		// The UI thread "Snaps" the frame ID for all other interface objects to display
		if (d_receiverCapture->CopyCurrentReceiverChannelData(channelID, channelFrame, currentReceiverCaptureSubscriberID)) 
		{
			double elapsed = channelFrame->timeStamp;
			int detectionQty = channelFrame->detections.size();
			int detectionIndex = 0;
			for (int i = 0; (i < detectionQty) && i < d_curveData[channelID].size(); i++)
			{
				Detection::Ptr detection = channelFrame->detections.at(i);

				if ((detection->distance >= d_receiverCapture->GetMinDistance()) && 
					(detection->distance <= d_receiverCapture->GetMaxDistance())) 
				{
					// Replace the new point to the end, with detected value
					const QPointF s(elapsed,  detection->distance);
					((*d_curveData)[channelID])->at(detectionIndex++)->addValue(s);
				}
			} // For i;

			// Add empty values to the remaining empty tracks
			for  (int i = detectionIndex; i < d_curveData[channelID].size(); i++) 
			{
				const QPointF s(elapsed, 0.0);
				((*d_curveData)[channelID])->at(detectionIndex++)->addValue(s);
			} // For i;

		} // if (..Copy ReceiverData..)
	} //For (channelID)
}

