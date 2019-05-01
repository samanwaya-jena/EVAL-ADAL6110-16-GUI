#include <qwt_sampling_thread.h>
#include "curvedata.h"
#include "ReceiverCapture.h"


using namespace std;
using namespace awl;

class SamplingThread: public QwtSamplingThread
{
    Q_OBJECT

public:
    SamplingThread( QObject *parent, CurveData::Array *curveData, ReceiverCapture::Ptr receiverCapture);

    double frequency() const;
    double amplitude() const;

public Q_SLOTS:
    void setAmplitude( double );
    void setFrequency( double );
	void setIntervalDelay( double intervalInMillisec );

protected:
    virtual void sample( double elapsed );
	void updateSamples();


private:
    double d_frequency;
    double d_amplitude;
	CurveData::Array * d_curveData;
	ReceiverCapture::Ptr d_receiverCapture;
	uint32_t lastFrameID;
	/** \brief Our subscription identifier to access to lidar data. */
	Subscription::SubscriberID currentReceiverCaptureSubscriberID;
};
