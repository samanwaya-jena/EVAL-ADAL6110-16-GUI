#include "awlqtscope.h"

#include <QtWidgets/QApplication>
#include <qapplication.h>

#include "samplingthread.h"
#include "AWLScopePlot.h"
#include "curvedata.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
    app.setPalette( Qt::darkGray );

    AWLQtScope window;
//    window.resize( 800, 400 );

    SamplingThread samplingThread(&window, window.getCurveData());

    samplingThread.setAmplitude( window.amplitude() );
    samplingThread.setInterval( window.signalInterval() );

    window.connect( &window, SIGNAL( frequencyChanged( double ) ),
        &samplingThread, SLOT( setFrequency( double ) ) );
    window.connect( &window, SIGNAL( amplitudeChanged( double ) ),
        &samplingThread, SLOT( setAmplitude( double ) ) );
    window.connect( &window, SIGNAL( signalIntervalChanged( double ) ),
        &samplingThread, SLOT( setIntervalDelay( double ) ) );

    window.show();

    samplingThread.start();
    window.start();

    bool ok = app.exec();

    samplingThread.stop();
    samplingThread.wait( 1000 );

    return ok;
}
