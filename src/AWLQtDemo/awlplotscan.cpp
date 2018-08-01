
/* Fov_2DScan.cpp */
/*
	Copyright (C) 2014, 2015  Phantom Intelligence Inc.

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

#include "awlplotscan.h"

#include "DetectionStruct.h"

#include <QWidget>
#include <QPainter>
#include <QLabel>
#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>
#include <QEvent>

#include <boost/foreach.hpp>


#include "awlcoord.h"
#include "AWLSettings.h"

#include <math.h>

using namespace awl;
const QColor rgbRulerLight(255, 255, 255, 128); // Transparent gray light



AWLPlotScan::AWLPlotScan(QWidget *parent) :
    QFrame(parent)
{
	//ui.setupUi(this);
	QWidget window;
	window.setFixedSize(200,200);
	window.show();


}
void AWLPlotScan::start(ReceiverCapture::Ptr inReceiverCapture)
{
	/*
        d_receiverCapture = inReceiverCapture;
        d_receiverCaptureSubscriberID = d_receiverCapture->Subscribe();

        for (int i = 0; i < d_plot.size(); i++)
        {
                d_plot[i]->start(d_receiverCapture, i);
        }

        if (d_timerId == 0) d_timerId = startTimer( timerInterval );
	*/
}




AWLPlotScan::~AWLPlotScan()
{
        stop();
}
	
void AWLPlotScan::stop()
{
     ;
}	


void AWLPlotScan::plotAScans()
{
    QPainter painter(this);
	painter.setPen(QPen(rgbRulerLight));

	BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	{
		plotAScan(aScan, &painter, 100 + 50 * aScan->channelID, 100, aScan->sampleCount, 50);
	}

    update();
}

void AWLPlotScan::plotAScan(AScan::Ptr aScan, QPainter *painter, int top, int left, int width, int height)
	
{
	int32_t *b32;
	float scaleFactor;
	int32_t x1, y1, x2, y2 = 0;

	if (aScan->samples) {
		x1 = left;
		y1 = top;
		scaleFactor = aScan->GetScaleFactorForRange(height);
		b32 = (int32_t *)(aScan->samples);
		for (int i = aScan->sampleOffset; i < aScan->sampleCount; i ++) {
			x2 = left + i;
			y2 = top + b32[i + aScan->sampleOffset] * scaleFactor;
			painter->drawLine(x1, y1, x2, y2);
			x1 = x2;
			y1 = y2;
		}
	}
}

void AWLPlotScan::slotDetectionDataChanged(const Detection::Vector& data)
{
	/*
    Detection::Vector::const_iterator i;
    int index;

        // Make a copy of the provided Detection::Vector to work with
    copyData.clear();
        copyData = data;


        //Ordering detection from bottom left to top right of 2D view. It's to simplify algo to draw detection in view.
        std::sort(copyData.begin(), copyData.end(), sortDetectionsBottomRightTopLeft);

        //Merge detections according to distance criteria
        mergeDetection();
    update();

    */
}


