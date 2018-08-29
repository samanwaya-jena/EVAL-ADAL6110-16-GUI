
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
const QColor rgbRulerMed(128, 128, 128, 128); // Transparent gray light
const QColor rgbRulerText(255, 170, 0);
const int SCAN_POSX = 0;
const int SCAN_POSY = 0;
const int SCAN_OFFSET_POSY = 50;
const int SCAN_SIGNAL_MAX_HEIGHT = 50;
const int SCAN_GRID_ORIGIN = 15;


AWLPlotScan::AWLPlotScan(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);
	QWidget window;
	//window.setFixedSize(100,100);
	//window.show();
	printf ("PlotScan\n");


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
	printf ("PlotScan start\n");
}




AWLPlotScan::~AWLPlotScan()
{
        stop();
}
	
void AWLPlotScan::stop()
{
     ;
}	

void AWLPlotScan::LabelAScan(int receiver, int channel)
{
	float maxRange, scale;
	int step;
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	maxRange = globalSettings->receiverSettings[0].channelsConfig[0].maxRange / 4;
	if ( maxRange < 10.0 ) {
		step = round((int)(maxRange)/5)*5;
	} else {
		step = round((int)(maxRange)/10)*10;
	}
	
	scale =  width() * ( (float) step / maxRange );

	QPainter painter(this);
	if (!showAScan) return;
	painter.setPen(QPen(rgbRulerMed));
	painter.setBrush(QBrush(rgbRulerMed));
	painter.drawText(scale/4, 10, QString::number(step) + "m");
	painter.drawText(scale*2/4, 10, QString::number(step*2) + "m");
	painter.drawText(scale*3/4, 10, QString::number(step*3) + "m");
	//painter.drawLine(0, SCAN_GRID_ORIGIN , width(), 15);
	painter.drawLine(scale/4, SCAN_GRID_ORIGIN , scale/4, height());
	painter.drawLine(scale*2/4, SCAN_GRID_ORIGIN , scale*2/4, height());
	painter.drawLine(scale*3/4, SCAN_GRID_ORIGIN , scale*3/4, height());

	painter.setBrush(QBrush(rgbRulerMed));
	painter.setPen(QPen(rgbRulerText));
	painter.drawText(SCAN_POSX, SCAN_OFFSET_POSY + 50 * channel, "Ch " + QString::number(receiver+1) + "." + QString::number(channel+1));
	painter.drawLine(SCAN_POSX, SCAN_OFFSET_POSY + 50 * channel, width(), SCAN_OFFSET_POSY + 50 * channel);
	//printf("pixel %d-%d\n", receiver, channel);
}

void AWLPlotScan::PlotAScan(int x1, int y1, int x2, int y2)
{
	QPainter painter(this);
	if (!showAScan) return;
	painter.setPen(QPen(rgbRulerLight));
	painter.setBrush(QBrush(rgbRulerMed));
	painter.drawLine(x1, y1, x2, y2);
}

void AWLPlotScan::plotAScans()
{
  float minFinal  =  INFINITY;
  float maxFinal  = -INFINITY;
  float maxRange = 0.0F;

  BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
  {
    float minV, maxV, meanV;

    aScan->FindMinMaxMean(&minV, &maxV, &meanV);

    if (minV < minFinal)
      minFinal = minV;

    if (maxV > maxFinal)
      maxFinal = maxV;
  }

  // maxRange will contain the peak absolute value for all ascans
  maxRange = abs(maxFinal);

  if (abs(minFinal) > maxRange)
    maxRange = abs(minFinal);

	BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	{
		aScan->Plot(SCAN_OFFSET_POSY + 50 * aScan->channelID, 0, width(), 50, this, maxRange);
		//aScan->Plot(50 + 50 * aScan->channelID + 25 * aScan->receiverID, 0, width(), 50, this);
	}

	update();
}

void AWLPlotScan::paintEvent(QPaintEvent *p)
{
	QPainter painter(this);
	painter.setPen(QPen(rgbRulerLight));
	painter.setBrush(QBrush(rgbRulerMed));
	//painter.drawLine(0, 0, 200, 200);

	plotAScans();
	//printf ("PlotScan paintEvent\n");
}

void AWLPlotScan::closeEvent(QCloseEvent * event)
{
}

void AWLPlotScan::resizeEvent(QResizeEvent * event)
{
	update();
}


