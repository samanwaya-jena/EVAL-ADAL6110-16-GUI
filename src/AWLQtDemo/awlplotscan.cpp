
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
#include <QScrollBar>

#include <boost/foreach.hpp>


#include "awlcoord.h"
#include "AWLSettings.h"

#include <math.h>

using namespace awl;
const QColor rgbRulerLight(255, 255, 255, 128); // Transparent gray light
const QColor rgbRulerMed(128, 128, 128, 128); // Transparent gray light
const QColor rgbRulerText(255, 170, 0);



AWLPlotScan::AWLPlotScan(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);
	//ui->scrollArea->setWidgetResizable(true);
	//QGridLayout *gridLayout = new QGridLayout(ui->scrollAreaWidget);
	//ui->frame->setFixedHeight(ui->scrollArea->height());
	//gridLayout->addWidget(ui->frame,0,0,1,1);
	//ui.AWLPlotScan->setVerticalScrollBarPolicy(QFrame::ScrollBarAsNeeded);
	//QWidget window;
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
	QPainter painter(this);
	if (!showAScan) return;
	painter.setBrush(QBrush(rgbRulerMed));
	painter.setPen(QPen(rgbRulerText));
	painter.drawText(0, 50 + 50 * channel + 25 * receiver, "Ch " + QString::number(receiver) + "-" + QString::number(channel+1));
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
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int receiverCount = globalSettings->receiverSettings.size();
	//printf("Receiver  %d\n", receiverCount);
	BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	{
		aScan->Plot(50 + 50 * aScan->channelID + 25 * aScan->receiverID, 0, width(), 50, this);
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


