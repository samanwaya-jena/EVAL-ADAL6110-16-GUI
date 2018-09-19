
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
const int SCAN_POSX = 0;
const int SCAN_POSY = 0;
const int SCAN_OFFSET_POSY = 50;
const int SCAN_GRID_ORIGIN = 15;


uint32_t numberOfSetBits(uint32_t i)
{
  // Java: use >>> instead of >>
  // C or C++: use uint32_t
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}



AWLPlotScan::AWLPlotScan(QWidget *parent) :
    QFrame(parent),
  m_chMask(0xFFFF)
{
  m_nbrCh = numberOfSetBits(m_chMask);
	ui.setupUi(this);
}

void AWLPlotScan::start(ReceiverCapture::Ptr inReceiverCapture)
{
}

AWLPlotScan::~AWLPlotScan()
{
        stop();
}
	
void AWLPlotScan::stop()
{
     ;
}

void AWLPlotScan::setChannelMask(uint32_t chMask)
{
  m_chMask = chMask;
  m_nbrCh = numberOfSetBits(m_chMask);
}

void AWLPlotScan::LabelAScan()
{
  float fAscanHeight = 850.0 / (m_nbrCh + 1);

	float maxRange, scale;
	int step;
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	maxRange = globalSettings->receiverSettings[0].channelsConfig[0].maxAscanRange / 5;
	if ( maxRange < 10.0 ) {
		step = ((int)((maxRange+2.5)/5)) * 5;
	} else {
		step = ((int)((maxRange+5)/10)) * 10;
	}
	
	scale =  width() * ( (float) step / maxRange );

	QPainter painter(this);
	if (!showAScan) return;
	painter.setPen(QPen(rgbRulerMed));
	painter.setBrush(QBrush(rgbRulerMed));

	painter.drawText(scale*1/5, 10, QString::number(step*1) + "m");
	painter.drawText(scale*2/5, 10, QString::number(step*2) + "m");
	painter.drawText(scale*3/5, 10, QString::number(step*3) + "m");
  painter.drawText(scale*4/5, 10, QString::number(step*4) + "m");

  //painter.drawLine(0, SCAN_GRID_ORIGIN , width(), 15);
	painter.drawLine(scale*1/5, SCAN_GRID_ORIGIN , scale*1/5, 17 * SCAN_OFFSET_POSY);
	painter.drawLine(scale*2/5, SCAN_GRID_ORIGIN , scale*2/5, 17 * SCAN_OFFSET_POSY);
	painter.drawLine(scale*3/5, SCAN_GRID_ORIGIN , scale*3/5, 17 * SCAN_OFFSET_POSY);
  painter.drawLine(scale*4/5, SCAN_GRID_ORIGIN,  scale*4/5, 17 * SCAN_OFFSET_POSY);
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
  int i = 0;
  int chIdx = 0;
  float minFinal  =  FLT_MAX;
  float maxFinal  = -FLT_MAX;
  float maxRange = 0.0F;
  float fAscanHeight = 850.0 / (m_nbrCh + 1);

  if (!showAScan) return;

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

  LabelAScan();

	BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	{
    if (m_chMask & (1 << i))
    {
      aScan->Plot(fAscanHeight * (chIdx + 1), 0, width(), fAscanHeight, this, maxRange);

      QPainter painter(this);
      painter.setBrush(QBrush(rgbRulerMed));
      painter.setPen(QPen(rgbRulerText));
      painter.drawText(SCAN_POSX, fAscanHeight * (chIdx + 1), "Ch " + QString::number(aScan->receiverID + 1) + "." + QString::number(aScan->channelID + 1));
      painter.drawLine(SCAN_POSX, fAscanHeight * (chIdx + 1), width(), fAscanHeight * (chIdx + 1));

      ++chIdx;
    }

    ++i;
	}

	update();
}

void AWLPlotScan::paintEvent(QPaintEvent *p)
{
	plotAScans();
}

void AWLPlotScan::closeEvent(QCloseEvent * event)
{
}

void AWLPlotScan::resizeEvent(QResizeEvent * event)
{
	update();
}


