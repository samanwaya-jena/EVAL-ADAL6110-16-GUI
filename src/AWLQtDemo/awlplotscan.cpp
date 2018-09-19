
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
  m_chMask(0xFFFF),
  m_maxRange(0.0F)
#ifdef USE_FPS_AWLPLOTSCAN
  , nFrames(0)
  ,FPS(0)
#endif //USE_FPS_AWLPLOTSCAN
{
  m_nbrCh = numberOfSetBits(m_chMask);
	ui.setupUi(this);

#ifdef USE_FPS_AWLPLOTSCAN
  m_timeFPS = boost::chrono::high_resolution_clock::now();
#endif //USE_FPS_AWLPLOTSCAN

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
  float fAscanHeight = float(height()) / (m_nbrCh + 1);

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
	painter.drawLine(scale*1/5, SCAN_GRID_ORIGIN , scale*1/5, 17 * fAscanHeight);
	painter.drawLine(scale*2/5, SCAN_GRID_ORIGIN , scale*2/5, 17 * fAscanHeight);
	painter.drawLine(scale*3/5, SCAN_GRID_ORIGIN , scale*3/5, 17 * fAscanHeight);
  painter.drawLine(scale*4/5, SCAN_GRID_ORIGIN,  scale*4/5, 17 * fAscanHeight);
}

QPoint pts[100];

void AWLPlotScan::PlotAScan(AScan::Ptr pAscan, int top, int left, int width, int height, float maxRange)
{
  int16_t *b16;
  int32_t *b32;
  float xScaleFactor = 0.0;
  float yScaleFactor = 0.0;
  int32_t x1, y1;
  int i;

  QPainter painter(this);
  if (!showAScan) return;
  painter.setPen(QPen(rgbRulerLight));
  painter.setBrush(QBrush(rgbRulerMed));

  if (pAscan->samples) {
    x1 = left;
    y1 = top;
    //printf ("%d %d\n", sampleCount, width);
    if (pAscan->sampleCount > width) {
      if (width)
        xScaleFactor = (float)pAscan->sampleCount / (float)width;

      yScaleFactor = height / maxRange;

      b16 = (int16_t *)(pAscan->samples);
      b32 = (int32_t *)(pAscan->samples);
      for (int x = 0; x < width; x++) {
        x1 = left + x;
        i = x * xScaleFactor;
        switch (pAscan->sampleSize) {
        default:
          return;
        case 2:
          y1 = top - b16[i + pAscan->sampleOffset] * yScaleFactor;
          break;
        case 4:
          y1 = top - b32[i + pAscan->sampleOffset] * yScaleFactor;
          break;
        }
        pts[x].setX(x1);
        pts[x].setY(y1);
      }

      painter.drawPolyline(pts, width);
    }
    else {
      if (pAscan->sampleCount)
        xScaleFactor = (float)width / (float)pAscan->sampleCount;

      yScaleFactor = height / maxRange;

      b16 = (int16_t *)(pAscan->samples);
      b32 = (int32_t *)(pAscan->samples);

      for (int x = 0; x < pAscan->sampleCount; x++) {
        i = x * xScaleFactor;
        x1 = left + i;
        switch (pAscan->sampleSize) {
        default:
          return;
        case 2:
          y1 = top - (b16[x + pAscan->sampleOffset] * yScaleFactor);
          break;
        case 4:
          y1 = top - (b32[x + pAscan->sampleOffset] * yScaleFactor);
          break;
        }
        pts[x].setX(x1);
        pts[x].setY(y1);
      }

      painter.drawPolyline(pts, pAscan->sampleCount);
    }
  }
}

void AWLPlotScan::plotAScans()
{
  int i = 0;
  int chIdx = 0;

  if (!showAScan) return;

#ifdef USE_FPS_AWLPLOTSCAN
  ++nFrames;
#endif //USE_FPS_AWLPLOTSCAN

  float minFinal  =  FLT_MAX;
  float maxFinal  = -FLT_MAX;
  float maxRange = 0.0F;
  float fAscanHeight = float(height()) / (m_nbrCh + 1);

  if (!showAScan) return;

  if (m_maxRange)
  {
    maxRange = m_maxRange;
  }
  else
  {
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
  }

  LabelAScan();

	BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	{
    if (m_chMask & (1 << i))
    {
      PlotAScan(aScan, fAscanHeight * (chIdx + 1), 0, width(), fAscanHeight, maxRange);

      QPainter painter(this);
      painter.setBrush(QBrush(rgbRulerMed));
      painter.setPen(QPen(rgbRulerText));
      painter.drawText(SCAN_POSX, fAscanHeight * (chIdx + 1), "Ch " + QString::number(aScan->receiverID + 1) + "." + QString::number(aScan->channelID + 1));
      painter.drawLine(SCAN_POSX, fAscanHeight * (chIdx + 1), width(), fAscanHeight * (chIdx + 1));

      ++chIdx;
    }

    ++i;
	}

#ifdef USE_FPS_AWLPLOTSCAN
  boost::chrono::duration<double> elapsed = t1 - m_timeFPS;

  if (elapsed.count() > 1.0)
  {
    FPS = nFrames;
    nFrames = 0;
    m_timeFPS = t1;
  }

  QPainter painter(this);
  painter.setPen(QPen(rgbRulerLight));
  painter.setBrush(QBrush(rgbRulerMed));
  painter.drawText(10, 10, QString::number(FPS) + " FPS");
#endif //USE_FPS_AWLPLOTSCAN

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

QSize AWLPlotScan::sizeHint() const
{
  return (maximumSizeHint());
}

QSize AWLPlotScan::minimumSizeHint() const
{
  return(QSize(230, 300));
}

QSize AWLPlotScan::maximumSizeHint() const
{
  QRect scr = QApplication::desktop()->availableGeometry();
  float maxHeight = scr.height() * 1;
  float maxWidth = scr.width()*0.4;
  return(QSize((int)maxWidth, (int)maxHeight));
}
