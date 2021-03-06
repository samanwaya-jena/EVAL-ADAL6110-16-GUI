
/* awlplotscan.cpp */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the CuteApplication of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/

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


#include "SensorCoord.h"
#include "AWLSettings.h"

#include <math.h>

using namespace awl;
SENSORCORE_USE_NAMESPACE

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
  m_selectedReceiver(0),
  m_maxRange(0.0F),
  m_numPts(0),
  m_pPts(NULL)
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

void AWLPlotScan::setVoxelMask(uint32_t chMask)
{
  m_chMask = chMask;
  m_nbrCh = numberOfSetBits(m_chMask);
}

void AWLPlotScan::selectReceiver(int receiver)
{
  m_selectedReceiver = receiver;
}

void AWLPlotScan::LabelAScan(QPainter* p)
{
  float fAscanHeight = float(height()) / (m_nbrCh + 1);

	float maxRange, scale;
	int step;
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	maxRange = globalSettings->receiverSettings[0].voxelsConfig[0].maxAscanRange / 5;
	if ( maxRange < 10.0 ) {
		step = ((int)((maxRange+2.5)/5)) * 5;
	} else {
		step = ((int)((maxRange+5)/10)) * 10;
	}
	
	scale =  width() * ( (float) step / maxRange );

	p->setPen(QPen(rgbRulerMed));
	p->setBrush(QBrush(rgbRulerMed));

	p->drawText(scale*1/5, 10, QString::number(step*1) + "m");
	p->drawText(scale*2/5, 10, QString::number(step*2) + "m");
	p->drawText(scale*3/5, 10, QString::number(step*3) + "m");
  p->drawText(scale*4/5, 10, QString::number(step*4) + "m");

  //p->drawLine(0, SCAN_GRID_ORIGIN , width(), 15);
	p->drawLine(scale*1/5, SCAN_GRID_ORIGIN , scale*1/5, 17 * fAscanHeight);
	p->drawLine(scale*2/5, SCAN_GRID_ORIGIN , scale*2/5, 17 * fAscanHeight);
	p->drawLine(scale*3/5, SCAN_GRID_ORIGIN , scale*3/5, 17 * fAscanHeight);
  p->drawLine(scale*4/5, SCAN_GRID_ORIGIN,  scale*4/5, 17 * fAscanHeight);
}

void AWLPlotScan::PlotAScan(QPainter* p, AScan::Ptr pAscan, int top, int left, int width, int height, float maxRange)
{
  int16_t *b16;
  int32_t *b32;
  float xScaleFactor = 0.0;
  float yScaleFactor = 0.0;
  int32_t x1, y1;
  int i;

  p->setPen(QPen(rgbRulerLight));
  p->setBrush(QBrush(rgbRulerMed));

  if (pAscan->samples) {
    x1 = left;
    y1 = top;
    //printf ("%d %d\n", sampleCount, width);
    if (pAscan->sampleCount > static_cast<size_t>(width)) {
      if (width > m_numPts)
      {
        m_numPts = width;
        m_pPts = (QPoint*) malloc(m_numPts * sizeof(QPoint));
      }

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
        m_pPts[x].setX(x1);
        m_pPts[x].setY(y1);
      }

      p->drawPolyline(m_pPts, width);
    }
    else {
      if (pAscan->sampleCount > static_cast<size_t>(m_numPts))
      {
        m_numPts = pAscan->sampleCount;
        m_pPts = (QPoint*)malloc(m_numPts * sizeof(QPoint));
      }

      if (pAscan->sampleCount)
        xScaleFactor = (float)width / (float)pAscan->sampleCount;

      yScaleFactor = height / maxRange;

      b16 = (int16_t *)(pAscan->samples);
      b32 = (int32_t *)(pAscan->samples);

      for (size_t x = 0; x < pAscan->sampleCount; x++) {
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
        m_pPts[x].setX(x1);
        m_pPts[x].setY(y1);
      }

      p->drawPolyline(m_pPts, pAscan->sampleCount);
    }
  }
}

void AWLPlotScan::AScanDataChanged(const AScan::Vector& inData)
{
  if (inData.size() > 0)
  {
    aScanData = inData;
    update();
  }
}

void AWLPlotScan::plotAScans(QPainter* p)
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

  // Text formatting info
  int lineSpacing = p->fontMetrics().lineSpacing();
  int leading = p->fontMetrics().leading();

  if (m_maxRange)
  {
    maxRange = m_maxRange;
  }
  else
  {
    i = 0;
    BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
    {
      if (aScan->receiverID == m_selectedReceiver && m_chMask & (1 << aScan->cellID.column))
      {
        float minV, maxV, meanV;

        aScan->FindMinMaxMean(&minV, &maxV, &meanV);

        if (minV < minFinal)
          minFinal = minV;

        if (maxV > maxFinal)
          maxFinal = maxV;
      }

      ++i;
    }

    // maxRange will contain the peak absolute value for all ascans
    maxRange = abs(maxFinal);

    if (abs(minFinal) > maxRange)
      maxRange = abs(minFinal);
  }

  LabelAScan(p);

  i = 0;
	BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	{
    if (aScan->receiverID == m_selectedReceiver && m_chMask & (1 << aScan->cellID.column))
    {
      PlotAScan(p, aScan, fAscanHeight * (chIdx + 1), 0, width(), fAscanHeight, maxRange);

      p->setBrush(QBrush(rgbRulerMed));
      p->setPen(QPen(rgbRulerText));
	  p->drawText(SCAN_POSX, fAscanHeight * (chIdx + 1) - (leading/2) - 1, "Rcv " + QString::number(aScan->receiverID + 1) + " Col " + QString::number(aScan->cellID.column));
	  p->drawText(SCAN_POSX, fAscanHeight * (chIdx + 1) + lineSpacing - (leading/2) -1, "Channel " + QString::number(aScan->channelNo));
	  p->drawLine(SCAN_POSX, fAscanHeight * (chIdx + 1), width(), fAscanHeight * (chIdx + 1));

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

  p->setPen(QPen(rgbRulerLight));
  p->setBrush(QBrush(rgbRulerMed));
  p->drawText(10, 10, QString::number(FPS) + " FPS");
#endif //USE_FPS_AWLPLOTSCAN

}

void AWLPlotScan::paintEvent(QPaintEvent *p)
{
  Q_UNUSED(p);

  QPainter painter(this);

  plotAScans(&painter);
}

void AWLPlotScan::closeEvent(QCloseEvent * event)
{
	Q_UNUSED(event);
  emit closed();
}

void AWLPlotScan::resizeEvent(QResizeEvent * event)
{
	Q_UNUSED(event);
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
