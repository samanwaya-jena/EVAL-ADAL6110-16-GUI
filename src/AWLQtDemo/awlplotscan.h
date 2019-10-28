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

#ifndef AWLPLOTSCAN_H
#define AWLPLOTSCAN_H

#include <QWidget>
#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QAction>
#include <QActionGroup>
#include "boost/chrono/system_clocks.hpp"

#include "ReceiverCapture.h"
#include "ui_awlplotscan.h"
#include "DetectionStruct.h"

//#define USE_FPS_AWLPLOTSCAN

QT_BEGIN_NAMESPACE

namespace awl
{

class AWLPlotScan : public QFrame
{
	Q_OBJECT
public:
	AWLPlotScan(QWidget *parent = 0);
	~AWLPlotScan();
	void start(SensorCoreScope::ReceiverCapture::Ptr inReceiverCapture);
	void stop();
  void setChannelMask(uint32_t chMask);
  void selectReceiver(int receiver);
  void AScanDataChanged(const SensorCoreScope::AScan::Vector& inData);
  void ShowAScan(bool show) { showAScan = show; }
  void SetMaxRange(float maxRange) { m_maxRange = maxRange; }

  QSize sizeHint() const;
  QSize minimumSizeHint() const;
  QSize maximumSizeHint() const;

private:
  bool showAScan;
  SensorCoreScope::AScan::Vector aScanData;
	Ui::AWLPlotScanFrame ui;
	void plotAScans(QPainter* p);
  void PlotAScan(QPainter* p, SensorCoreScope::AScan::Ptr pAscan, int top, int left, int width, int height, float maxRange);
	void LabelAScan(QPainter* p);

signals:
	void closed();
protected :
   void paintEvent(QPaintEvent *p);
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);

  QPoint * m_pPts;
  int m_numPts;

  float m_maxRange;

  uint32_t m_chMask;
  int m_selectedReceiver;
  uint32_t m_nbrCh;

#ifdef USE_FPS_AWLPLOTSCAN
  // FPS
  boost::chrono::time_point<boost::chrono::high_resolution_clock> m_timeFPS;
  int nFrames;
  int FPS;
#endif //USE_FPS_AWLPLOTSCAN
};

} // namespace awl
#endif // AWLPLOTSCAN_H
