#ifndef AWLPLOTSCAN_H
#define AWLPLOTSCAN_H

/*
	Copyright 2014, 2015 Phantom Intelligence Inc.

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
	void start(SENSORCORE_NAMESPACE_PREFIX::ReceiverCapture::Ptr inReceiverCapture);
	void stop();
  void setChannelMask(uint32_t chMask);
  void selectReceiver(int receiver);
  void AScanDataChanged(const SENSORCORE_NAMESPACE_PREFIX::AScan::Vector& inData);
  void ShowAScan(bool show) { showAScan = show; }
  void SetMaxRange(float maxRange) { m_maxRange = maxRange; }

  QSize sizeHint() const;
  QSize minimumSizeHint() const;
  QSize maximumSizeHint() const;

private:
  bool showAScan;
  SENSORCORE_NAMESPACE_PREFIX::AScan::Vector aScanData;
	Ui::AWLPlotScanFrame ui;
	void plotAScans(QPainter* p);
  void PlotAScan(QPainter* p, SENSORCORE_NAMESPACE_PREFIX::AScan::Ptr pAscan, int top, int left, int width, int height, float maxRange);
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
