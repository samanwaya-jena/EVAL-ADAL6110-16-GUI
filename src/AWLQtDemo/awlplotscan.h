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
#include "ReceiverCapture.h"
#include "ui_awlplotscan.h"
#include "DetectionStruct.h"

namespace awl
{

class AWLPlotScan : public QFrame, public AScanPlotter
{
	Q_OBJECT
public:
	AWLPlotScan(QWidget *parent = 0);
	~AWLPlotScan();
	void start(ReceiverCapture::Ptr inReceiverCapture);
	void stop();
private:
	Ui::AWLPlotScanFrame *ui;
	void plotAScans(void);
	void PlotAScan(int x1, int y1, int x2, int y2);

signals:
	void closed();
protected :
   void paintEvent(QPaintEvent *p);
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);
};

} // namespace awl
#endif // AWLPLOTSCAN_H
