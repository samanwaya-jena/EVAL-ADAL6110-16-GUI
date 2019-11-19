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
#ifndef TableView_H
#define TableView_H

#include <QFrame>
#include <QAction>
#include <QActionGroup>

#include "ui_TableView.h"
#include "DetectionStruct.h"
#include <boost/container/vector.hpp>

namespace awl
{

class TableView : public QFrame
{
    Q_OBJECT
public:

	enum RealTimeColumn
	{
		eRealTimeReceiverIDColumn = 0,
		eRealTimeChannelIDColumn = 1,
		eRealTimeDetectionIDColumn = 2,
		eRealTimeTrackColumn = 3,
		eRealTimeDistanceColumn = 4,
		eRealTimeVelocityColumn = 5, 
		eRealTimeIntensityColumn = 6,
		eRealTimeCollisionLevelColumn = 7
	};


public:
    explicit TableView(QWidget *parent = 0);
	QSize sizeHint() const;
	QSize minimumSizeHint() const;
	QSize maximumSizeHint() const;

signals:
    void closed();

public slots:
	void ShowContextMenu(const QPoint& pos);
	void slotDetectionsPerChannelAction();
    void slotConfigChanged();
    void slotDetectionDataChanged(const SensorCoreScope::Detection::Vector &data);

protected :
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);

private:
	QSize unconstrainedTableSize() const;

	void PrepareTableViews();
	void AdjustTableSize();

	void DisplayReceiverValues(const SensorCoreScope::Detection::Vector &data);
	void AddDistanceToText(int rowIndex,  QTableWidget *pTable , const Detection::Ptr &detection);
	void AddDistanceToText(int rowIndex, QTableWidget *pTable,
						   int receiverID,
						   CellID cellID,
						   int detectionID, 
						   TrackID trackID = 0, 
						   float distance = NAN, 
						  SensorCoreScope::AlertCondition::ThreatLevel level =SensorCoreScope::AlertCondition::eThreatNone,
						   float intensity = NAN,
						   float velocity = NAN,
						   float acceleration = NAN, 
						   float timeToCollision = NAN,
						   float decelerationToStop = NAN,
						   float probability = 0);


private:
	void createAction();
	//Action Item
	Ui::AWLTableView ui;

	QActionGroup* groupDetectionsPerChannel;
	QAction* detectionsPerChannel1Action;
	QAction* detectionsPerChannel2Action;
	QAction* detectionsPerChannel4Action;
	QAction* detectionsPerChannel8Action;

	int displayedDetectionsPerChannel;
	boost::container::vector<int> receiverFirstRow;
};


} // namespace awl
#endif // TableView_H
