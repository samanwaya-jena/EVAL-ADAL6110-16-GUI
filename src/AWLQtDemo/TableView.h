#ifndef TableView_H
#define TableView_H

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

	typedef enum RealTimeColumn
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
    void slotDetectionDataChanged(const Detection::Vector &data);

protected :
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);

private:
	QSize unconstrainedTableSize() const;

	void PrepareTableViews();
	void AdjustTableSize();

	void DisplayReceiverValues(const Detection::Vector &data);
	void AddDistanceToText(int rowIndex,  QTableWidget *pTable , const Detection::Ptr &detection);
	void AddDistanceToText(int rowIndex, QTableWidget *pTable,
						   int receiverID,
						   int channelID,
						   int detectionID, 
						   TrackID trackID = 0, 
						   float distance = NAN, 
						   Detection::ThreatLevel level = Detection::eThreatNone, 
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
