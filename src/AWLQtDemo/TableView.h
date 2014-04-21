#ifndef TableView_H
#define TableView_H

#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QAction>
#include <QActionGroup>

#include "ui_TableView.h"

#include "Tracker.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "AWLSettings.h"

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
		eRealTimeDistanceColumn = 3,
		eRealTimeIntensityColumn = 4,
		eRealTimeTrackColumn = 5,
		eRealTimeVelocityColumn = 6,
		eRealTimeCollisionLevelColumn = 7
	};


public:
    explicit TableView(QWidget *parent = 0);


signals:
    void closed();

public slots:
	void ShowContextMenu(const QPoint& pos);
	void slotDetectionsPerChannelAction();
    void slotConfigChanged();
    void slotDetectionDataChanged(DetectionDataVect* data);

protected :
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);

private:
	void PrepareTableViews();
	void DisplayReceiverValues(DetectionDataVect* data);
	void AddDistanceToText(int rowIndex,  QTableWidget *pTable , const Detection *detection);
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
};


} // namespace awl
#endif // TableView_H
