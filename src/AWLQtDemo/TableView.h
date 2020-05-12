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

/** \brief Window displays detection and track data in tabular form.
*/
class TableView : public QFrame
{
    Q_OBJECT
public:

	/** \brief enum Column IDs.
	*/
	enum RealTimeColumn
	{
		/**Column for Receiver ID*/
		eRealTimeReceiverIDColumn = 0,
		/**Column for Channel ID*/
		eRealTimeChannelIDColumn = 1,
		/** Column for Detection ID*/
		eRealTimeDetectionIDColumn = 2,
		/**Column for Track*/
		eRealTimeTrackColumn = 3,
		/**Column for Distance*/
		eRealTimeDistanceColumn = 4,
		/**Column for Velocity*/
		eRealTimeVelocityColumn = 5,
		/**Column for Intensity*/
		eRealTimeIntensityColumn = 6,
		/**Column for Collision Level*/
		eRealTimeCollisionLevelColumn = 7
	};


public:
	/** \brief Constructor.
	*/
    explicit TableView(QWidget *parent = 0);

	/** \brief  Used by Qt in resizing, provides "ideal" size for the A-Scan Window.
		*/
	QSize sizeHint() const;
	/** \brief  Used by Qt in resizing, provides "minimum" acceptable size for the A-Scan Window.
	 */
	QSize minimumSizeHint() const;
	/** \brief  Used by Qt in resizing, provides "maximum" size for the A-Scan Window.
	 */
	QSize maximumSizeHint() const;

signals:
    void closed();

public slots:
	/** \brief  Slot sets-up and shows the right-click menu.
	*/
	void ShowContextMenu(const QPoint& pos);

	/** \brief  Update the table parameters for  detections par channel, following the right-click menu selection.
	*/
	void slotDetectionsPerChannelAction();

	/** \brief  Update the table with new detections data following any configuration change.
	*/
	void slotConfigChanged();

	/** \brief  Update the table with new detections data.
	*/
    void slotDetectionDataChanged(const SensorCoreScope::Detection::Vector &data);

protected :
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);

private:
	/** \brief  Calculate and returb the "ideal" table size, in preparation for resize*/
	QSize unconstrainedTableSize() const;


	/** \brief  Prepare the table contents, based on current configuration*/
	void PrepareTableViews();
	/** \brief  Force recalculation of size preferences.  Used after a change in table configuration.*/
	void AdjustTableSize();

	/** \brief  Display current receiver data*/
	void DisplayReceiverValues(const SensorCoreScope::Detection::Vector &data);

	/** \brief  Format distance text and color of all cells of row at rowIndex, for the specified row in the table, using provided detection*/
	void AddDistanceToText(int rowIndex,  QTableWidget *pTable , const Detection::Ptr &detection);

	/** \brief  Format distance text and color for all cells at rowIndex, using the the provided detailed information*/
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

	/** \brief  Internal variable holds current configuration for detections per Channel*/
	int displayedDetectionsPerChannel;

	/** \brief  Array that contains index of the first row for each receiver*/
	boost::container::vector<int> receiverFirstRow;
};


} // namespace awl
#endif // TableView_H
