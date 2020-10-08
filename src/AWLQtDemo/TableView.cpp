/* TableView.cpp : Display detection characteristics in a table*/
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

#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>
#include <QTableWidget>
#include <QScrollBar>
#include <QListWidget>

#include <boost/foreach.hpp>

#include "AWLSettings.h"
#include "TableView.h"
#include "DetectionStruct.h"

//using namespace std;
using namespace awl;
SENSORCORE_USE_NAMESPACE

TableView::TableView(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);
	setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	ui.distanceTable->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded); 
	setWindowIcon(QApplication::windowIcon());

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	displayedDetectionsPerChannel = globalSettings->displayedDetectionsPerVoxelInTableView;

	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(ShowContextMenu(const QPoint&)));

	createAction();
	}

void TableView::createAction()
{
	groupDetectionsPerChannel = new QActionGroup( this );
	detectionsPerChannel1Action = new QAction("1", this);
	detectionsPerChannel1Action->setCheckable(true);
	detectionsPerChannel1Action->setActionGroup(groupDetectionsPerChannel);

	detectionsPerChannel2Action = new QAction("2", this);
	detectionsPerChannel2Action->setCheckable(true);
	detectionsPerChannel2Action->setActionGroup(groupDetectionsPerChannel);

	detectionsPerChannel4Action = new QAction("4", this);
	detectionsPerChannel4Action->setCheckable(true);
	detectionsPerChannel4Action->setActionGroup(groupDetectionsPerChannel);

	detectionsPerChannel8Action = new QAction("8", this);
	detectionsPerChannel8Action->setCheckable(true);
	detectionsPerChannel8Action->setActionGroup(groupDetectionsPerChannel);

	switch (displayedDetectionsPerChannel)
	{
	case 1:
		detectionsPerChannel1Action->setChecked(true);
		break;
	case 2:
		detectionsPerChannel2Action->setChecked(true);
		break;
	case 4:
		detectionsPerChannel4Action->setChecked(true);
		break;
	case 8:
		detectionsPerChannel8Action->setChecked(true);
		break;
	default:
		break;
	}

	connect(groupDetectionsPerChannel, SIGNAL(triggered(QAction*)), this, SLOT(slotDetectionsPerChannelAction()));
	PrepareTableViews();
	AdjustTableSize();
}

void TableView::slotDetectionsPerChannelAction()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	if (detectionsPerChannel1Action->isChecked())
	{
		displayedDetectionsPerChannel = 1;
	}
	else if (detectionsPerChannel2Action->isChecked())
	{
		displayedDetectionsPerChannel = 2;

	}
	else if (detectionsPerChannel4Action->isChecked())
	{
		displayedDetectionsPerChannel = 4;

	}
	else if (detectionsPerChannel8Action->isChecked())
	{
		displayedDetectionsPerChannel = 8;

	}
	else 
	{
		displayedDetectionsPerChannel = globalSettings->displayedDetectionsPerVoxelInTableView;
	}

	PrepareTableViews();
	AdjustTableSize();
}

void TableView::slotConfigChanged()
{
	PrepareTableViews();
	AdjustTableSize();
}

void TableView::slotDetectionDataChanged(const Detection::Vector &inData)

{
	DisplayReceiverValues(inData);
}

void TableView::closeEvent(QCloseEvent * /*event*/)
{
	emit closed();
}

void TableView::resizeEvent(QResizeEvent * /*event*/)
{

}

void TableView::ShowContextMenu(const QPoint& pos) // this is a slot
{
    // for most widgets
    QPoint globalPos = mapToGlobal(pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 


	QMenu mainMenu;
	QMenu* menuDisplayedDetectionsPerChannel = mainMenu.addMenu("Detections per voxel");
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel1Action);
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel2Action);
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel4Action);
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel8Action);

	mainMenu.exec(globalPos);
}

/** \brief Structure to hold the description of a column in the table view.
*/typedef struct
{
		TableView::RealTimeColumn columnID;
		QString  columnFormat;
		int alignment;
}
TableColumSettings;

TableColumSettings columnSettings[] = {
	{TableView::eRealTimeReceiverIDColumn,		"0", Qt::AlignHCenter},
	{TableView::eRealTimeChannelIDColumn,		"00", Qt::AlignHCenter},
	{TableView::eRealTimeDetectionIDColumn,		"0", Qt::AlignHCenter},
	{TableView::eRealTimeTrackColumn,			"00000", Qt::AlignHCenter},
	{TableView::eRealTimeDistanceColumn,		"000.0", Qt::AlignRight},
	{ TableView::eRealTimeVelocityColumn,		"000.0", Qt::AlignRight }, 
	{ TableView::eRealTimeIntensityColumn,		"00.0", Qt::AlignRight },
	{TableView::eRealTimeCollisionLevelColumn,	"-00.00", Qt::AlignRight}
};

QSize TableView::sizeHint() const 
{
	return (maximumSizeHint());
}

const int scrollBarWidth = 30;
const int headerSpacing = 6;
QSize TableView::minimumSizeHint() const 

{
	// Calculate the dimensions of the table widget
	QSize newSize = unconstrainedTableSize();

	//Calculate height
	QTableWidget *tableWidget = ui.distanceTable;
	int rowCount = 4;
	newSize.setHeight(rowCount * (tableWidget->rowHeight(1)+1) + tableWidget->horizontalHeader()->height());

	// Calculate the size of the frame around it.
	int leftMargin, rightMargin, topMargin, bottomMargin;
	layout()->getContentsMargins(&leftMargin, &topMargin, &rightMargin, &bottomMargin);
	newSize.setWidth(newSize.width() + leftMargin + rightMargin); 
	newSize.setHeight(newSize.height() + topMargin + bottomMargin);
	return (newSize);
}

QSize TableView::maximumSizeHint() const 

{
	QSize newSize = unconstrainedTableSize();
	// Calculate the size of the frame around it.
	int leftMargin, rightMargin, topMargin, bottomMargin;
	layout()->getContentsMargins(&leftMargin, &topMargin, &rightMargin, &bottomMargin);
	newSize.setWidth(newSize.width() + leftMargin + rightMargin); 
	newSize.setHeight(newSize.height() + topMargin + bottomMargin);
	return(newSize);
}

QSize TableView::unconstrainedTableSize() const 

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	// Calculate the dimensions of the table widget
	QTableWidget *tableWidget = ui.distanceTable;

	//Calculate height
	int rowCount = 0;
	int tableHeight = 0;
	int receiverCount = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		rowCount += globalSettings->receiverSettings[receiverID].voxelsConfig.size()*displayedDetectionsPerChannel;
	} // for receiverID
	tableHeight = rowCount * (tableWidget->rowHeight(1));
	tableHeight += tableWidget->horizontalHeader()->height() + 10;
	
	
	// Calculate width
	int tableWidth =0;
	for (int column = 0; column < tableWidget->columnCount(); column++)
	{
#if 0
		tableWidth += columnSettings[column].columnWidth;
#else
		QTableWidgetItem * headerItem = tableWidget->horizontalHeaderItem(column);
		QFontMetrics fm(headerItem->font());
		int headerTextWidth = fm.width(headerItem->text());
		int dataTextWidth = fm.width(columnSettings[column].columnFormat);
		int headerWidth = std::max(headerTextWidth, dataTextWidth) + headerSpacing;
		tableWidth += headerWidth;
#endif
	}

	tableWidth += scrollBarWidth;
	tableWidth += tableWidget->frameWidth() * 2;

	// Calculate the size of the frame around it.
	return (QSize(tableWidth, tableHeight));
}


void TableView::PrepareTableViews()

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	QTableWidget *tableWidget = ui.distanceTable;
	tableWidget->clearContents();
	tableWidget->setRowCount(1);

	receiverFirstRow.clear();

	// Adjust the velocity title to display units
	bool bDisplayVelocityKmh = (globalSettings->velocityUnits == eVelocityUnitsKMH);
	if (bDisplayVelocityKmh)
	{
		tableWidget->horizontalHeaderItem(eRealTimeVelocityColumn)->setText("km/h");
	}
	else
	{
		tableWidget->horizontalHeaderItem(eRealTimeVelocityColumn)->setText("m/s");
	}

	// Adjust column width to the header text
	int columnQty =  tableWidget->columnCount();
	int tableWidth = 0;
	for (int column = 0; column < columnQty; column++) 
	{
#if 0
		tableWidget->setColumnWidth(column, columnSettings[column].columnWidth);
		tableWidth += columnSettings[column].columnWidth;
#else
		QTableWidgetItem * headerItem = tableWidget->horizontalHeaderItem(column);
		QFontMetrics fm(headerItem->font());
		int headerTextWidth = fm.width(headerItem->text());
		int dataTextWidth = fm.width(columnSettings[column].columnFormat);
		int headerWidth = std::max(headerTextWidth, dataTextWidth) + headerSpacing;

		tableWidget->setColumnWidth(column, headerWidth);
		tableWidth += headerWidth;
#endif
	}

	// Create the table widgets that will hold the data.  If required, add additional rows.
	int row = 0;
	int receiverCount = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		// Store the index of the first row for each voxel, for future references
		receiverFirstRow.push_back(row);

		int voxelCount = globalSettings->receiverSettings[receiverID].voxelsConfig.size();
		for (int voxelIndex = 0; voxelIndex < voxelCount; voxelIndex++) 
		{
			for (int detectionID = 0; detectionID < displayedDetectionsPerChannel; detectionID++) 
			{ 
				if (row >= tableWidget->rowCount())
				{
					tableWidget->insertRow(tableWidget->rowCount());
				}

				// Create the table items
				for (int column = 0; column < tableWidget->columnCount(); column++)
				{
					QTableWidgetItem *newItem = new QTableWidgetItem("");
					newItem->setTextAlignment(columnSettings[column].alignment);
					tableWidget->setItem(row, column, newItem);
				}  // for column

				row++;
			} // for detectionID
		} // for voxelIndex
	} // for receiverID
}

void TableView::AdjustTableSize()
{
	setMinimumSize(minimumSizeHint());
	setMaximumSize(maximumSizeHint());
}

void TableView::DisplayReceiverValues(const Detection::Vector &inData)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	QTableWidget *tableWidget = ui.distanceTable;
	// Fill the table with blanks
	int receiverCount = globalSettings->receiverSettings.size();
	int tableRow = 0;
	for (int receiverID = 0; receiverID < receiverCount; receiverID++) 
	{	
		int voxelCount = globalSettings->receiverSettings[receiverID].voxelsConfig.size();
		int columns = globalSettings->receiverSettings[receiverID].receiverColumns;

		for (int voxelIndex = 0; voxelIndex < voxelCount; voxelIndex++) 
		{
			CellID cellID(voxelIndex % columns, voxelIndex / columns);
			for (int detectionID = 0; detectionID < displayedDetectionsPerChannel; detectionID++)
			{
				AddDistanceToText(tableRow++, tableWidget, receiverID, cellID, detectionID);
			}  // for detection ID;
		} // for voxelIndex
	} // for receiverID

	// Place the receiver data
	BOOST_FOREACH(const Detection::Ptr & detection, inData)
	{
		if (detection->detectionID < displayedDetectionsPerChannel) 
		{
			CellID cellID = detection->cellID;
			int columns = globalSettings->receiverSettings[detection->receiverID].receiverColumns;
			tableRow = detection->detectionID + 
				       (cellID.column * displayedDetectionsPerChannel) + 
				       (cellID.row * columns * displayedDetectionsPerChannel) + 
					   receiverFirstRow.at(detection->receiverID);
			AddDistanceToText(tableRow, tableWidget, detection);
		}
	}

}


void TableView::AddDistanceToText(int rowIndex, QTableWidget *pTable, const Detection::Ptr &detection)

{
	if (rowIndex >= pTable->rowCount()) return;

	AddDistanceToText(rowIndex, pTable, detection->receiverID, detection->cellID, detection->detectionID,
		detection->trackID, detection->distance,  detection->threatLevel,
		detection->intensity, detection->velocity, detection->acceleration, detection->timeToCollision,
		detection->decelerationToStop, detection->probability);
}

void TableView::AddDistanceToText(int rowIndex, QTableWidget *pTable,  int receiverID, CellID inCellID, int detectionID, TrackID trackID, 
								float distance, 
								AlertCondition::ThreatLevel threatLevel,
								float intensity,
								float velocity,
								float acceleration, 
								float timeToCollision,
								float decelerationToStop,
								float probability
								)

{
	QString receiverStr;
	QString voxelStr;
	QString detectionStr;
	QString distanceStr;
	QString trackStr;
	QString velocityStr;
	QString intensityStr;
	QString threatStr;
	QColor  threatBackgroundColor;
	QColor  threatTextColor(Qt::white);
	QColor  threatEmptyColor = Qt::transparent;

	(void)acceleration;  // Not used
	(void)timeToCollision;  // Not used
	(void)decelerationToStop;  // Not used

	if (rowIndex >= pTable->rowCount()) return;

	if ((distance <= 0.0) || isNAN(distance) || trackID == 0)
	{
		receiverStr.sprintf("%d", receiverID+1);
		voxelStr.sprintf("%d,%d", inCellID.column, inCellID.row);
		detectionStr.sprintf("%d", detectionID+1);
		distanceStr.sprintf("");
		trackStr.sprintf("");
		velocityStr.sprintf("");
		intensityStr.sprintf("");
		threatStr.sprintf("");
		threatBackgroundColor = threatEmptyColor;
	}
	else
	{
		receiverStr.sprintf("%d", receiverID+1);
		voxelStr.sprintf("%d,%d", inCellID.column, inCellID.row);
		detectionStr.sprintf("%d", detectionID+1);

		distanceStr.sprintf("%.2f", distance);

		if (trackID > 0) 
		{
			trackStr.sprintf("%d", trackID);
		}
		else 
		{
			trackStr.sprintf("");
		}


		if (!isNAN(velocity)) 
		{
			if (AWLSettings::GetGlobalSettings()->velocityUnits == eVelocityUnitsMS)
			{
			velocityStr.sprintf("%.1f", velocity);  // Display velocity in m/s
			}
			else
			{
			velocityStr.sprintf("%.1f", VelocityToKmH(velocity));  // Display velocity in km/h
			}
		}
		else 
		{
			velocityStr.sprintf("");
		}

		if (!isNAN(intensity))
		{
			intensityStr.sprintf("%.1f", intensity);
		}
		else 
		{
			intensityStr.sprintf("");
		}

		if (!isNAN(probability))
		{
			threatStr.sprintf("%.1f", probability);
		}
		else
		{
			threatStr.sprintf("");
		}


		switch(threatLevel)
		{
		case AlertCondition::eThreatNone:
			{
				threatBackgroundColor = Qt::blue;
			}
			break;

		case AlertCondition::eThreatLow:
			{
				threatBackgroundColor = Qt::green;
			}
			break;

		case AlertCondition::eThreatWarn:
			{
				threatBackgroundColor = Qt::yellow;
				threatTextColor = Qt::black;
			}
			break;

		case AlertCondition::eThreatCritical:
			{
				threatBackgroundColor = Qt::red;
			}
			break;

		default:
			{
			}
		}
	}

	if (pTable->isVisible())
	{
		pTable->item(rowIndex, eRealTimeReceiverIDColumn)->setText(receiverStr);
		pTable->item(rowIndex, eRealTimeChannelIDColumn)->setText(voxelStr);
		pTable->item(rowIndex, eRealTimeDetectionIDColumn)->setText(detectionStr);

		pTable->item(rowIndex, eRealTimeDistanceColumn)->setText(distanceStr);
		pTable->item(rowIndex, eRealTimeIntensityColumn)->setText(intensityStr);
		pTable->item(rowIndex, eRealTimeVelocityColumn)->setText(velocityStr);
		pTable->item(rowIndex, eRealTimeTrackColumn)->setText(trackStr);
		pTable->item(rowIndex, eRealTimeCollisionLevelColumn)->setText(threatStr);
	
		pTable->item(rowIndex, eRealTimeCollisionLevelColumn)->setBackgroundColor(threatBackgroundColor);
		pTable->item(rowIndex, eRealTimeCollisionLevelColumn)->setTextColor(threatTextColor);
	}
}
