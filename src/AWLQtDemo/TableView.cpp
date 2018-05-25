/* TableView.cpp */
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

using namespace std;
using namespace awl;

TableView::TableView(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);
	setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	ui.distanceTable->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded); 
	setWindowIcon(QApplication::windowIcon());

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	displayedDetectionsPerChannel = globalSettings->displayedDetectionsPerChannelInTableView;

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
		displayedDetectionsPerChannel = globalSettings->displayedDetectionsPerChannelInTableView;
	}

	PrepareTableViews();
	AdjustTableSize();
}

void TableView::slotConfigChanged()
{
	PrepareTableViews();
	AdjustTableSize();
}

void TableView::slotDetectionDataChanged(const Detection::Vector &data)

{
	DisplayReceiverValues(data);
}

void TableView::closeEvent(QCloseEvent * event)
{
	emit closed();
}

void TableView::resizeEvent(QResizeEvent * event)
{

}

void TableView::ShowContextMenu(const QPoint& pos) // this is a slot
{
    // for most widgets
    QPoint globalPos = mapToGlobal(pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 


	QMenu mainMenu;
	QMenu* menuDisplayedDetectionsPerChannel = mainMenu.addMenu("Detections per channel");
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel1Action);
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel2Action);
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel4Action);
 	menuDisplayedDetectionsPerChannel->addAction(detectionsPerChannel8Action);

	mainMenu.exec(globalPos);
}

typedef struct 
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
	int tableHeight = 0;
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
		rowCount += globalSettings->receiverSettings[receiverID].channelsConfig.size()*displayedDetectionsPerChannel;
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
		int headerWidth = max(headerTextWidth, dataTextWidth) + headerSpacing;
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
		int headerWidth = max(headerTextWidth, dataTextWidth) + headerSpacing;

		tableWidget->setColumnWidth(column, headerWidth);
		tableWidth += headerWidth;
#endif
	}

	// Create the table widgets that will hold the data.  If required, add additional rows.
	int row = 0;
	int tableHeight = 0;
	int receiverCount = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++)
	{
		// Store the index of the first row for each channel, for future references
		receiverFirstRow.push_back(row);

		int channelCount = globalSettings->receiverSettings[receiverID].channelsConfig.size();
		for (int channelID = 0; channelID < channelCount; channelID++) 
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
		} // for channelID
	} // for receiverID
}

void TableView::AdjustTableSize()
{
	setMinimumSize(minimumSizeHint());
	setMaximumSize(maximumSizeHint());
}

void TableView::DisplayReceiverValues(const Detection::Vector &data)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	QTableWidget *tableWidget = ui.distanceTable;
	int rowCount = tableWidget->rowCount();
	// Fill the table with blanks
	int receiverCount = globalSettings->receiverSettings.size();
	int tableRow = 0;
	for (int receiverID = 0; receiverID < receiverCount; receiverID++) 
	{	
		int channelCount = globalSettings->receiverSettings[receiverID].channelsConfig.size();
		for (int channelID = 0; channelID < channelCount; channelID++) 
		{
			for (int detectionID = 0; detectionID < displayedDetectionsPerChannel; detectionID++)
			{
				AddDistanceToText(tableRow++, tableWidget, receiverID, channelID, detectionID);
			}  // for detection ID;
		} // for channelID
	} // for receiverID

	// Place the receiver data
	BOOST_FOREACH(const Detection::Ptr & detection, data)
	{
		tableRow = detection->detectionID + (detection->channelID * displayedDetectionsPerChannel) + receiverFirstRow.at(detection->receiverID); 
		if (detection->detectionID < displayedDetectionsPerChannel) AddDistanceToText(tableRow, tableWidget, detection);
	}

}


void TableView::AddDistanceToText(int rowIndex, QTableWidget *pTable, const Detection::Ptr &detection)

{
	if (rowIndex >= pTable->rowCount()) return;


	AddDistanceToText(rowIndex, pTable, detection->receiverID, detection->channelID, detection->detectionID,
		detection->trackID, detection->distance,  detection->threatLevel,
		detection->intensity, detection->velocity, detection->acceleration, detection->timeToCollision,
		detection->decelerationToStop, detection->probability);
}

void TableView::AddDistanceToText(int rowIndex, QTableWidget *pTable,  int receiverID, int channelID, int detectionID, TrackID trackID, 
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
	QString channelStr;
	QString detectionStr;
	QString distanceStr;
	QString trackStr;
	QString velocityStr;
	QString intensityStr;
	QString threatStr;
	QColor  threatBackgroundColor;
	QColor  threatTextColor(Qt::white);
	QColor  threatEmptyColor = Qt::transparent;

	if (rowIndex >= pTable->rowCount()) return;

	if ((distance <= 0.0) || isNAN(distance) || trackID == 0)
	{
		receiverStr.sprintf("%d", receiverID+1);
		channelStr.sprintf("%d", channelID+1);
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
		channelStr.sprintf("%d", channelID+1);
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
		pTable->item(rowIndex, eRealTimeChannelIDColumn)->setText(channelStr);
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
