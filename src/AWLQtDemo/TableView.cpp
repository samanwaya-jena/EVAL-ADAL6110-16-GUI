#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>
#include <QTableWidget>
#include <QListWidget>

#include <boost/foreach.hpp>

#include "AWLSettings.h"
#include "TableView.h"
#include "Tracker.h"

using namespace std;
using namespace awl;

TableView::TableView(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);
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
}

void TableView::slotConfigChanged()
{
	PrepareTableViews();
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
	ui.distanceTable->resize(ui.distanceTable->width(), height() -10);
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


void TableView::PrepareTableViews()

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	QTableWidget *tableWidget;
	tableWidget = ui.distanceTable;
	tableWidget->clearContents();
	tableWidget->setRowCount(1);

	receiverFirstRow.clear();


	// Adjust column width
	int columnQty =  tableWidget->columnCount();
	for (int column = 0; column < columnQty; column++) 
	{
		if (column <= eRealTimeDetectionIDColumn) 
		{
			tableWidget->setColumnWidth(column, tableWidget->horizontalHeader()->minimumSectionSize());
		}
		else
		{
			tableWidget->setColumnWidth(column, tableWidget->horizontalHeader()->defaultSectionSize());
		}
	}

	// Adjust the velocity title to display units
	bool bDisplayVelocityKmh = (globalSettings->velocityUnits == eVelocityUnitsKMH);
	if (bDisplayVelocityKmh) 
	{
		tableWidget->horizontalHeaderItem(eRealTimeVelocityColumn)->setText("Vel km/h");
	}
	else
	{
		tableWidget->horizontalHeaderItem(eRealTimeVelocityColumn)->setText("Vel m/s");
	}


	// Create the table widgets that will hold the data.  If required, add additional rows.
	int row = 0;
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
					tableWidget->rowHeight(row);
				}

				// Create the table items
				for (int column = 0; column < tableWidget->columnCount(); column++)
				{
					QTableWidgetItem *newItem = new QTableWidgetItem("");
					tableWidget->setItem(row, column, newItem);
				}  // for column

				row++;
			} // for detectionID
		} // for channelID
	} // for receiverID

	// Resize the window to reflect the size of the updated table
	int tableHeight = 2 + tableWidget->horizontalHeader()->height(); 
	for(int row = 0; row < tableWidget->rowCount(); row++)
	{ 
	    tableHeight += tableWidget->rowHeight(row); 
	} 

	QRect scr = QApplication::desktop()->availableGeometry(/*QApplication::desktop()->primaryScreen()*/);
	QRect frame = frameGeometry();
	QRect client = geometry();
	int verticalDecorationsHeight = frame.height() - client.height();
	int horizontalDecorationsWidth = frame.width() - client.width();

	float recommendedHeight =  scr.height() - verticalDecorationsHeight;
	float recommendedWidth = scr.width() - horizontalDecorationsWidth;

	int width = tableWidget->width() + 10;
	int height = tableHeight + 10;
	if (height > recommendedHeight) height = recommendedHeight;

	setMinimumSize(width,height);
	resize(width, height);
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
								Detection::ThreatLevel threatLevel, 
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
	QColor  threatEmptyColor(0x60, 0x60, 0x60);

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
			intensityStr.sprintf("%.0f", intensity * 100);
		}
		else 
		{
			intensityStr.sprintf("");
		}

		if (!isNAN(decelerationToStop))
		{
			threatStr.sprintf("%.1f", decelerationToStop);
		}
		else
		{
			threatStr.sprintf("");
		}


		switch(threatLevel)
		{
		case Detection::eThreatNone:
			{
				threatBackgroundColor = Qt::blue;
			}
			break;

		case Detection::eThreatLow:
			{
				threatBackgroundColor = Qt::green;
			}
			break;

		case Detection::eThreatWarn:
			{
				threatBackgroundColor = Qt::yellow;
				threatTextColor = Qt::black;
			}
			break;

		case Detection::eThreatCritical:
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