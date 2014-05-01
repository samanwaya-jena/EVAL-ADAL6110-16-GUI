#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>
#include <QTableWidget>
#include <QListWidget>


#include "AWLSettings.h"
#include "TableView.h"
#include "DetectionStruct.h"

using namespace std;
using namespace awl;

TableView::TableView(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	displayedDetectionsPerChannel = globalSettings->displayedDetectionsPerChannelInTableView;

	// Position the widget on the top left side
	setMinimumSize(350,350);

	// Change the window icon if there is an override in the INI file
	if (!globalSettings->sIconFileName.empty())
	{
		setWindowIcon(QIcon(globalSettings->sIconFileName.c_str()));
	}

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

void TableView::slotDetectionDataChanged(DetectionDataVect* data)

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
					tableWidget->setItem(row, column, newItem);
				}  // for column

				row++;
			} // for detectionID
		} // for channelID
	} // for receiverID
}


void TableView::DisplayReceiverValues(DetectionDataVect* data)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	QTableWidget *tableWidget = ui.distanceTable;
	int rowCount = tableWidget->rowCount();

	DetectionDataVect::const_iterator detection = data->begin();
 
	int tableRow = 0;
	int receiverCount = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverCount; receiverID++) 
	{
		int channelCount = globalSettings->receiverSettings[receiverID].channelsConfig.size();
		for (int channelID = 0; channelID < channelCount ; channelID++) 
		{
			int detectionID = 0;
			for (detectionID = 0; detectionID < displayedDetectionsPerChannel; detectionID++)
			{
				if (detection->receiverID == receiverID && detection->channelID == channelID && detection->detectionID == detectionID)
				{
					AddDistanceToText(tableRow++, tableWidget, detection);	
					if (detection != data->end()) detection++;
				}
				else
				{
					AddDistanceToText(tableRow++, tableWidget, receiverID, channelID, detectionID);
				}
			}  // for detection ID;

			// Skip all extra detections for the channel
			while (detection != data->end() && detection->receiverID == receiverID && detection->channelID == channelID)
			{
				detection++;
			} // while 

		} // for channelID
	} // for receiverID
}


void TableView::AddDistanceToText(int rowIndex, QTableWidget *pTable, const Detection *detection)

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
		receiverStr.sprintf("%2d", receiverID+1);
		channelStr.sprintf("%2d", channelID+1);
		detectionStr.sprintf("%0d", detectionID+1);

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
