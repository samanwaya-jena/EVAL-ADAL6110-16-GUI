#ifndef AWLQTDEMO_H
#define AWLQTDEMO_H

#include <QtWidgets/QMainWindow>
#include <QTimer>

#include "ui_awlqtdemoscope.h"

using namespace awl;
using namespace pcl;

class AWLQtDemocope : public QDialog
{
	Q_OBJECT

public:
	AWLQtDemoScope();
	~AWLQtDemoScope();

	private slots:


protected:

private:
	Ui::AWLQtDemoScopeClass ui;
};

#endif // AWLQTDEMO_H
