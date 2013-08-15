#include "awlqtdemo.h"
#include <QtWidgets/QApplication>

using namespace awl;

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QStyle *pStyle;
#if 0
	a.setStyle("fusion");

#endif
	a.setPalette( Qt::darkGray );

	AWLQtDemo w(argc, argv);
	w.show();
	return a.exec();
}
