/********************************************************************************
** Form generated from reading UI file 'awlplotscan.ui'
**
** Created by: Qt User Interface Compiler version 5.9.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AWLPLOTSCAN_H
#define UI_AWLPLOTSCAN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_AWLPlotScanFrame
{
public:

    void setupUi(QFrame *AWLPlotScanFrame)
    {
        if (AWLPlotScanFrame->objectName().isEmpty())
            AWLPlotScanFrame->setObjectName(QStringLiteral("AWLPlotScanFrame"));
        AWLPlotScanFrame->resize(250, 1000);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(2);
        sizePolicy.setHeightForWidth(AWLPlotScanFrame->sizePolicy().hasHeightForWidth());
        AWLPlotScanFrame->setSizePolicy(sizePolicy);
        AWLPlotScanFrame->setMinimumSize(QSize(250, 600));
        AWLPlotScanFrame->setBaseSize(QSize(500, 600));
        QIcon icon;
        icon.addFile(QStringLiteral("AWLQtDemo.ico"), QSize(), QIcon::Normal, QIcon::Off);
        AWLPlotScanFrame->setWindowIcon(icon);
        AWLPlotScanFrame->setStyleSheet(QStringLiteral(""));
        AWLPlotScanFrame->setFrameShape(QFrame::StyledPanel);
        AWLPlotScanFrame->setFrameShadow(QFrame::Raised);

        retranslateUi(AWLPlotScanFrame);

        QMetaObject::connectSlotsByName(AWLPlotScanFrame);
    } // setupUi

    void retranslateUi(QFrame *AWLPlotScanFrame)
    {
        AWLPlotScanFrame->setWindowTitle(QApplication::translate("AWLPlotScanFrame", "Frame", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class AWLPlotScanFrame: public Ui_AWLPlotScanFrame {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AWLPLOTSCAN_H
