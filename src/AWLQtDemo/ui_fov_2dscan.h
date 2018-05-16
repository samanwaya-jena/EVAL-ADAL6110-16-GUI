/********************************************************************************
** Form generated from reading UI file 'FOV_2DScan.ui'
**
** Created by: Qt User Interface Compiler version 5.9.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FOV_2DSCAN_H
#define UI_FOV_2DSCAN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_FOV_2DScanFrame
{
public:

    void setupUi(QFrame *FOV_2DScanFrame)
    {
        if (FOV_2DScanFrame->objectName().isEmpty())
            FOV_2DScanFrame->setObjectName(QStringLiteral("FOV_2DScanFrame"));
        FOV_2DScanFrame->resize(250, 300);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(2);
        sizePolicy.setHeightForWidth(FOV_2DScanFrame->sizePolicy().hasHeightForWidth());
        FOV_2DScanFrame->setSizePolicy(sizePolicy);
        FOV_2DScanFrame->setMinimumSize(QSize(250, 300));
        FOV_2DScanFrame->setBaseSize(QSize(500, 600));
        QIcon icon;
        icon.addFile(QStringLiteral("AWLQtDemo.ico"), QSize(), QIcon::Normal, QIcon::Off);
        FOV_2DScanFrame->setWindowIcon(icon);
        FOV_2DScanFrame->setStyleSheet(QStringLiteral(""));
        FOV_2DScanFrame->setFrameShape(QFrame::StyledPanel);
        FOV_2DScanFrame->setFrameShadow(QFrame::Raised);

        retranslateUi(FOV_2DScanFrame);

        QMetaObject::connectSlotsByName(FOV_2DScanFrame);
    } // setupUi

    void retranslateUi(QFrame *FOV_2DScanFrame)
    {
        FOV_2DScanFrame->setWindowTitle(QApplication::translate("FOV_2DScanFrame", "Frame", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class FOV_2DScanFrame: public Ui_FOV_2DScanFrame {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FOV_2DSCAN_H
