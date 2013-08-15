/********************************************************************************
** Form generated from reading UI file 'FOV_2DScan.ui'
**
** Created by: Qt User Interface Compiler version 5.0.1
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
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_Frame
{
public:
    QHBoxLayout *horizontalLayout;
    QGraphicsView *FOV_2DScan;

    void setupUi(QFrame *Frame)
    {
        if (Frame->objectName().isEmpty())
            Frame->setObjectName(QStringLiteral("Frame"));
        Frame->resize(480, 386);
        Frame->setFrameShape(QFrame::StyledPanel);
        Frame->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(Frame);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        FOV_2DScan = new QGraphicsView(Frame);
        FOV_2DScan->setObjectName(QStringLiteral("FOV_2DScan"));

        horizontalLayout->addWidget(FOV_2DScan);


        retranslateUi(Frame);

        QMetaObject::connectSlotsByName(Frame);
    } // setupUi

    void retranslateUi(QFrame *Frame)
    {
        Frame->setWindowTitle(QApplication::translate("Frame", "Frame", 0));
    } // retranslateUi

};

namespace Ui {
    class Frame: public Ui_Frame {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FOV_2DSCAN_H
