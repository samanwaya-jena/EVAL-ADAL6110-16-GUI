/********************************************************************************
** Form generated from reading UI file 'awlqtscope.ui'
**
** Created by: Qt User Interface Compiler version 5.0.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AWLQTSCOPE_H
#define UI_AWLQTSCOPE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>
#include "awlscopeplot.h"
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_AWLQtScopeClass
{
public:
    QFrame *scopeFrame;
    AWLScopePlot *channel5PlotFrame;
    AWLScopePlot *channel3PlotFrame;
    AWLScopePlot *channel4PlotFrame;
    AWLScopePlot *channel6PlotFrame;
    AWLScopePlot *channel7PlotFrame;
    AWLScopePlot *channel1PlotFrame;
    AWLScopePlot *channel2PlotFrame;
    QGroupBox *scopeCurveStyleGroupBox;
    QRadioButton *scopeCurveStyleDotsRadioButton;
    QRadioButton *scopeCurveStyleLinesRadioButton;

    void setupUi(QWidget *AWLQtScopeClass)
    {
        if (AWLQtScopeClass->objectName().isEmpty())
            AWLQtScopeClass->setObjectName(QStringLiteral("AWLQtScopeClass"));
        AWLQtScopeClass->resize(1243, 446);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AWLQtScopeClass->sizePolicy().hasHeightForWidth());
        AWLQtScopeClass->setSizePolicy(sizePolicy);
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(74, 74, 74, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(111, 111, 111, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(92, 92, 92, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(37, 37, 37, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(49, 49, 49, 255));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        QBrush brush6(QColor(0, 0, 0, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush6);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush4);
        QBrush brush7(QColor(255, 255, 220, 255));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush7);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush7);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush7);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush6);
        AWLQtScopeClass->setPalette(palette);
        scopeFrame = new QFrame(AWLQtScopeClass);
        scopeFrame->setObjectName(QStringLiteral("scopeFrame"));
        scopeFrame->setGeometry(QRect(10, 10, 1061, 411));
        scopeFrame->setFrameShape(QFrame::Panel);
        scopeFrame->setFrameShadow(QFrame::Raised);
        channel5PlotFrame = new AWLScopePlot(scopeFrame);
        channel5PlotFrame->setObjectName(QStringLiteral("channel5PlotFrame"));
        channel5PlotFrame->setGeometry(QRect(140, 10, 250, 175));
        channel5PlotFrame->setStyleSheet(QStringLiteral(""));
        channel3PlotFrame = new AWLScopePlot(scopeFrame);
        channel3PlotFrame->setObjectName(QStringLiteral("channel3PlotFrame"));
        channel3PlotFrame->setGeometry(QRect(530, 230, 250, 175));
        channel4PlotFrame = new AWLScopePlot(scopeFrame);
        channel4PlotFrame->setObjectName(QStringLiteral("channel4PlotFrame"));
        channel4PlotFrame->setGeometry(QRect(790, 230, 250, 175));
        channel6PlotFrame = new AWLScopePlot(scopeFrame);
        channel6PlotFrame->setObjectName(QStringLiteral("channel6PlotFrame"));
        channel6PlotFrame->setGeometry(QRect(400, 10, 250, 175));
        channel7PlotFrame = new AWLScopePlot(scopeFrame);
        channel7PlotFrame->setObjectName(QStringLiteral("channel7PlotFrame"));
        channel7PlotFrame->setGeometry(QRect(660, 10, 250, 175));
        channel1PlotFrame = new AWLScopePlot(scopeFrame);
        channel1PlotFrame->setObjectName(QStringLiteral("channel1PlotFrame"));
        channel1PlotFrame->setGeometry(QRect(10, 230, 250, 175));
        channel2PlotFrame = new AWLScopePlot(scopeFrame);
        channel2PlotFrame->setObjectName(QStringLiteral("channel2PlotFrame"));
        channel2PlotFrame->setGeometry(QRect(270, 230, 250, 175));
        scopeCurveStyleGroupBox = new QGroupBox(AWLQtScopeClass);
        scopeCurveStyleGroupBox->setObjectName(QStringLiteral("scopeCurveStyleGroupBox"));
        scopeCurveStyleGroupBox->setGeometry(QRect(1080, 270, 161, 71));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(scopeCurveStyleGroupBox->sizePolicy().hasHeightForWidth());
        scopeCurveStyleGroupBox->setSizePolicy(sizePolicy1);
        scopeCurveStyleGroupBox->setMinimumSize(QSize(161, 71));
        scopeCurveStyleGroupBox->setStyleSheet(QLatin1String("QToolTip\n"
"{\n"
"border: 1px solid black;\n"
"background-color: #ffa02f;\n"
"padding: 1px;\n"
"border-radius: 3px;\n"
"opacity: 100;\n"
"}\n"
"\n"
"QWidget\n"
"{\n"
"color: #dedede;\n"
"background-color: #404040;\n"
"}\n"
"\n"
"QWidget:item:hover\n"
"{\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #ca0619);\n"
"color: #000000;\n"
"}\n"
"\n"
"QWidget:item:selected\n"
"{\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QMenuBar::item\n"
"{\n"
"background: transparent;\n"
"}\n"
"\n"
"QMenuBar::item:selected\n"
"{\n"
"background: transparent;\n"
"border: 1px solid #ffaa00;\n"
"}\n"
"\n"
"QMenuBar::item:pressed\n"
"{\n"
"background: #444;\n"
"border: 1px solid #000;\n"
"background-color: QLinearGradient(\n"
"x1:0, y1:0,\n"
"x2:0, y2:1,\n"
"stop:1 #212121,\n"
"stop:0.4 #343434/*,\n"
"stop:0.2 #343434,\n"
"stop:0.1 #ffaa00*/\n"
");\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"}\n"
"\n"
"QMenu\n"
""
                        "{\n"
"border: 1px solid #000;\n"
"}\n"
"\n"
"QMenu::item\n"
"{\n"
"padding: 2px 20px 2px 20px;\n"
"}\n"
"\n"
"QMenu::item:selected\n"
"{\n"
"color: #000000;\n"
"}\n"
"\n"
"QWidget:disabled\n"
"{\n"
"color: #404040;\n"
"background-color: #808080;\n"
"}\n"
"\n"
"QAbstractItemView\n"
"{\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0.1 #646464, stop: 1 #5d5d5d);\n"
"}\n"
"\n"
"QWidget:focus\n"
"{\n"
"/*border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);*/\n"
"}\n"
"\n"
"QLineEdit\n"
"{\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0 #646464, stop: 1 #5d5d5d);\n"
"padding: 1px;\n"
"border-style: solid;\n"
"border: 1px solid #bebebe;\n"
"border-radius: 5;\n"
"}\n"
"\n"
"QPushButton\n"
"{\n"
"color: #dedede;\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #666666, stop: 0.1 #626262, stop: 0.5 #5e5e5e, stop: 0.9 #5a5a5a, stop: 1 #565656);\n"
"border-width: 1p"
                        "x;\n"
"border-color: #bebebe;\n"
"border-style: solid;\n"
"border-radius: 6;\n"
"padding: 3px;\n"
"font-size: 12px;\n"
"padding-left: 5px;\n"
"padding-right: 5px;\n"
"}\n"
"\n"
"QPushButton:pressed\n"
"{\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #2d2d2d, stop: 0.1 #2b2b2b, stop: 0.5 #292929, stop: 0.9 #282828, stop: 1 #252525);\n"
"}\n"
"\n"
"QComboBox\n"
"{\n"
"selection-background-color: #ffaa00;\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #565656, stop: 0.1 #525252, stop: 0.5 #4e4e4e, stop: 0.9 #4a4a4a, stop: 1 #464646);\n"
"border-style: solid;\n"
"border: 1px solid #1e1e1e;\n"
"border-radius: 5;\n"
"}\n"
"\n"
"QComboBox:hover,QPushButton:hover\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"\n"
"QComboBox:on\n"
"{\n"
"padding-top: 3px;\n"
"padding-left: 4px;\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #2d2d2d, stop: 0.1 #2b2b2b, stop: 0.5"
                        " #292929, stop: 0.9 #282828, stop: 1 #252525);\n"
"selection-background-color: #ffaa00;\n"
"}\n"
"\n"
"QComboBox QAbstractItemView\n"
"{\n"
"border: 2px solid darkgray;\n"
"selection-background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QComboBox::drop-down\n"
"{\n"
"subcontrol-origin: padding;\n"
"subcontrol-position: top right;\n"
"width: 15px;\n"
"\n"
"border-left-width: 0px;\n"
"border-left-color: darkgray;\n"
"border-left-style: solid; /* just a single line */\n"
"border-top-right-radius: 3px; /* same radius as the QComboBox */\n"
"border-bottom-right-radius: 3px;\n"
"}\n"
"\n"
"\n"
"QGroupBox:focus\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QTextEdit:focus\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QScrollBar:horizontal {\n"
"border: 1px solid #222222;\n"
"background: QLinearGradient( x1: "
                        "0, y1: 0, x2: 0, y2: 1, stop: 0.0 #121212, stop: 0.2 #282828, stop: 1 #484848);\n"
"height: 7px;\n"
"margin: 0px 16px 0 16px;\n"
"}\n"
"\n"
"QScrollBar::handle:horizontal\n"
"{\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 0.5 #d7801a, stop: 1 #ffa02f);\n"
"min-height: 20px;\n"
"border-radius: 2px;\n"
"}\n"
"\n"
"QScrollBar::add-line:horizontal {\n"
"border: 1px solid #1b1b19;\n"
"border-radius: 2px;\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"width: 14px;\n"
"subcontrol-position: right;\n"
"subcontrol-origin: margin;\n"
"}\n"
"\n"
"QScrollBar::sub-line:horizontal {\n"
"border: 1px solid #1b1b19;\n"
"border-radius: 2px;\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"width: 14px;\n"
"subcontrol-position: left;\n"
"subcontrol-origin: margin;\n"
"}\n"
"\n"
"QScrollBar::right-arrow:horizontal, QScrollBar::left-arrow:horizontal\n"
"{\n"
"border: 1px solid black;\n"
"widt"
                        "h: 1px;\n"
"height: 1px;\n"
"background: white;\n"
"}\n"
"\n"
"QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal\n"
"{\n"
"background: none;\n"
"}\n"
"\n"
"QScrollBar:vertical\n"
"{\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0.0 #121212, stop: 0.2 #282828, stop: 1 #484848);\n"
"width: 7px;\n"
"margin: 16px 0 16px 0;\n"
"border: 1px solid #222222;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical\n"
"{\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 0.5 #d7801a, stop: 1 #ffa02f);\n"
"min-height: 20px;\n"
"border-radius: 2px;\n"
"}\n"
"\n"
"QScrollBar::add-line:vertical\n"
"{\n"
"border: 1px solid #1b1b19;\n"
"border-radius: 2px;\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"height: 14px;\n"
"subcontrol-position: bottom;\n"
"subcontrol-origin: margin;\n"
"}\n"
"\n"
"QScrollBar::sub-line:vertical\n"
"{\n"
"border: 1px solid #1b1b19;\n"
"border-radius: 2px;\n"
"background: QLinearGradient( x1: "
                        "0, y1: 0, x2: 0, y2: 1, stop: 0 #d7801a, stop: 1 #ffa02f);\n"
"height: 14px;\n"
"subcontrol-position: top;\n"
"subcontrol-origin: margin;\n"
"}\n"
"\n"
"QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical\n"
"{\n"
"border: 1px solid black;\n"
"width: 1px;\n"
"height: 1px;\n"
"background: white;\n"
"}\n"
"\n"
"\n"
"QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical\n"
"{\n"
"background: none;\n"
"}\n"
"\n"
"QTextEdit\n"
"{\n"
"background-color: #242424;\n"
"}\n"
"\n"
"QPlainTextEdit\n"
"{\n"
"background-color: #242424;\n"
"}\n"
"\n"
"QHeaderView::section\n"
"{\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #616161, stop: 0.5 #505050, stop: 0.6 #434343, stop:1 #656565);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #6c6c6c;\n"
"}\n"
"\n"
"QCheckBox:disabled\n"
"{\n"
"color: #414141;\n"
"}\n"
"\n"
"QDockWidget::title\n"
"{\n"
"text-align: center;\n"
"spacing: 3px; /* spacing between items in the tool bar */\n"
"background-color: QLinearGradient(x1:0, y1:"
                        "0, x2:0, y2:1, stop:0 #808080, stop: 0.5 #242424, stop:1 #808080);\n"
"}\n"
"\n"
"QDockWidget::close-button, QDockWidget::float-button\n"
"{\n"
"text-align: center;\n"
"spacing: 1px; /* spacing between items in the tool bar */\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #808080, stop: 0.5 #242424, stop:1 #808080);\n"
"}\n"
"\n"
"QDockWidget::close-button:hover, QDockWidget::float-button:hover\n"
"{\n"
"background: #242424;\n"
"}\n"
"\n"
"QDockWidget::close-button:pressed, QDockWidget::float-button:pressed\n"
"{\n"
"padding: 1px -1px -1px 1px;\n"
"}\n"
"\n"
"QMainWindow::separator\n"
"{\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #161616, stop: 0.5 #151515, stop: 0.6 #212121, stop:1 #343434);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #4c4c4c;\n"
"spacing: 3px; /* spacing between items in the tool bar */\n"
"}\n"
"\n"
"QMainWindow::separator:hover\n"
"{\n"
"\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d7801a, stop:0."
                        "5 #b56c17 stop:1 #ffa02f);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #6c6c6c;\n"
"spacing: 3px; /* spacing between items in the tool bar */\n"
"}\n"
"\n"
"QToolBar::handle\n"
"{\n"
"spacing: 3px; /* spacing between items in the tool bar */\n"
"background: url(:/images/handle.png);\n"
"}\n"
"\n"
"QMenu::separator\n"
"{\n"
"height: 2px;\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #161616, stop: 0.5 #151515, stop: 0.6 #212121, stop:1 #343434);\n"
"color: white;\n"
"padding-left: 4px;\n"
"margin-left: 10px;\n"
"margin-right: 5px;\n"
"}\n"
"\n"
"QProgressBar\n"
"{\n"
"border: 2px solid grey;\n"
"border-radius: 5px;\n"
"text-align: center;\n"
"}\n"
"\n"
"QProgressBar::chunk\n"
"{\n"
"background-color: #d7801a;\n"
"width: 2.15px;\n"
"margin: 0.5px;\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"color: #b1b1b1;\n"
"border: 2px solid #dedede;\n"
"border-radius: 5px;\n"
"border-bottom-style: none;\n"
"background-color: #202020;\n"
"padding-left: 10px;\n"
"padding-right: 10px;\n"
"padding"
                        "-top: 3px;\n"
"padding-bottom: 2px;\n"
"margin-right: -1px;\n"
"}\n"
"\n"
"QTabWidget::pane {\n"
"border: 2px solid #dedede;\n"
"border-radius: 5px;\n"
"top: -2px;\n"
"}\n"
"\n"
"QTabBar::tab:last\n"
"{\n"
"margin-right: 0; /* the last selected tab has nothing to overlap with on the right */\n"
"border-top-right-radius: 5px;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-style:solid;\n"
"}\n"
"\n"
"QTabBar::tab:first:!selected\n"
"{\n"
"margin-left: 0px; /* the last selected tab has nothing to overlap with on the right */\n"
"\n"
"border-top-left-radius: 5px;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-style:solid;\n"
"}\n"
"\n"
"QTabBar::tab:!selected\n"
"{\n"
"color: #b1b1b1;\n"
"border-bottom-style: none;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-style:solid;\n"
"margin-top: 3px;\n"
"}\n"
"\n"
"QTabBar::tab:selected\n"
"{\n"
"color: #dedede;\n"
"border-top-left-radius: "
                        "5px;\n"
"border-top-right-radius: 5px;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-color: #404040;\n"
"background-color: #404040;\n"
"\n"
"border-bottom-style:solid;\n"
"margin-bottom: 0px;\n"
"}\n"
"\n"
"QTabBar::tab:!selected:hover\n"
"{\n"
"/*border-top: 2px solid #ffaa00;\n"
"padding-bottom: 3px;*/\n"
"border-top-left-radius: 5px;\n"
"border-top-right-radius: 5px;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-style:solid;\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:1 #212121, stop:0.4 #343434, stop:0.2 #343434, stop:0.1 #ffaa00);\n"
"}\n"
"\n"
"\n"
"QRadioButton::indicator:checked, QRadioButton::indicator:unchecked{\n"
"color: #b1b1b1;\n"
"background-color: #808080;\n"
"border: 1px solid #b1b1b1;\n"
"border-radius: 6px;\n"
"}\n"
"\n"
"QRadioButton::indicator:checked\n"
"{\n"
"background-color: qradialgradient(\n"
"cx: 0.5, cy: 0.5,\n"
"fx: 0.5, fy: 0.5,\n"
"radius: 1.0,\n"
"stop: 0.25 #ffaa00,"
                        "\n"
"stop: 0.3 #808080\n"
");\n"
"}\n"
"\n"
"QCheckBox::indicator{\n"
"color: #b1b1b1;\n"
"background-color: #808080;\n"
"border: 1px solid #b1b1b1;\n"
"width: 9px;\n"
"height: 9px;\n"
"}\n"
"\n"
"QRadioButton::indicator\n"
"{\n"
"border-radius: 6px;\n"
"}\n"
"\n"
"QRadioButton::indicator:hover, QCheckBox::indicator:hover\n"
"{\n"
"border: 1px solid #ffaa00;\n"
"}\n"
"\n"
"\n"
"QCheckBox::indicator:disabled, QRadioButton::indicator:disabled\n"
"{\n"
"border: 1px solid #444;\n"
"}\n"
""));
        scopeCurveStyleDotsRadioButton = new QRadioButton(scopeCurveStyleGroupBox);
        scopeCurveStyleDotsRadioButton->setObjectName(QStringLiteral("scopeCurveStyleDotsRadioButton"));
        scopeCurveStyleDotsRadioButton->setGeometry(QRect(20, 20, 91, 17));
        scopeCurveStyleLinesRadioButton = new QRadioButton(scopeCurveStyleGroupBox);
        scopeCurveStyleLinesRadioButton->setObjectName(QStringLiteral("scopeCurveStyleLinesRadioButton"));
        scopeCurveStyleLinesRadioButton->setGeometry(QRect(20, 40, 101, 17));

        retranslateUi(AWLQtScopeClass);
        QObject::connect(scopeCurveStyleDotsRadioButton, SIGNAL(toggled(bool)), AWLQtScopeClass, SLOT(on_scopeCurveStyleDots_setChecked(bool)));
        QObject::connect(scopeCurveStyleLinesRadioButton, SIGNAL(toggled(bool)), AWLQtScopeClass, SLOT(on_scopeCurveStyleLines_setChecked(bool)));

        QMetaObject::connectSlotsByName(AWLQtScopeClass);
    } // setupUi

    void retranslateUi(QWidget *AWLQtScopeClass)
    {
        AWLQtScopeClass->setWindowTitle(QApplication::translate("AWLQtScopeClass", "AWLQtScope", 0));
        scopeCurveStyleGroupBox->setTitle(QApplication::translate("AWLQtScopeClass", "Curve Style", 0));
        scopeCurveStyleDotsRadioButton->setText(QApplication::translate("AWLQtScopeClass", "Dots", 0));
        scopeCurveStyleLinesRadioButton->setText(QApplication::translate("AWLQtScopeClass", "Lines", 0));
    } // retranslateUi

};

namespace Ui {
    class AWLQtScopeClass: public Ui_AWLQtScopeClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AWLQTSCOPE_H
