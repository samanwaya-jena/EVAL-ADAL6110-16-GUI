/********************************************************************************
** Form generated from reading UI file 'awlqtdemo.ui'
**
** Created by: Qt User Interface Compiler version 5.0.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AWLQTDEMO_H
#define UI_AWLQTDEMO_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AWLQtDemoClass
{
public:
    QAction *actionQuitter;
    QAction *action3D_View;
    QAction *actionGraph;
    QAction *action2D;
    QAction *actionCamera;
    QWidget *centralWidget;
    QTabWidget *interfaceTabs;
    QWidget *realTimeTab;
    QTableWidget *distanceTable5;
    QTableWidget *distanceTable6;
    QTableWidget *distanceTable7;
    QTableWidget *distanceTable1;
    QTableWidget *distanceTable2;
    QTableWidget *distanceTable3;
    QTableWidget *distanceTable4;
    QWidget *controlTab;
    QGroupBox *recordPlayGroupBox;
    QPushButton *recordButton;
    QPushButton *playbackButton;
    QGroupBox *channelMaskGroupBox;
    QCheckBox *channel7CheckBox;
    QCheckBox *channel6CheckBox;
    QCheckBox *channel5CheckBox;
    QCheckBox *channel2CheckBox;
    QCheckBox *channel3CheckBox;
    QCheckBox *channel1CheckBox;
    QCheckBox *channel4CheckBox;
    QLabel *frameRateLabel;
    QSpinBox *frameRateSpinBox;
    QPushButton *stopButton;
    QGroupBox *recordFileNameGroupBox;
    QLineEdit *recordFileNameEdit;
    QLabel *label;
    QLabel *label_2;
    QLineEdit *playbackFileNameEdit;
    QGroupBox *algoGroupBox;
    QRadioButton *algo1Checkbox;
    QRadioButton *algo2RadioButton;
    QRadioButton *algo3RadioButton;
    QRadioButton *algo4RadioButton;
    QGroupBox *injectSimulatedGroupBox;
    QCheckBox *injectSimulatedCheckbox;
    QWidget *statusTab;
    QLabel *VersionLabel;
    QLineEdit *versionEdit;
    QLineEdit *temperatureEdit;
    QLabel *TemperatureLabel;
    QLabel *TemperatureLabel_2;
    QLineEdit *voltageEdit;
    QLabel *VoltageLabel;
    QLabel *VoltageLabel_2;
    QGroupBox *bootGroupBox;
    QCheckBox *bootReceiverCheckBox;
    QCheckBox *bootEmitter1CheckBox;
    QCheckBox *bootAuxChecksumCheckBox;
    QCheckBox *bootEmitter2CheckBox;
    QCheckBox *bootMainChecksumCheckBox;
    QCheckBox *bootMemoryCheckBox;
    QCheckBox *bootDSPCheckBox;
    QCheckBox *bootChecksumCheckBox;
    QGroupBox *statusGroupBox;
    QCheckBox *statusSelfTestCheckBox;
    QCheckBox *statusShutdownCheckBox;
    QCheckBox *statusSensorBlockedCheckBox;
    QCheckBox *statusReducedPerformanceCheckBox;
    QCheckBox *statusSaturationCheckBox;
    QCheckBox *statusSaturationCheckBox_2;
    QCheckBox *statusSensorBlockedCheckBox_2;
    QCheckBox *statusShutdownCheckBox_2;
    QCheckBox *statusReducedPerformanceCheckBox_2;
    QCheckBox *statusSelfTestCheckBox_2;
    QGroupBox *hardwareGroupBox;
    QCheckBox *hardwareReceiverCheckBox;
    QCheckBox *hardwareEmitter1CheckBox;
    QCheckBox *hardwareEmitter2CheckBox;
    QCheckBox *hardwareMemoryCheckBox;
    QCheckBox *hardwareDSPCheckBox;
    QGroupBox *receiverGroupBox;
    QCheckBox *receiverChannel3CheckBox;
    QCheckBox *receiverChannel1CheckBox;
    QCheckBox *receiverChannel2CheckBox;
    QCheckBox *receiverChannel5CheckBox;
    QCheckBox *receiverChannel4CheckBox;
    QCheckBox *receiverChannel7CheckBox;
    QCheckBox *receiverChannel6CheckBox;
    QWidget *tab;
    QGroupBox *registerFPGAGroupBox;
    QPushButton *registerFPGASetPushButton;
    QPushButton *registerFPGAGetPushButton;
    QGroupBox *registerFPGASetGroupBox;
    QLabel *registerFPGAAddressSetLabel;
    QLabel *registerFPGAValueSetLabel;
    QComboBox *registerFPGAAddressSetComboBox;
    QLineEdit *registerFPGAValueSetLineEdit;
    QGroupBox *registerFPGAGetGroupBox;
    QLineEdit *registerFPGAAddressGetLineEdit;
    QLineEdit *registerFPGAValueGetLineEdit;
    QGroupBox *registerADCGroupBox;
    QPushButton *registerADCSetPushButton;
    QPushButton *registerADCGetPushButton;
    QGroupBox *registerADCSetGroupBox;
    QLabel *registerADCAddressSetLabel;
    QLabel *registerADCValueSetLabel;
    QComboBox *registerADCAddressSetComboBox;
    QLineEdit *registerADCValueSetLineEdit;
    QGroupBox *registerADCGetGroupBox;
    QLineEdit *registerADCAddressGetLineEdit;
    QLineEdit *registerADCValueGetLineEdit;
    QWidget *calibrationTab;
    QGroupBox *internalCalibrationGroupBox;
    QGroupBox *calibrationChannelMaskGroupBox;
    QCheckBox *calibrationChannel7CheckBox;
    QCheckBox *calibrationChannel6CheckBox;
    QCheckBox *calibrationChannel5CheckBox;
    QCheckBox *calibrationChannel2CheckBox;
    QCheckBox *calibrationChannel3CheckBox;
    QCheckBox *calibrationChannel1CheckBox;
    QCheckBox *calibrationChannel4CheckBox;
    QLabel *calibrationBetaLabel;
    QDoubleSpinBox *calibrationBetaDoubleSpinBox;
    QSpinBox *calibrationFrameQtySpinBox;
    QLabel *calibrationFrameQtyLabel;
    QPushButton *calibrateButton;
    QGroupBox *externalcalibrationGroupBox;
    QGroupBox *sensorPositionGroupBox;
    QDoubleSpinBox *sensorHeightSpinBox;
    QLabel *sensorHeightLabel;
    QDoubleSpinBox *sensorDepthSpinBox;
    QLabel *sensorDepthLabel;
    QGroupBox *receiverCalibrationGroupBox;
    QLabel *measurementOffsetLabel;
    QDoubleSpinBox *measurementOffsetSpinBox;
    QGroupBox *sensorRangeGroupBox;
    QLabel *sensorRangeMinLabel;
    QLabel *sensorRangeMaxLabel;
    QDoubleSpinBox *sensorRangeMinSpinBox;
    QDoubleSpinBox *sensorRangeMaxSpinBox;
    QWidget *threeDOptionsTab;
    QFrame *viewerColorFrame;
    QGroupBox *viewColorGroupBox;
    QRadioButton *colorImageRadioButton;
    QRadioButton *rangeImageRadioButton;
    QGroupBox *cameraViewGroupBox;
    QPushButton *viewSidePushButton;
    QPushButton *viewTopPushButton;
    QPushButton *viewIsoPushButton;
    QPushButton *viewFrontPushButton;
    QPushButton *viewFrontPushButton_2;
    QGroupBox *displayOptionsGroupBox;
    QSpinBox *decimationSpinBox;
    QLabel *decimationLabel;
    QLabel *pixelSizeLabel;
    QSpinBox *pixelSizeSpinBox;
    QMenuBar *menuBar;
    QMenu *menuFichier;
    QMenu *menuView;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *AWLQtDemoClass)
    {
        if (AWLQtDemoClass->objectName().isEmpty())
            AWLQtDemoClass->setObjectName(QStringLiteral("AWLQtDemoClass"));
        AWLQtDemoClass->resize(1042, 317);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AWLQtDemoClass->sizePolicy().hasHeightForWidth());
        AWLQtDemoClass->setSizePolicy(sizePolicy);
        QIcon icon;
        icon.addFile(QStringLiteral("Aerostar-logoForIcon128.bmp"), QSize(), QIcon::Normal, QIcon::Off);
        AWLQtDemoClass->setWindowIcon(icon);
        AWLQtDemoClass->setLayoutDirection(Qt::LeftToRight);
        AWLQtDemoClass->setAutoFillBackground(false);
        AWLQtDemoClass->setStyleSheet(QLatin1String("QToolTip\n"
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
"QRadioButton::indicator\n"
"{\n"
"border-radius: 6px;\n"
"}\n"
"\n"
" QRadioButton::indicator:disabled\n"
"{\n"
"border: 1px solid #444;\n"
"}\n"
"\n"
"QRadioButton::indicator:hover, QCheckBox::indicator:hover\n"
"{\n"
"border: 1px solid #ffaa00;\n"
"}\n"
"\n"
"\n"
"QCheckBox::indicator\n"
"{\n"
"color: #b1b1b1;\n"
"background-color: #808080;\n"
"border: 1px solid #b1b1b1;\n"
"width: 9px;\n"
"height: 9px;\n"
"}\n"
"\n"
"QCheckBox::indicator:disabled\n"
"{\n"
"border: 1px solid #444;\n"
"}\n"
"\n"
"QCheckBox:disabled\n"
"{\n"
"color: #414141;\n"
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
"QCheckBox::indicator:checked, QCheckBox::indicator:unchecked{\n"
"color: #b1b1b1;\n"
"background-color: #808080;\n"
"border: 1px solid #b1b1b1;\n"
"border-radius: 6px;\n"
"}\n"
"\n"
"QCheckBox::indicator:checked\n"
"{\n"
"background-color: qradialgradient(\n"
"cx: "
                        "0.5, cy: 0.5,\n"
"fx: 0.5, fy: 0.5,\n"
"radius: 1.0,\n"
"stop: 0.25 #ffaa00,\n"
"stop: 0.3 #808080\n"
");\n"
"}\n"
""));
        AWLQtDemoClass->setTabShape(QTabWidget::Rounded);
        actionQuitter = new QAction(AWLQtDemoClass);
        actionQuitter->setObjectName(QStringLiteral("actionQuitter"));
        action3D_View = new QAction(AWLQtDemoClass);
        action3D_View->setObjectName(QStringLiteral("action3D_View"));
        action3D_View->setCheckable(true);
        actionGraph = new QAction(AWLQtDemoClass);
        actionGraph->setObjectName(QStringLiteral("actionGraph"));
        actionGraph->setCheckable(true);
        action2D = new QAction(AWLQtDemoClass);
        action2D->setObjectName(QStringLiteral("action2D"));
        action2D->setCheckable(true);
        actionCamera = new QAction(AWLQtDemoClass);
        actionCamera->setObjectName(QStringLiteral("actionCamera"));
        actionCamera->setCheckable(true);
        centralWidget = new QWidget(AWLQtDemoClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        interfaceTabs = new QTabWidget(centralWidget);
        interfaceTabs->setObjectName(QStringLiteral("interfaceTabs"));
        interfaceTabs->setGeometry(QRect(10, 0, 1021, 261));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(interfaceTabs->sizePolicy().hasHeightForWidth());
        interfaceTabs->setSizePolicy(sizePolicy1);
        interfaceTabs->setAutoFillBackground(false);
        interfaceTabs->setTabPosition(QTabWidget::North);
        interfaceTabs->setTabShape(QTabWidget::Rounded);
        interfaceTabs->setUsesScrollButtons(false);
        interfaceTabs->setDocumentMode(false);
        interfaceTabs->setMovable(true);
        realTimeTab = new QWidget();
        realTimeTab->setObjectName(QStringLiteral("realTimeTab"));
        distanceTable5 = new QTableWidget(realTimeTab);
        if (distanceTable5->columnCount() < 4)
            distanceTable5->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        distanceTable5->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        distanceTable5->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        distanceTable5->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        distanceTable5->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        if (distanceTable5->rowCount() < 8)
            distanceTable5->setRowCount(8);
        distanceTable5->setObjectName(QStringLiteral("distanceTable5"));
        distanceTable5->setGeometry(QRect(126, 10, 240, 105));
        QFont font;
        font.setFamily(QStringLiteral("Segoe UI"));
        font.setPointSize(8);
        distanceTable5->setFont(font);
        distanceTable5->setAutoFillBackground(true);
        distanceTable5->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable5->setFrameShape(QFrame::Panel);
        distanceTable5->setFrameShadow(QFrame::Sunken);
        distanceTable5->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable5->setGridStyle(Qt::SolidLine);
        distanceTable5->setWordWrap(false);
        distanceTable5->setRowCount(8);
        distanceTable5->setColumnCount(4);
        distanceTable5->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable5->horizontalHeader()->setHighlightSections(true);
        distanceTable5->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable5->verticalHeader()->setDefaultSectionSize(21);
        distanceTable6 = new QTableWidget(realTimeTab);
        if (distanceTable6->columnCount() < 4)
            distanceTable6->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        distanceTable6->setHorizontalHeaderItem(0, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        distanceTable6->setHorizontalHeaderItem(1, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        distanceTable6->setHorizontalHeaderItem(2, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        distanceTable6->setHorizontalHeaderItem(3, __qtablewidgetitem7);
        if (distanceTable6->rowCount() < 8)
            distanceTable6->setRowCount(8);
        distanceTable6->setObjectName(QStringLiteral("distanceTable6"));
        distanceTable6->setGeometry(QRect(376, 10, 240, 105));
        distanceTable6->setFont(font);
        distanceTable6->setAutoFillBackground(true);
        distanceTable6->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable6->setFrameShape(QFrame::Panel);
        distanceTable6->setFrameShadow(QFrame::Sunken);
        distanceTable6->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable6->setGridStyle(Qt::SolidLine);
        distanceTable6->setWordWrap(false);
        distanceTable6->setRowCount(8);
        distanceTable6->setColumnCount(4);
        distanceTable6->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable6->horizontalHeader()->setHighlightSections(true);
        distanceTable6->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable6->verticalHeader()->setDefaultSectionSize(21);
        distanceTable7 = new QTableWidget(realTimeTab);
        if (distanceTable7->columnCount() < 4)
            distanceTable7->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        distanceTable7->setHorizontalHeaderItem(0, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        distanceTable7->setHorizontalHeaderItem(1, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        distanceTable7->setHorizontalHeaderItem(2, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        distanceTable7->setHorizontalHeaderItem(3, __qtablewidgetitem11);
        if (distanceTable7->rowCount() < 8)
            distanceTable7->setRowCount(8);
        distanceTable7->setObjectName(QStringLiteral("distanceTable7"));
        distanceTable7->setGeometry(QRect(627, 10, 240, 105));
        distanceTable7->setFont(font);
        distanceTable7->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable7->setFrameShape(QFrame::Panel);
        distanceTable7->setFrameShadow(QFrame::Sunken);
        distanceTable7->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable7->setGridStyle(Qt::SolidLine);
        distanceTable7->setWordWrap(false);
        distanceTable7->setRowCount(8);
        distanceTable7->setColumnCount(4);
        distanceTable7->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable7->horizontalHeader()->setHighlightSections(true);
        distanceTable7->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable7->verticalHeader()->setDefaultSectionSize(21);
        distanceTable1 = new QTableWidget(realTimeTab);
        if (distanceTable1->columnCount() < 4)
            distanceTable1->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        distanceTable1->setHorizontalHeaderItem(0, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        distanceTable1->setHorizontalHeaderItem(1, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        distanceTable1->setHorizontalHeaderItem(2, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        distanceTable1->setHorizontalHeaderItem(3, __qtablewidgetitem15);
        if (distanceTable1->rowCount() < 8)
            distanceTable1->setRowCount(8);
        distanceTable1->setObjectName(QStringLiteral("distanceTable1"));
        distanceTable1->setGeometry(QRect(5, 120, 240, 105));
        distanceTable1->setFont(font);
        distanceTable1->setAutoFillBackground(true);
        distanceTable1->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable1->setFrameShape(QFrame::Panel);
        distanceTable1->setFrameShadow(QFrame::Sunken);
        distanceTable1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable1->setGridStyle(Qt::SolidLine);
        distanceTable1->setWordWrap(false);
        distanceTable1->setRowCount(8);
        distanceTable1->setColumnCount(4);
        distanceTable1->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable1->horizontalHeader()->setHighlightSections(true);
        distanceTable1->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable1->verticalHeader()->setDefaultSectionSize(21);
        distanceTable2 = new QTableWidget(realTimeTab);
        if (distanceTable2->columnCount() < 4)
            distanceTable2->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        distanceTable2->setHorizontalHeaderItem(0, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        distanceTable2->setHorizontalHeaderItem(1, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        distanceTable2->setHorizontalHeaderItem(2, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        distanceTable2->setHorizontalHeaderItem(3, __qtablewidgetitem19);
        if (distanceTable2->rowCount() < 8)
            distanceTable2->setRowCount(8);
        distanceTable2->setObjectName(QStringLiteral("distanceTable2"));
        distanceTable2->setGeometry(QRect(256, 120, 240, 105));
        distanceTable2->setFont(font);
        distanceTable2->setAutoFillBackground(true);
        distanceTable2->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable2->setFrameShape(QFrame::Panel);
        distanceTable2->setFrameShadow(QFrame::Sunken);
        distanceTable2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable2->setGridStyle(Qt::SolidLine);
        distanceTable2->setWordWrap(false);
        distanceTable2->setRowCount(8);
        distanceTable2->setColumnCount(4);
        distanceTable2->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable2->horizontalHeader()->setHighlightSections(true);
        distanceTable2->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable2->verticalHeader()->setDefaultSectionSize(21);
        distanceTable3 = new QTableWidget(realTimeTab);
        if (distanceTable3->columnCount() < 4)
            distanceTable3->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem20 = new QTableWidgetItem();
        distanceTable3->setHorizontalHeaderItem(0, __qtablewidgetitem20);
        QTableWidgetItem *__qtablewidgetitem21 = new QTableWidgetItem();
        distanceTable3->setHorizontalHeaderItem(1, __qtablewidgetitem21);
        QTableWidgetItem *__qtablewidgetitem22 = new QTableWidgetItem();
        distanceTable3->setHorizontalHeaderItem(2, __qtablewidgetitem22);
        QTableWidgetItem *__qtablewidgetitem23 = new QTableWidgetItem();
        distanceTable3->setHorizontalHeaderItem(3, __qtablewidgetitem23);
        if (distanceTable3->rowCount() < 8)
            distanceTable3->setRowCount(8);
        distanceTable3->setObjectName(QStringLiteral("distanceTable3"));
        distanceTable3->setGeometry(QRect(507, 120, 240, 105));
        distanceTable3->setFont(font);
        distanceTable3->setAutoFillBackground(true);
        distanceTable3->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable3->setFrameShape(QFrame::Panel);
        distanceTable3->setFrameShadow(QFrame::Sunken);
        distanceTable3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable3->setGridStyle(Qt::SolidLine);
        distanceTable3->setWordWrap(false);
        distanceTable3->setRowCount(8);
        distanceTable3->setColumnCount(4);
        distanceTable3->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable3->horizontalHeader()->setHighlightSections(true);
        distanceTable3->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable3->verticalHeader()->setDefaultSectionSize(21);
        distanceTable4 = new QTableWidget(realTimeTab);
        if (distanceTable4->columnCount() < 4)
            distanceTable4->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem24 = new QTableWidgetItem();
        distanceTable4->setHorizontalHeaderItem(0, __qtablewidgetitem24);
        QTableWidgetItem *__qtablewidgetitem25 = new QTableWidgetItem();
        distanceTable4->setHorizontalHeaderItem(1, __qtablewidgetitem25);
        QTableWidgetItem *__qtablewidgetitem26 = new QTableWidgetItem();
        distanceTable4->setHorizontalHeaderItem(2, __qtablewidgetitem26);
        QTableWidgetItem *__qtablewidgetitem27 = new QTableWidgetItem();
        distanceTable4->setHorizontalHeaderItem(3, __qtablewidgetitem27);
        if (distanceTable4->rowCount() < 8)
            distanceTable4->setRowCount(8);
        distanceTable4->setObjectName(QStringLiteral("distanceTable4"));
        distanceTable4->setGeometry(QRect(760, 120, 240, 105));
        distanceTable4->setFont(font);
        distanceTable4->setAutoFillBackground(true);
        distanceTable4->setStyleSheet(QLatin1String("background-color: rgb(96,96,96);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        distanceTable4->setFrameShape(QFrame::Panel);
        distanceTable4->setFrameShadow(QFrame::Sunken);
        distanceTable4->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable4->setGridStyle(Qt::SolidLine);
        distanceTable4->setWordWrap(false);
        distanceTable4->setRowCount(8);
        distanceTable4->setColumnCount(4);
        distanceTable4->horizontalHeader()->setDefaultSectionSize(51);
        distanceTable4->horizontalHeader()->setHighlightSections(true);
        distanceTable4->horizontalHeader()->setMinimumSectionSize(10);
        distanceTable4->verticalHeader()->setDefaultSectionSize(21);
        interfaceTabs->addTab(realTimeTab, QString());
        controlTab = new QWidget();
        controlTab->setObjectName(QStringLiteral("controlTab"));
        recordPlayGroupBox = new QGroupBox(controlTab);
        recordPlayGroupBox->setObjectName(QStringLiteral("recordPlayGroupBox"));
        recordPlayGroupBox->setGeometry(QRect(10, 10, 481, 221));
        recordButton = new QPushButton(recordPlayGroupBox);
        recordButton->setObjectName(QStringLiteral("recordButton"));
        recordButton->setGeometry(QRect(30, 190, 75, 20));
        recordButton->setCheckable(true);
        recordButton->setChecked(false);
        recordButton->setAutoDefault(false);
        recordButton->setDefault(false);
        recordButton->setFlat(false);
        playbackButton = new QPushButton(recordPlayGroupBox);
        playbackButton->setObjectName(QStringLiteral("playbackButton"));
        playbackButton->setGeometry(QRect(110, 190, 75, 20));
        playbackButton->setCheckable(true);
        playbackButton->setChecked(false);
        playbackButton->setAutoDefault(false);
        playbackButton->setDefault(false);
        playbackButton->setFlat(false);
        channelMaskGroupBox = new QGroupBox(recordPlayGroupBox);
        channelMaskGroupBox->setObjectName(QStringLiteral("channelMaskGroupBox"));
        channelMaskGroupBox->setGeometry(QRect(10, 50, 461, 61));
        channelMaskGroupBox->setFlat(true);
        channelMaskGroupBox->setCheckable(false);
        channel7CheckBox = new QCheckBox(channelMaskGroupBox);
        channel7CheckBox->setObjectName(QStringLiteral("channel7CheckBox"));
        channel7CheckBox->setGeometry(QRect(190, 20, 31, 17));
        channel6CheckBox = new QCheckBox(channelMaskGroupBox);
        channel6CheckBox->setObjectName(QStringLiteral("channel6CheckBox"));
        channel6CheckBox->setGeometry(QRect(150, 20, 31, 17));
        channel5CheckBox = new QCheckBox(channelMaskGroupBox);
        channel5CheckBox->setObjectName(QStringLiteral("channel5CheckBox"));
        channel5CheckBox->setGeometry(QRect(110, 20, 31, 17));
        channel2CheckBox = new QCheckBox(channelMaskGroupBox);
        channel2CheckBox->setObjectName(QStringLiteral("channel2CheckBox"));
        channel2CheckBox->setGeometry(QRect(130, 40, 31, 17));
        channel3CheckBox = new QCheckBox(channelMaskGroupBox);
        channel3CheckBox->setObjectName(QStringLiteral("channel3CheckBox"));
        channel3CheckBox->setGeometry(QRect(170, 40, 31, 17));
        channel1CheckBox = new QCheckBox(channelMaskGroupBox);
        channel1CheckBox->setObjectName(QStringLiteral("channel1CheckBox"));
        channel1CheckBox->setGeometry(QRect(90, 40, 31, 17));
        channel1CheckBox->setChecked(false);
        channel4CheckBox = new QCheckBox(channelMaskGroupBox);
        channel4CheckBox->setObjectName(QStringLiteral("channel4CheckBox"));
        channel4CheckBox->setGeometry(QRect(200, 40, 31, 17));
        frameRateLabel = new QLabel(recordPlayGroupBox);
        frameRateLabel->setObjectName(QStringLiteral("frameRateLabel"));
        frameRateLabel->setGeometry(QRect(30, 20, 61, 16));
        frameRateLabel->setFrameShape(QFrame::NoFrame);
        frameRateSpinBox = new QSpinBox(recordPlayGroupBox);
        frameRateSpinBox->setObjectName(QStringLiteral("frameRateSpinBox"));
        frameRateSpinBox->setGeometry(QRect(100, 20, 71, 22));
        frameRateSpinBox->setMinimum(1);
        frameRateSpinBox->setMaximum(100);
        frameRateSpinBox->setValue(100);
        stopButton = new QPushButton(recordPlayGroupBox);
        stopButton->setObjectName(QStringLiteral("stopButton"));
        stopButton->setEnabled(false);
        stopButton->setGeometry(QRect(260, 190, 75, 20));
        stopButton->setCheckable(true);
        stopButton->setChecked(false);
        stopButton->setAutoDefault(false);
        stopButton->setDefault(false);
        stopButton->setFlat(false);
        recordFileNameGroupBox = new QGroupBox(recordPlayGroupBox);
        recordFileNameGroupBox->setObjectName(QStringLiteral("recordFileNameGroupBox"));
        recordFileNameGroupBox->setGeometry(QRect(10, 110, 461, 71));
        recordFileNameGroupBox->setFlat(true);
        recordFileNameGroupBox->setCheckable(false);
        recordFileNameEdit = new QLineEdit(recordFileNameGroupBox);
        recordFileNameEdit->setObjectName(QStringLiteral("recordFileNameEdit"));
        recordFileNameEdit->setGeometry(QRect(90, 20, 361, 20));
        recordFileNameEdit->setInputMask(QStringLiteral(""));
        recordFileNameEdit->setMaxLength(31);
        recordFileNameEdit->setPlaceholderText(QStringLiteral(""));
        label = new QLabel(recordFileNameGroupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 22, 46, 13));
        label_2 = new QLabel(recordFileNameGroupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(20, 50, 46, 13));
        playbackFileNameEdit = new QLineEdit(recordFileNameGroupBox);
        playbackFileNameEdit->setObjectName(QStringLiteral("playbackFileNameEdit"));
        playbackFileNameEdit->setGeometry(QRect(90, 49, 361, 20));
        playbackFileNameEdit->setInputMask(QStringLiteral(""));
        playbackFileNameEdit->setMaxLength(31);
        playbackFileNameEdit->setPlaceholderText(QStringLiteral(""));
        algoGroupBox = new QGroupBox(controlTab);
        algoGroupBox->setObjectName(QStringLiteral("algoGroupBox"));
        algoGroupBox->setGeometry(QRect(510, 10, 121, 111));
        algo1Checkbox = new QRadioButton(algoGroupBox);
        algo1Checkbox->setObjectName(QStringLiteral("algo1Checkbox"));
        algo1Checkbox->setGeometry(QRect(20, 20, 70, 17));
        algo1Checkbox->setChecked(false);
        algo2RadioButton = new QRadioButton(algoGroupBox);
        algo2RadioButton->setObjectName(QStringLiteral("algo2RadioButton"));
        algo2RadioButton->setGeometry(QRect(20, 40, 70, 17));
        algo3RadioButton = new QRadioButton(algoGroupBox);
        algo3RadioButton->setObjectName(QStringLiteral("algo3RadioButton"));
        algo3RadioButton->setGeometry(QRect(20, 60, 70, 17));
        algo4RadioButton = new QRadioButton(algoGroupBox);
        algo4RadioButton->setObjectName(QStringLiteral("algo4RadioButton"));
        algo4RadioButton->setGeometry(QRect(20, 80, 70, 17));
        injectSimulatedGroupBox = new QGroupBox(controlTab);
        injectSimulatedGroupBox->setObjectName(QStringLiteral("injectSimulatedGroupBox"));
        injectSimulatedGroupBox->setGeometry(QRect(510, 190, 121, 41));
        injectSimulatedCheckbox = new QCheckBox(injectSimulatedGroupBox);
        injectSimulatedCheckbox->setObjectName(QStringLiteral("injectSimulatedCheckbox"));
        injectSimulatedCheckbox->setGeometry(QRect(20, 20, 70, 17));
        injectSimulatedCheckbox->setCheckable(true);
        injectSimulatedCheckbox->setChecked(false);
        injectSimulatedCheckbox->setAutoRepeat(false);
        injectSimulatedCheckbox->setTristate(false);
        interfaceTabs->addTab(controlTab, QString());
        statusTab = new QWidget();
        statusTab->setObjectName(QStringLiteral("statusTab"));
        VersionLabel = new QLabel(statusTab);
        VersionLabel->setObjectName(QStringLiteral("VersionLabel"));
        VersionLabel->setGeometry(QRect(20, 10, 71, 20));
        versionEdit = new QLineEdit(statusTab);
        versionEdit->setObjectName(QStringLiteral("versionEdit"));
        versionEdit->setGeometry(QRect(70, 10, 113, 20));
        versionEdit->setReadOnly(true);
        temperatureEdit = new QLineEdit(statusTab);
        temperatureEdit->setObjectName(QStringLiteral("temperatureEdit"));
        temperatureEdit->setGeometry(QRect(256, 10, 51, 20));
        temperatureEdit->setReadOnly(true);
        TemperatureLabel = new QLabel(statusTab);
        TemperatureLabel->setObjectName(QStringLiteral("TemperatureLabel"));
        TemperatureLabel->setGeometry(QRect(220, 10, 31, 20));
        TemperatureLabel_2 = new QLabel(statusTab);
        TemperatureLabel_2->setObjectName(QStringLiteral("TemperatureLabel_2"));
        TemperatureLabel_2->setGeometry(QRect(310, 10, 21, 20));
        TemperatureLabel_2->setTextFormat(Qt::RichText);
        voltageEdit = new QLineEdit(statusTab);
        voltageEdit->setObjectName(QStringLiteral("voltageEdit"));
        voltageEdit->setGeometry(QRect(400, 10, 51, 20));
        voltageEdit->setReadOnly(true);
        VoltageLabel = new QLabel(statusTab);
        VoltageLabel->setObjectName(QStringLiteral("VoltageLabel"));
        VoltageLabel->setGeometry(QRect(350, 10, 41, 20));
        VoltageLabel_2 = new QLabel(statusTab);
        VoltageLabel_2->setObjectName(QStringLiteral("VoltageLabel_2"));
        VoltageLabel_2->setGeometry(QRect(453, 10, 30, 20));
        VoltageLabel_2->setTextFormat(Qt::AutoText);
        bootGroupBox = new QGroupBox(statusTab);
        bootGroupBox->setObjectName(QStringLiteral("bootGroupBox"));
        bootGroupBox->setGeometry(QRect(10, 40, 111, 181));
        bootReceiverCheckBox = new QCheckBox(bootGroupBox);
        bootReceiverCheckBox->setObjectName(QStringLiteral("bootReceiverCheckBox"));
        bootReceiverCheckBox->setGeometry(QRect(10, 100, 91, 17));
        bootReceiverCheckBox->setCheckable(false);
        bootEmitter1CheckBox = new QCheckBox(bootGroupBox);
        bootEmitter1CheckBox->setObjectName(QStringLiteral("bootEmitter1CheckBox"));
        bootEmitter1CheckBox->setGeometry(QRect(10, 60, 91, 17));
        bootEmitter1CheckBox->setCheckable(false);
        bootAuxChecksumCheckBox = new QCheckBox(bootGroupBox);
        bootAuxChecksumCheckBox->setObjectName(QStringLiteral("bootAuxChecksumCheckBox"));
        bootAuxChecksumCheckBox->setGeometry(QRect(10, 40, 91, 17));
        bootAuxChecksumCheckBox->setCheckable(false);
        bootEmitter2CheckBox = new QCheckBox(bootGroupBox);
        bootEmitter2CheckBox->setObjectName(QStringLiteral("bootEmitter2CheckBox"));
        bootEmitter2CheckBox->setGeometry(QRect(10, 80, 91, 17));
        bootEmitter2CheckBox->setCheckable(false);
        bootMainChecksumCheckBox = new QCheckBox(bootGroupBox);
        bootMainChecksumCheckBox->setObjectName(QStringLiteral("bootMainChecksumCheckBox"));
        bootMainChecksumCheckBox->setGeometry(QRect(10, 20, 91, 17));
        bootMainChecksumCheckBox->setCheckable(false);
        bootMemoryCheckBox = new QCheckBox(bootGroupBox);
        bootMemoryCheckBox->setObjectName(QStringLiteral("bootMemoryCheckBox"));
        bootMemoryCheckBox->setGeometry(QRect(10, 140, 91, 17));
        bootMemoryCheckBox->setCheckable(false);
        bootDSPCheckBox = new QCheckBox(bootGroupBox);
        bootDSPCheckBox->setObjectName(QStringLiteral("bootDSPCheckBox"));
        bootDSPCheckBox->setGeometry(QRect(10, 120, 91, 17));
        bootDSPCheckBox->setCheckable(false);
        bootChecksumCheckBox = new QCheckBox(bootGroupBox);
        bootChecksumCheckBox->setObjectName(QStringLiteral("bootChecksumCheckBox"));
        bootChecksumCheckBox->setGeometry(QRect(10, 160, 91, 17));
        bootChecksumCheckBox->setCheckable(false);
        statusGroupBox = new QGroupBox(statusTab);
        statusGroupBox->setObjectName(QStringLiteral("statusGroupBox"));
        statusGroupBox->setGeometry(QRect(400, 40, 151, 181));
        statusSelfTestCheckBox = new QCheckBox(statusGroupBox);
        statusSelfTestCheckBox->setObjectName(QStringLiteral("statusSelfTestCheckBox"));
        statusSelfTestCheckBox->setGeometry(QRect(10, 20, 131, 17));
        statusSelfTestCheckBox->setCheckable(false);
        statusShutdownCheckBox = new QCheckBox(statusGroupBox);
        statusShutdownCheckBox->setObjectName(QStringLiteral("statusShutdownCheckBox"));
        statusShutdownCheckBox->setGeometry(QRect(10, 40, 131, 17));
        statusShutdownCheckBox->setCheckable(false);
        statusSensorBlockedCheckBox = new QCheckBox(statusGroupBox);
        statusSensorBlockedCheckBox->setObjectName(QStringLiteral("statusSensorBlockedCheckBox"));
        statusSensorBlockedCheckBox->setGeometry(QRect(10, 60, 131, 17));
        statusSensorBlockedCheckBox->setCheckable(false);
        statusReducedPerformanceCheckBox = new QCheckBox(statusGroupBox);
        statusReducedPerformanceCheckBox->setObjectName(QStringLiteral("statusReducedPerformanceCheckBox"));
        statusReducedPerformanceCheckBox->setGeometry(QRect(10, 80, 131, 17));
        statusReducedPerformanceCheckBox->setCheckable(false);
        statusSaturationCheckBox = new QCheckBox(statusGroupBox);
        statusSaturationCheckBox->setObjectName(QStringLiteral("statusSaturationCheckBox"));
        statusSaturationCheckBox->setGeometry(QRect(10, 100, 131, 17));
        statusSaturationCheckBox->setCheckable(false);
        statusSaturationCheckBox_2 = new QCheckBox(statusGroupBox);
        statusSaturationCheckBox_2->setObjectName(QStringLiteral("statusSaturationCheckBox_2"));
        statusSaturationCheckBox_2->setGeometry(QRect(-300, 100, 131, 17));
        statusSaturationCheckBox_2->setCheckable(false);
        statusSensorBlockedCheckBox_2 = new QCheckBox(statusGroupBox);
        statusSensorBlockedCheckBox_2->setObjectName(QStringLiteral("statusSensorBlockedCheckBox_2"));
        statusSensorBlockedCheckBox_2->setGeometry(QRect(-300, 60, 91, 17));
        statusSensorBlockedCheckBox_2->setCheckable(false);
        statusShutdownCheckBox_2 = new QCheckBox(statusGroupBox);
        statusShutdownCheckBox_2->setObjectName(QStringLiteral("statusShutdownCheckBox_2"));
        statusShutdownCheckBox_2->setGeometry(QRect(-300, 40, 70, 17));
        statusShutdownCheckBox_2->setCheckable(false);
        statusReducedPerformanceCheckBox_2 = new QCheckBox(statusGroupBox);
        statusReducedPerformanceCheckBox_2->setObjectName(QStringLiteral("statusReducedPerformanceCheckBox_2"));
        statusReducedPerformanceCheckBox_2->setGeometry(QRect(-300, 80, 131, 17));
        statusReducedPerformanceCheckBox_2->setCheckable(false);
        statusSelfTestCheckBox_2 = new QCheckBox(statusGroupBox);
        statusSelfTestCheckBox_2->setObjectName(QStringLiteral("statusSelfTestCheckBox_2"));
        statusSelfTestCheckBox_2->setGeometry(QRect(-300, 20, 70, 17));
        statusSelfTestCheckBox_2->setCheckable(false);
        hardwareGroupBox = new QGroupBox(statusTab);
        hardwareGroupBox->setObjectName(QStringLiteral("hardwareGroupBox"));
        hardwareGroupBox->setGeometry(QRect(140, 40, 111, 181));
        hardwareReceiverCheckBox = new QCheckBox(hardwareGroupBox);
        hardwareReceiverCheckBox->setObjectName(QStringLiteral("hardwareReceiverCheckBox"));
        hardwareReceiverCheckBox->setGeometry(QRect(10, 60, 91, 17));
        hardwareReceiverCheckBox->setCheckable(false);
        hardwareEmitter1CheckBox = new QCheckBox(hardwareGroupBox);
        hardwareEmitter1CheckBox->setObjectName(QStringLiteral("hardwareEmitter1CheckBox"));
        hardwareEmitter1CheckBox->setGeometry(QRect(10, 20, 91, 17));
        hardwareEmitter1CheckBox->setCheckable(false);
        hardwareEmitter2CheckBox = new QCheckBox(hardwareGroupBox);
        hardwareEmitter2CheckBox->setObjectName(QStringLiteral("hardwareEmitter2CheckBox"));
        hardwareEmitter2CheckBox->setGeometry(QRect(10, 40, 91, 17));
        hardwareEmitter2CheckBox->setCheckable(false);
        hardwareMemoryCheckBox = new QCheckBox(hardwareGroupBox);
        hardwareMemoryCheckBox->setObjectName(QStringLiteral("hardwareMemoryCheckBox"));
        hardwareMemoryCheckBox->setGeometry(QRect(10, 100, 91, 17));
        hardwareMemoryCheckBox->setCheckable(false);
        hardwareDSPCheckBox = new QCheckBox(hardwareGroupBox);
        hardwareDSPCheckBox->setObjectName(QStringLiteral("hardwareDSPCheckBox"));
        hardwareDSPCheckBox->setGeometry(QRect(10, 80, 91, 17));
        hardwareDSPCheckBox->setCheckable(false);
        receiverGroupBox = new QGroupBox(statusTab);
        receiverGroupBox->setObjectName(QStringLiteral("receiverGroupBox"));
        receiverGroupBox->setGeometry(QRect(270, 40, 111, 181));
        receiverChannel3CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel3CheckBox->setObjectName(QStringLiteral("receiverChannel3CheckBox"));
        receiverChannel3CheckBox->setGeometry(QRect(10, 60, 91, 17));
        receiverChannel3CheckBox->setCheckable(false);
        receiverChannel1CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel1CheckBox->setObjectName(QStringLiteral("receiverChannel1CheckBox"));
        receiverChannel1CheckBox->setGeometry(QRect(10, 20, 91, 17));
        receiverChannel1CheckBox->setCheckable(false);
        receiverChannel2CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel2CheckBox->setObjectName(QStringLiteral("receiverChannel2CheckBox"));
        receiverChannel2CheckBox->setGeometry(QRect(10, 40, 91, 17));
        receiverChannel2CheckBox->setCheckable(false);
        receiverChannel5CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel5CheckBox->setObjectName(QStringLiteral("receiverChannel5CheckBox"));
        receiverChannel5CheckBox->setGeometry(QRect(10, 100, 91, 17));
        receiverChannel5CheckBox->setCheckable(false);
        receiverChannel4CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel4CheckBox->setObjectName(QStringLiteral("receiverChannel4CheckBox"));
        receiverChannel4CheckBox->setGeometry(QRect(10, 80, 91, 17));
        receiverChannel4CheckBox->setCheckable(false);
        receiverChannel7CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel7CheckBox->setObjectName(QStringLiteral("receiverChannel7CheckBox"));
        receiverChannel7CheckBox->setGeometry(QRect(10, 140, 91, 17));
        receiverChannel7CheckBox->setCheckable(false);
        receiverChannel6CheckBox = new QCheckBox(receiverGroupBox);
        receiverChannel6CheckBox->setObjectName(QStringLiteral("receiverChannel6CheckBox"));
        receiverChannel6CheckBox->setGeometry(QRect(10, 120, 91, 17));
        receiverChannel6CheckBox->setCheckable(false);
        interfaceTabs->addTab(statusTab, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        registerFPGAGroupBox = new QGroupBox(tab);
        registerFPGAGroupBox->setObjectName(QStringLiteral("registerFPGAGroupBox"));
        registerFPGAGroupBox->setGeometry(QRect(10, 10, 431, 141));
        registerFPGASetPushButton = new QPushButton(registerFPGAGroupBox);
        registerFPGASetPushButton->setObjectName(QStringLiteral("registerFPGASetPushButton"));
        registerFPGASetPushButton->setGeometry(QRect(50, 110, 75, 23));
        registerFPGAGetPushButton = new QPushButton(registerFPGAGroupBox);
        registerFPGAGetPushButton->setObjectName(QStringLiteral("registerFPGAGetPushButton"));
        registerFPGAGetPushButton->setGeometry(QRect(220, 110, 75, 23));
        registerFPGASetGroupBox = new QGroupBox(registerFPGAGroupBox);
        registerFPGASetGroupBox->setObjectName(QStringLiteral("registerFPGASetGroupBox"));
        registerFPGASetGroupBox->setGeometry(QRect(10, 20, 321, 81));
        registerFPGASetGroupBox->setMinimumSize(QSize(4, 0));
        registerFPGAAddressSetLabel = new QLabel(registerFPGASetGroupBox);
        registerFPGAAddressSetLabel->setObjectName(QStringLiteral("registerFPGAAddressSetLabel"));
        registerFPGAAddressSetLabel->setGeometry(QRect(10, 22, 71, 16));
        registerFPGAValueSetLabel = new QLabel(registerFPGASetGroupBox);
        registerFPGAValueSetLabel->setObjectName(QStringLiteral("registerFPGAValueSetLabel"));
        registerFPGAValueSetLabel->setGeometry(QRect(10, 52, 41, 16));
        registerFPGAAddressSetComboBox = new QComboBox(registerFPGASetGroupBox);
        registerFPGAAddressSetComboBox->setObjectName(QStringLiteral("registerFPGAAddressSetComboBox"));
        registerFPGAAddressSetComboBox->setGeometry(QRect(90, 20, 211, 22));
        registerFPGAAddressSetComboBox->setEditable(true);
        registerFPGAValueSetLineEdit = new QLineEdit(registerFPGASetGroupBox);
        registerFPGAValueSetLineEdit->setObjectName(QStringLiteral("registerFPGAValueSetLineEdit"));
        registerFPGAValueSetLineEdit->setGeometry(QRect(90, 50, 61, 20));
        registerFPGAValueSetLineEdit->setFrame(true);
        registerFPGAGetGroupBox = new QGroupBox(registerFPGAGroupBox);
        registerFPGAGetGroupBox->setObjectName(QStringLiteral("registerFPGAGetGroupBox"));
        registerFPGAGetGroupBox->setGeometry(QRect(340, 20, 81, 81));
        registerFPGAGetGroupBox->setMinimumSize(QSize(4, 0));
        registerFPGAAddressGetLineEdit = new QLineEdit(registerFPGAGetGroupBox);
        registerFPGAAddressGetLineEdit->setObjectName(QStringLiteral("registerFPGAAddressGetLineEdit"));
        registerFPGAAddressGetLineEdit->setGeometry(QRect(10, 20, 61, 20));
        registerFPGAAddressGetLineEdit->setFrame(false);
        registerFPGAAddressGetLineEdit->setReadOnly(true);
        registerFPGAValueGetLineEdit = new QLineEdit(registerFPGAGetGroupBox);
        registerFPGAValueGetLineEdit->setObjectName(QStringLiteral("registerFPGAValueGetLineEdit"));
        registerFPGAValueGetLineEdit->setGeometry(QRect(10, 50, 61, 20));
        registerFPGAValueGetLineEdit->setFrame(false);
        registerFPGAValueGetLineEdit->setReadOnly(true);
        registerADCGroupBox = new QGroupBox(tab);
        registerADCGroupBox->setObjectName(QStringLiteral("registerADCGroupBox"));
        registerADCGroupBox->setGeometry(QRect(460, 10, 431, 141));
        registerADCSetPushButton = new QPushButton(registerADCGroupBox);
        registerADCSetPushButton->setObjectName(QStringLiteral("registerADCSetPushButton"));
        registerADCSetPushButton->setGeometry(QRect(50, 110, 75, 23));
        registerADCGetPushButton = new QPushButton(registerADCGroupBox);
        registerADCGetPushButton->setObjectName(QStringLiteral("registerADCGetPushButton"));
        registerADCGetPushButton->setGeometry(QRect(220, 110, 75, 23));
        registerADCSetGroupBox = new QGroupBox(registerADCGroupBox);
        registerADCSetGroupBox->setObjectName(QStringLiteral("registerADCSetGroupBox"));
        registerADCSetGroupBox->setGeometry(QRect(10, 20, 321, 81));
        registerADCSetGroupBox->setMinimumSize(QSize(4, 0));
        registerADCAddressSetLabel = new QLabel(registerADCSetGroupBox);
        registerADCAddressSetLabel->setObjectName(QStringLiteral("registerADCAddressSetLabel"));
        registerADCAddressSetLabel->setGeometry(QRect(10, 22, 71, 16));
        registerADCValueSetLabel = new QLabel(registerADCSetGroupBox);
        registerADCValueSetLabel->setObjectName(QStringLiteral("registerADCValueSetLabel"));
        registerADCValueSetLabel->setGeometry(QRect(10, 52, 41, 16));
        registerADCAddressSetComboBox = new QComboBox(registerADCSetGroupBox);
        registerADCAddressSetComboBox->setObjectName(QStringLiteral("registerADCAddressSetComboBox"));
        registerADCAddressSetComboBox->setGeometry(QRect(90, 20, 211, 22));
        registerADCAddressSetComboBox->setEditable(true);
        registerADCValueSetLineEdit = new QLineEdit(registerADCSetGroupBox);
        registerADCValueSetLineEdit->setObjectName(QStringLiteral("registerADCValueSetLineEdit"));
        registerADCValueSetLineEdit->setGeometry(QRect(90, 50, 61, 20));
        registerADCValueSetLineEdit->setFrame(true);
        registerADCGetGroupBox = new QGroupBox(registerADCGroupBox);
        registerADCGetGroupBox->setObjectName(QStringLiteral("registerADCGetGroupBox"));
        registerADCGetGroupBox->setGeometry(QRect(340, 20, 81, 81));
        registerADCGetGroupBox->setMinimumSize(QSize(4, 0));
        registerADCAddressGetLineEdit = new QLineEdit(registerADCGetGroupBox);
        registerADCAddressGetLineEdit->setObjectName(QStringLiteral("registerADCAddressGetLineEdit"));
        registerADCAddressGetLineEdit->setGeometry(QRect(10, 20, 61, 20));
        registerADCAddressGetLineEdit->setFrame(false);
        registerADCAddressGetLineEdit->setReadOnly(true);
        registerADCValueGetLineEdit = new QLineEdit(registerADCGetGroupBox);
        registerADCValueGetLineEdit->setObjectName(QStringLiteral("registerADCValueGetLineEdit"));
        registerADCValueGetLineEdit->setGeometry(QRect(10, 50, 61, 20));
        registerADCValueGetLineEdit->setFrame(false);
        registerADCValueGetLineEdit->setReadOnly(true);
        interfaceTabs->addTab(tab, QString());
        calibrationTab = new QWidget();
        calibrationTab->setObjectName(QStringLiteral("calibrationTab"));
        internalCalibrationGroupBox = new QGroupBox(calibrationTab);
        internalCalibrationGroupBox->setObjectName(QStringLiteral("internalCalibrationGroupBox"));
        internalCalibrationGroupBox->setGeometry(QRect(400, 10, 201, 211));
        calibrationChannelMaskGroupBox = new QGroupBox(internalCalibrationGroupBox);
        calibrationChannelMaskGroupBox->setObjectName(QStringLiteral("calibrationChannelMaskGroupBox"));
        calibrationChannelMaskGroupBox->setGeometry(QRect(10, 80, 171, 61));
        calibrationChannelMaskGroupBox->setFlat(true);
        calibrationChannelMaskGroupBox->setCheckable(false);
        calibrationChannel7CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel7CheckBox->setObjectName(QStringLiteral("calibrationChannel7CheckBox"));
        calibrationChannel7CheckBox->setGeometry(QRect(130, 20, 31, 17));
        calibrationChannel6CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel6CheckBox->setObjectName(QStringLiteral("calibrationChannel6CheckBox"));
        calibrationChannel6CheckBox->setGeometry(QRect(90, 20, 31, 17));
        calibrationChannel5CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel5CheckBox->setObjectName(QStringLiteral("calibrationChannel5CheckBox"));
        calibrationChannel5CheckBox->setGeometry(QRect(50, 20, 31, 17));
        calibrationChannel2CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel2CheckBox->setObjectName(QStringLiteral("calibrationChannel2CheckBox"));
        calibrationChannel2CheckBox->setGeometry(QRect(70, 40, 31, 17));
        calibrationChannel3CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel3CheckBox->setObjectName(QStringLiteral("calibrationChannel3CheckBox"));
        calibrationChannel3CheckBox->setGeometry(QRect(110, 40, 31, 17));
        calibrationChannel1CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel1CheckBox->setObjectName(QStringLiteral("calibrationChannel1CheckBox"));
        calibrationChannel1CheckBox->setGeometry(QRect(30, 40, 31, 17));
        calibrationChannel1CheckBox->setChecked(false);
        calibrationChannel4CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel4CheckBox->setObjectName(QStringLiteral("calibrationChannel4CheckBox"));
        calibrationChannel4CheckBox->setGeometry(QRect(140, 40, 31, 17));
        calibrationBetaLabel = new QLabel(internalCalibrationGroupBox);
        calibrationBetaLabel->setObjectName(QStringLiteral("calibrationBetaLabel"));
        calibrationBetaLabel->setGeometry(QRect(20, 53, 31, 16));
        calibrationBetaLabel->setFrameShape(QFrame::NoFrame);
        calibrationBetaDoubleSpinBox = new QDoubleSpinBox(internalCalibrationGroupBox);
        calibrationBetaDoubleSpinBox->setObjectName(QStringLiteral("calibrationBetaDoubleSpinBox"));
        calibrationBetaDoubleSpinBox->setGeometry(QRect(91, 50, 51, 22));
        calibrationBetaDoubleSpinBox->setDecimals(1);
        calibrationBetaDoubleSpinBox->setMaximum(1);
        calibrationBetaDoubleSpinBox->setSingleStep(0.1);
        calibrationFrameQtySpinBox = new QSpinBox(internalCalibrationGroupBox);
        calibrationFrameQtySpinBox->setObjectName(QStringLiteral("calibrationFrameQtySpinBox"));
        calibrationFrameQtySpinBox->setGeometry(QRect(90, 20, 51, 22));
        calibrationFrameQtySpinBox->setMinimum(1);
        calibrationFrameQtySpinBox->setMaximum(255);
        calibrationFrameQtySpinBox->setValue(100);
        calibrationFrameQtyLabel = new QLabel(internalCalibrationGroupBox);
        calibrationFrameQtyLabel->setObjectName(QStringLiteral("calibrationFrameQtyLabel"));
        calibrationFrameQtyLabel->setGeometry(QRect(20, 23, 61, 16));
        calibrationFrameQtyLabel->setFrameShape(QFrame::NoFrame);
        calibrateButton = new QPushButton(internalCalibrationGroupBox);
        calibrateButton->setObjectName(QStringLiteral("calibrateButton"));
        calibrateButton->setGeometry(QRect(60, 180, 75, 20));
        calibrateButton->setCheckable(true);
        calibrateButton->setChecked(false);
        calibrateButton->setAutoDefault(false);
        calibrateButton->setDefault(false);
        calibrateButton->setFlat(false);
        externalcalibrationGroupBox = new QGroupBox(calibrationTab);
        externalcalibrationGroupBox->setObjectName(QStringLiteral("externalcalibrationGroupBox"));
        externalcalibrationGroupBox->setGeometry(QRect(10, 10, 371, 211));
        sensorPositionGroupBox = new QGroupBox(externalcalibrationGroupBox);
        sensorPositionGroupBox->setObjectName(QStringLiteral("sensorPositionGroupBox"));
        sensorPositionGroupBox->setGeometry(QRect(10, 20, 341, 51));
        sensorPositionGroupBox->setFlat(true);
        sensorHeightSpinBox = new QDoubleSpinBox(sensorPositionGroupBox);
        sensorHeightSpinBox->setObjectName(QStringLiteral("sensorHeightSpinBox"));
        sensorHeightSpinBox->setGeometry(QRect(100, 20, 62, 22));
        sensorHeightSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensorHeightSpinBox->setMinimum(0.1);
        sensorHeightSpinBox->setMaximum(10);
        sensorHeightSpinBox->setSingleStep(0.1);
        sensorHeightSpinBox->setValue(1.38);
        sensorHeightLabel = new QLabel(sensorPositionGroupBox);
        sensorHeightLabel->setObjectName(QStringLiteral("sensorHeightLabel"));
        sensorHeightLabel->setGeometry(QRect(20, 21, 71, 20));
        sensorDepthSpinBox = new QDoubleSpinBox(sensorPositionGroupBox);
        sensorDepthSpinBox->setObjectName(QStringLiteral("sensorDepthSpinBox"));
        sensorDepthSpinBox->setGeometry(QRect(260, 19, 62, 22));
        sensorDepthSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensorDepthSpinBox->setMinimum(-100);
        sensorDepthSpinBox->setMaximum(10);
        sensorDepthSpinBox->setSingleStep(0.1);
        sensorDepthSpinBox->setValue(-1);
        sensorDepthLabel = new QLabel(sensorPositionGroupBox);
        sensorDepthLabel->setObjectName(QStringLiteral("sensorDepthLabel"));
        sensorDepthLabel->setGeometry(QRect(180, 20, 71, 20));
        receiverCalibrationGroupBox = new QGroupBox(externalcalibrationGroupBox);
        receiverCalibrationGroupBox->setObjectName(QStringLiteral("receiverCalibrationGroupBox"));
        receiverCalibrationGroupBox->setGeometry(QRect(10, 150, 341, 51));
        receiverCalibrationGroupBox->setFlat(true);
        measurementOffsetLabel = new QLabel(receiverCalibrationGroupBox);
        measurementOffsetLabel->setObjectName(QStringLiteral("measurementOffsetLabel"));
        measurementOffsetLabel->setGeometry(QRect(20, 23, 71, 16));
        measurementOffsetSpinBox = new QDoubleSpinBox(receiverCalibrationGroupBox);
        measurementOffsetSpinBox->setObjectName(QStringLiteral("measurementOffsetSpinBox"));
        measurementOffsetSpinBox->setGeometry(QRect(100, 20, 62, 22));
        measurementOffsetSpinBox->setDecimals(1);
        measurementOffsetSpinBox->setMinimum(-40);
        measurementOffsetSpinBox->setMaximum(40);
        measurementOffsetSpinBox->setSingleStep(0.1);
        sensorRangeGroupBox = new QGroupBox(externalcalibrationGroupBox);
        sensorRangeGroupBox->setObjectName(QStringLiteral("sensorRangeGroupBox"));
        sensorRangeGroupBox->setGeometry(QRect(10, 80, 341, 51));
        sensorRangeGroupBox->setFlat(true);
        sensorRangeMinLabel = new QLabel(sensorRangeGroupBox);
        sensorRangeMinLabel->setObjectName(QStringLiteral("sensorRangeMinLabel"));
        sensorRangeMinLabel->setGeometry(QRect(20, 23, 61, 16));
        sensorRangeMaxLabel = new QLabel(sensorRangeGroupBox);
        sensorRangeMaxLabel->setObjectName(QStringLiteral("sensorRangeMaxLabel"));
        sensorRangeMaxLabel->setGeometry(QRect(180, 23, 61, 16));
        sensorRangeMinSpinBox = new QDoubleSpinBox(sensorRangeGroupBox);
        sensorRangeMinSpinBox->setObjectName(QStringLiteral("sensorRangeMinSpinBox"));
        sensorRangeMinSpinBox->setGeometry(QRect(100, 20, 62, 22));
        sensorRangeMinSpinBox->setDecimals(1);
        sensorRangeMinSpinBox->setMaximum(1000);
        sensorRangeMinSpinBox->setSingleStep(0.1);
        sensorRangeMaxSpinBox = new QDoubleSpinBox(sensorRangeGroupBox);
        sensorRangeMaxSpinBox->setObjectName(QStringLiteral("sensorRangeMaxSpinBox"));
        sensorRangeMaxSpinBox->setGeometry(QRect(260, 20, 62, 22));
        sensorRangeMaxSpinBox->setDecimals(1);
        sensorRangeMaxSpinBox->setMaximum(1000);
        sensorRangeMaxSpinBox->setSingleStep(0.1);
        sensorRangeMaxSpinBox->setValue(30);
        interfaceTabs->addTab(calibrationTab, QString());
        threeDOptionsTab = new QWidget();
        threeDOptionsTab->setObjectName(QStringLiteral("threeDOptionsTab"));
        viewerColorFrame = new QFrame(threeDOptionsTab);
        viewerColorFrame->setObjectName(QStringLiteral("viewerColorFrame"));
        viewerColorFrame->setGeometry(QRect(10, 10, 661, 191));
        viewerColorFrame->setFrameShape(QFrame::WinPanel);
        viewerColorFrame->setFrameShadow(QFrame::Raised);
        viewColorGroupBox = new QGroupBox(viewerColorFrame);
        viewColorGroupBox->setObjectName(QStringLiteral("viewColorGroupBox"));
        viewColorGroupBox->setGeometry(QRect(200, 10, 171, 81));
        colorImageRadioButton = new QRadioButton(viewColorGroupBox);
        colorImageRadioButton->setObjectName(QStringLiteral("colorImageRadioButton"));
        colorImageRadioButton->setGeometry(QRect(10, 28, 82, 17));
        colorImageRadioButton->setChecked(true);
        rangeImageRadioButton = new QRadioButton(viewColorGroupBox);
        rangeImageRadioButton->setObjectName(QStringLiteral("rangeImageRadioButton"));
        rangeImageRadioButton->setGeometry(QRect(10, 48, 82, 17));
        cameraViewGroupBox = new QGroupBox(viewerColorFrame);
        cameraViewGroupBox->setObjectName(QStringLiteral("cameraViewGroupBox"));
        cameraViewGroupBox->setGeometry(QRect(390, 10, 111, 171));
        viewSidePushButton = new QPushButton(cameraViewGroupBox);
        viewSidePushButton->setObjectName(QStringLiteral("viewSidePushButton"));
        viewSidePushButton->setGeometry(QRect(10, 20, 81, 23));
        viewTopPushButton = new QPushButton(cameraViewGroupBox);
        viewTopPushButton->setObjectName(QStringLiteral("viewTopPushButton"));
        viewTopPushButton->setGeometry(QRect(10, 50, 81, 23));
        viewIsoPushButton = new QPushButton(cameraViewGroupBox);
        viewIsoPushButton->setObjectName(QStringLiteral("viewIsoPushButton"));
        viewIsoPushButton->setGeometry(QRect(10, 80, 81, 23));
        viewFrontPushButton = new QPushButton(cameraViewGroupBox);
        viewFrontPushButton->setObjectName(QStringLiteral("viewFrontPushButton"));
        viewFrontPushButton->setGeometry(QRect(10, 110, 81, 23));
        viewFrontPushButton_2 = new QPushButton(cameraViewGroupBox);
        viewFrontPushButton_2->setObjectName(QStringLiteral("viewFrontPushButton_2"));
        viewFrontPushButton_2->setGeometry(QRect(10, 140, 81, 23));
        displayOptionsGroupBox = new QGroupBox(viewerColorFrame);
        displayOptionsGroupBox->setObjectName(QStringLiteral("displayOptionsGroupBox"));
        displayOptionsGroupBox->setGeometry(QRect(20, 10, 171, 81));
        decimationSpinBox = new QSpinBox(displayOptionsGroupBox);
        decimationSpinBox->setObjectName(QStringLiteral("decimationSpinBox"));
        decimationSpinBox->setGeometry(QRect(90, 20, 61, 22));
        decimationSpinBox->setFrame(true);
        decimationSpinBox->setMinimum(1);
        decimationSpinBox->setMaximum(8);
        decimationSpinBox->setSingleStep(1);
        decimationLabel = new QLabel(displayOptionsGroupBox);
        decimationLabel->setObjectName(QStringLiteral("decimationLabel"));
        decimationLabel->setGeometry(QRect(10, 23, 61, 20));
        pixelSizeLabel = new QLabel(displayOptionsGroupBox);
        pixelSizeLabel->setObjectName(QStringLiteral("pixelSizeLabel"));
        pixelSizeLabel->setGeometry(QRect(10, 50, 61, 20));
        pixelSizeSpinBox = new QSpinBox(displayOptionsGroupBox);
        pixelSizeSpinBox->setObjectName(QStringLiteral("pixelSizeSpinBox"));
        pixelSizeSpinBox->setGeometry(QRect(90, 50, 61, 22));
        pixelSizeSpinBox->setFrame(true);
        pixelSizeSpinBox->setMinimum(1);
        pixelSizeSpinBox->setMaximum(15);
        pixelSizeSpinBox->setSingleStep(1);
        interfaceTabs->addTab(threeDOptionsTab, QString());
        AWLQtDemoClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(AWLQtDemoClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1042, 21));
        menuFichier = new QMenu(menuBar);
        menuFichier->setObjectName(QStringLiteral("menuFichier"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QStringLiteral("menuView"));
        AWLQtDemoClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(AWLQtDemoClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        AWLQtDemoClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(AWLQtDemoClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        AWLQtDemoClass->setStatusBar(statusBar);
        QWidget::setTabOrder(interfaceTabs, decimationSpinBox);
        QWidget::setTabOrder(decimationSpinBox, pixelSizeSpinBox);
        QWidget::setTabOrder(pixelSizeSpinBox, colorImageRadioButton);
        QWidget::setTabOrder(colorImageRadioButton, rangeImageRadioButton);
        QWidget::setTabOrder(rangeImageRadioButton, viewSidePushButton);
        QWidget::setTabOrder(viewSidePushButton, viewTopPushButton);
        QWidget::setTabOrder(viewTopPushButton, viewIsoPushButton);
        QWidget::setTabOrder(viewIsoPushButton, viewFrontPushButton);
        QWidget::setTabOrder(viewFrontPushButton, sensorHeightSpinBox);
        QWidget::setTabOrder(sensorHeightSpinBox, sensorDepthSpinBox);
        QWidget::setTabOrder(sensorDepthSpinBox, distanceTable5);
        QWidget::setTabOrder(distanceTable5, distanceTable6);
        QWidget::setTabOrder(distanceTable6, distanceTable7);
        QWidget::setTabOrder(distanceTable7, distanceTable1);
        QWidget::setTabOrder(distanceTable1, distanceTable2);
        QWidget::setTabOrder(distanceTable2, distanceTable3);
        QWidget::setTabOrder(distanceTable3, distanceTable4);

        menuBar->addAction(menuFichier->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuFichier->addAction(actionQuitter);
        menuView->addAction(action2D);
        menuView->addAction(action3D_View);
        menuView->addAction(actionGraph);
        menuView->addAction(actionCamera);

        retranslateUi(AWLQtDemoClass);
        QObject::connect(decimationSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_decimationSpin_editingFinished()));
        QObject::connect(pixelSizeSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_pixelSizeSpin_editingFinished()));
        QObject::connect(colorImageRadioButton, SIGNAL(toggled(bool)), AWLQtDemoClass, SLOT(on_colorImageRadioButton_setChecked(bool)));
        QObject::connect(rangeImageRadioButton, SIGNAL(toggled(bool)), AWLQtDemoClass, SLOT(on_rangeImageRadioButton_setChecked(bool)));
        QObject::connect(viewSidePushButton, SIGNAL(pressed()), AWLQtDemoClass, SLOT(on_viewFrontPushButton_pressed()));
        QObject::connect(viewTopPushButton, SIGNAL(pressed()), AWLQtDemoClass, SLOT(on_viewTopPushButton_pressed()));
        QObject::connect(viewFrontPushButton, SIGNAL(pressed()), AWLQtDemoClass, SLOT(on_viewFrontPushButton_pressed()));
        QObject::connect(sensorHeightSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_registerFPGASetPushButton_clicked()));
        QObject::connect(sensorDepthSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_sensorDepthSpin_editingFinished()));
        QObject::connect(viewFrontPushButton_2, SIGNAL(pressed()), AWLQtDemoClass, SLOT(on_viewZoomPushButton_pressed()));
        QObject::connect(viewIsoPushButton, SIGNAL(pressed()), AWLQtDemoClass, SLOT(on_viewIsoPushButton_pressed()));
        QObject::connect(measurementOffsetSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_measurementOffsetSpin_editingFinished()));
        QObject::connect(injectSimulatedCheckbox, SIGNAL(toggled(bool)), AWLQtDemoClass, SLOT(on_simulatedDataInjectCheckBox_setChecked(bool)));
        QObject::connect(recordButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_recordPushButton_clicked()));
        QObject::connect(playbackButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_playbackPushButton_clicked()));
        QObject::connect(stopButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_stopPushButton_clicked()));
        QObject::connect(calibrateButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_calibratePushButton_clicked()));
        QObject::connect(registerFPGASetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(onRegisterFPGASetPushButton_clicked()));
        QObject::connect(registerFPGAGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerFPGAGetPushButton_clicked()));
        QObject::connect(registerADCSetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerADCSetPushButton_clicked()));
        QObject::connect(registerADCGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerADCGetPushButton_clicked()));

        interfaceTabs->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(AWLQtDemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *AWLQtDemoClass)
    {
        AWLQtDemoClass->setWindowTitle(QApplication::translate("AWLQtDemoClass", "AWLQtDemo", 0));
        actionQuitter->setText(QApplication::translate("AWLQtDemoClass", "Quitter", 0));
        action3D_View->setText(QApplication::translate("AWLQtDemoClass", "3D", 0));
        actionGraph->setText(QApplication::translate("AWLQtDemoClass", "Scope", 0));
        action2D->setText(QApplication::translate("AWLQtDemoClass", "2D", 0));
        actionCamera->setText(QApplication::translate("AWLQtDemoClass", "Camera", 0));
        QTableWidgetItem *___qtablewidgetitem = distanceTable5->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem1 = distanceTable5->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem2 = distanceTable5->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem3 = distanceTable5->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        QTableWidgetItem *___qtablewidgetitem4 = distanceTable6->horizontalHeaderItem(0);
        ___qtablewidgetitem4->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem5 = distanceTable6->horizontalHeaderItem(1);
        ___qtablewidgetitem5->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem6 = distanceTable6->horizontalHeaderItem(2);
        ___qtablewidgetitem6->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem7 = distanceTable6->horizontalHeaderItem(3);
        ___qtablewidgetitem7->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        QTableWidgetItem *___qtablewidgetitem8 = distanceTable7->horizontalHeaderItem(0);
        ___qtablewidgetitem8->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem9 = distanceTable7->horizontalHeaderItem(1);
        ___qtablewidgetitem9->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem10 = distanceTable7->horizontalHeaderItem(2);
        ___qtablewidgetitem10->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem11 = distanceTable7->horizontalHeaderItem(3);
        ___qtablewidgetitem11->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        QTableWidgetItem *___qtablewidgetitem12 = distanceTable1->horizontalHeaderItem(0);
        ___qtablewidgetitem12->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem13 = distanceTable1->horizontalHeaderItem(1);
        ___qtablewidgetitem13->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem14 = distanceTable1->horizontalHeaderItem(2);
        ___qtablewidgetitem14->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem15 = distanceTable1->horizontalHeaderItem(3);
        ___qtablewidgetitem15->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        QTableWidgetItem *___qtablewidgetitem16 = distanceTable2->horizontalHeaderItem(0);
        ___qtablewidgetitem16->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem17 = distanceTable2->horizontalHeaderItem(1);
        ___qtablewidgetitem17->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem18 = distanceTable2->horizontalHeaderItem(2);
        ___qtablewidgetitem18->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem19 = distanceTable2->horizontalHeaderItem(3);
        ___qtablewidgetitem19->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        QTableWidgetItem *___qtablewidgetitem20 = distanceTable3->horizontalHeaderItem(0);
        ___qtablewidgetitem20->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem21 = distanceTable3->horizontalHeaderItem(1);
        ___qtablewidgetitem21->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem22 = distanceTable3->horizontalHeaderItem(2);
        ___qtablewidgetitem22->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem23 = distanceTable3->horizontalHeaderItem(3);
        ___qtablewidgetitem23->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        QTableWidgetItem *___qtablewidgetitem24 = distanceTable4->horizontalHeaderItem(0);
        ___qtablewidgetitem24->setText(QApplication::translate("AWLQtDemoClass", "Distance", 0));
        QTableWidgetItem *___qtablewidgetitem25 = distanceTable4->horizontalHeaderItem(1);
        ___qtablewidgetitem25->setText(QApplication::translate("AWLQtDemoClass", "Velocity", 0));
        QTableWidgetItem *___qtablewidgetitem26 = distanceTable4->horizontalHeaderItem(2);
        ___qtablewidgetitem26->setText(QApplication::translate("AWLQtDemoClass", "Track", 0));
        QTableWidgetItem *___qtablewidgetitem27 = distanceTable4->horizontalHeaderItem(3);
        ___qtablewidgetitem27->setText(QApplication::translate("AWLQtDemoClass", "Level", 0));
        interfaceTabs->setTabText(interfaceTabs->indexOf(realTimeTab), QApplication::translate("AWLQtDemoClass", "Real-Time", 0));
        recordPlayGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Record / Play", 0));
        recordButton->setText(QApplication::translate("AWLQtDemoClass", "Record", 0));
        playbackButton->setText(QApplication::translate("AWLQtDemoClass", "Playback", 0));
        channelMaskGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Channel mask", 0));
        channel7CheckBox->setText(QApplication::translate("AWLQtDemoClass", "7", 0));
        channel6CheckBox->setText(QApplication::translate("AWLQtDemoClass", "6", 0));
        channel5CheckBox->setText(QApplication::translate("AWLQtDemoClass", "5", 0));
        channel2CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 2", 0));
        channel3CheckBox->setText(QApplication::translate("AWLQtDemoClass", "3", 0));
        channel1CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 1", 0));
        channel4CheckBox->setText(QApplication::translate("AWLQtDemoClass", "4", 0));
        frameRateLabel->setText(QApplication::translate("AWLQtDemoClass", "Frame rate:", 0));
        frameRateSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "Hz.", 0));
        stopButton->setText(QApplication::translate("AWLQtDemoClass", "Stop", 0));
        recordFileNameGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "File names", 0));
        label->setText(QApplication::translate("AWLQtDemoClass", "Record:", 0));
        label_2->setText(QApplication::translate("AWLQtDemoClass", "Playback:", 0));
        algoGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Detection Algorithm", 0));
        algo1Checkbox->setText(QApplication::translate("AWLQtDemoClass", "Threshold", 0));
        algo2RadioButton->setText(QApplication::translate("AWLQtDemoClass", "Wavelet 1", 0));
        algo3RadioButton->setText(QApplication::translate("AWLQtDemoClass", "Wavelet 2", 0));
        algo4RadioButton->setText(QApplication::translate("AWLQtDemoClass", "Double-d", 0));
        injectSimulatedGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Inject Ramp", 0));
        injectSimulatedCheckbox->setText(QApplication::translate("AWLQtDemoClass", "Enabled", 0));
        interfaceTabs->setTabText(interfaceTabs->indexOf(controlTab), QApplication::translate("AWLQtDemoClass", "Control", 0));
        VersionLabel->setText(QApplication::translate("AWLQtDemoClass", "Version:", 0));
        TemperatureLabel->setText(QApplication::translate("AWLQtDemoClass", "Temp:", 0));
        TemperatureLabel_2->setText(QApplication::translate("AWLQtDemoClass", "<html><head/><body><p><span style=\" vertical-align:super;\">o</span>C</p></body></html>", 0));
        VoltageLabel->setText(QApplication::translate("AWLQtDemoClass", "Voltage:", 0));
        VoltageLabel_2->setText(QApplication::translate("AWLQtDemoClass", "volts", 0));
        bootGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Boot errors", 0));
        bootReceiverCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Receiver", 0));
        bootEmitter1CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 1", 0));
        bootAuxChecksumCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Aux. checksum", 0));
        bootEmitter2CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 2", 0));
        bootMainChecksumCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Main checksum", 0));
        bootMemoryCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Memory", 0));
        bootDSPCheckBox->setText(QApplication::translate("AWLQtDemoClass", "DSP", 0));
        bootChecksumCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Checksum", 0));
        statusGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Status", 0));
        statusSelfTestCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Self test", 0));
        statusShutdownCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Shutdown", 0));
        statusSensorBlockedCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Sensor blocked", 0));
        statusReducedPerformanceCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Reduced performance", 0));
        statusSaturationCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Saturation", 0));
        statusSaturationCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Saturation", 0));
        statusSensorBlockedCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Sensor blocked", 0));
        statusShutdownCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Shutdown", 0));
        statusReducedPerformanceCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Reduced performance", 0));
        statusSelfTestCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Self test", 0));
        hardwareGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Hardware errors", 0));
        hardwareReceiverCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Receiver", 0));
        hardwareEmitter1CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 1", 0));
        hardwareEmitter2CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 2", 0));
        hardwareMemoryCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Memory", 0));
        hardwareDSPCheckBox->setText(QApplication::translate("AWLQtDemoClass", "DSP", 0));
        receiverGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "ReceiverErrors", 0));
        receiverChannel3CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 3", 0));
        receiverChannel1CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 1", 0));
        receiverChannel2CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 2", 0));
        receiverChannel5CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 5", 0));
        receiverChannel4CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 4", 0));
        receiverChannel7CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 7", 0));
        receiverChannel6CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 6", 0));
        interfaceTabs->setTabText(interfaceTabs->indexOf(statusTab), QApplication::translate("AWLQtDemoClass", "Status", 0));
        registerFPGAGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "AWL Registers", 0));
        registerFPGASetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", 0));
        registerFPGAGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", 0));
        registerFPGASetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Set", 0));
        registerFPGAAddressSetLabel->setText(QApplication::translate("AWLQtDemoClass", "AWL Register:", 0));
        registerFPGAValueSetLabel->setText(QApplication::translate("AWLQtDemoClass", "Value:", 0));
        registerFPGAValueSetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", 0));
        registerFPGAValueSetLineEdit->setText(QApplication::translate("AWLQtDemoClass", "FF", 0));
        registerFPGAGetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Confirm", 0));
        registerFPGAAddressGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", 0));
        registerFPGAValueGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", 0));
        registerADCGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "ADC Registers", 0));
        registerADCSetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", 0));
        registerADCGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", 0));
        registerADCSetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Set", 0));
        registerADCAddressSetLabel->setText(QApplication::translate("AWLQtDemoClass", "ADC Register:", 0));
        registerADCValueSetLabel->setText(QApplication::translate("AWLQtDemoClass", "Value:", 0));
        registerADCValueSetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", 0));
        registerADCValueSetLineEdit->setText(QApplication::translate("AWLQtDemoClass", "FF", 0));
        registerADCGetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Confirm", 0));
        registerADCAddressGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", 0));
        registerADCValueGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", 0));
        interfaceTabs->setTabText(interfaceTabs->indexOf(tab), QApplication::translate("AWLQtDemoClass", "Registers", 0));
        internalCalibrationGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Internal Calibration", 0));
        calibrationChannelMaskGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Channel mask", 0));
        calibrationChannel7CheckBox->setText(QApplication::translate("AWLQtDemoClass", "7", 0));
        calibrationChannel6CheckBox->setText(QApplication::translate("AWLQtDemoClass", "6", 0));
        calibrationChannel5CheckBox->setText(QApplication::translate("AWLQtDemoClass", "5", 0));
        calibrationChannel2CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 2", 0));
        calibrationChannel3CheckBox->setText(QApplication::translate("AWLQtDemoClass", "3", 0));
        calibrationChannel1CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 1", 0));
        calibrationChannel4CheckBox->setText(QApplication::translate("AWLQtDemoClass", "4", 0));
        calibrationBetaLabel->setText(QApplication::translate("AWLQtDemoClass", "Beta:", 0));
        calibrationFrameQtySpinBox->setSuffix(QString());
        calibrationFrameQtyLabel->setText(QApplication::translate("AWLQtDemoClass", "Frame qty:", 0));
        calibrateButton->setText(QApplication::translate("AWLQtDemoClass", "Calibrate", 0));
        externalcalibrationGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Sensor Range and position", 0));
        sensorPositionGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Sensor Position", 0));
        sensorHeightSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", 0));
        sensorHeightLabel->setText(QApplication::translate("AWLQtDemoClass", "Sensor height:", 0));
        sensorDepthSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", 0));
        sensorDepthLabel->setText(QApplication::translate("AWLQtDemoClass", "Sensor depth:", 0));
        receiverCalibrationGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Measurement Offsets", 0));
        measurementOffsetLabel->setText(QApplication::translate("AWLQtDemoClass", "Range offset:", 0));
        measurementOffsetSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", 0));
        sensorRangeGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Displayed Range", 0));
        sensorRangeMinLabel->setText(QApplication::translate("AWLQtDemoClass", "Range min:", 0));
        sensorRangeMaxLabel->setText(QApplication::translate("AWLQtDemoClass", "Range max:", 0));
        sensorRangeMinSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", 0));
        sensorRangeMaxSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", 0));
        interfaceTabs->setTabText(interfaceTabs->indexOf(calibrationTab), QApplication::translate("AWLQtDemoClass", "Calibration", 0));
        viewColorGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "3D color style", 0));
        colorImageRadioButton->setText(QApplication::translate("AWLQtDemoClass", "I&mage", 0));
        colorImageRadioButton->setShortcut(QApplication::translate("AWLQtDemoClass", "M", 0));
        rangeImageRadioButton->setText(QApplication::translate("AWLQtDemoClass", "&Range", 0));
        rangeImageRadioButton->setShortcut(QApplication::translate("AWLQtDemoClass", "R", 0));
        cameraViewGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Camera view angle", 0));
        viewSidePushButton->setText(QApplication::translate("AWLQtDemoClass", "&Side", 0));
        viewSidePushButton->setShortcut(QApplication::translate("AWLQtDemoClass", "S", 0));
        viewTopPushButton->setText(QApplication::translate("AWLQtDemoClass", "&Top", 0));
        viewTopPushButton->setShortcut(QApplication::translate("AWLQtDemoClass", "T", 0));
        viewIsoPushButton->setText(QApplication::translate("AWLQtDemoClass", "&Isometric", 0));
        viewIsoPushButton->setShortcut(QApplication::translate("AWLQtDemoClass", "I", 0));
        viewFrontPushButton->setText(QApplication::translate("AWLQtDemoClass", "&Front", 0));
        viewFrontPushButton->setShortcut(QApplication::translate("AWLQtDemoClass", "F", 0));
        viewFrontPushButton_2->setText(QApplication::translate("AWLQtDemoClass", "&Zoom", 0));
        viewFrontPushButton_2->setShortcut(QApplication::translate("AWLQtDemoClass", "F", 0));
        displayOptionsGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Display options", 0));
        decimationSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", " pixel", 0));
        decimationLabel->setText(QApplication::translate("AWLQtDemoClass", "Decimation:", 0));
        pixelSizeLabel->setText(QApplication::translate("AWLQtDemoClass", "Pixel size:", 0));
        pixelSizeSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", " pixel", 0));
        interfaceTabs->setTabText(interfaceTabs->indexOf(threeDOptionsTab), QApplication::translate("AWLQtDemoClass", "3D display options", 0));
        menuFichier->setTitle(QApplication::translate("AWLQtDemoClass", "Fichier", 0));
        menuView->setTitle(QApplication::translate("AWLQtDemoClass", "View", 0));
    } // retranslateUi

};

namespace Ui {
    class AWLQtDemoClass: public Ui_AWLQtDemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AWLQTDEMO_H
