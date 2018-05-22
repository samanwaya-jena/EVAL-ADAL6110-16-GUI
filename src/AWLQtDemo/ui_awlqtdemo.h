/********************************************************************************
** Form generated from reading UI file 'awlqtdemo.ui'
**
** Created by: Qt User Interface Compiler version 5.9.4
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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AWLQtDemoClass
{

public:
    QAction *actionQuitter;
    QAction *actionGraph;
    QAction *action2D;
    QAction *actionCamera;
    QAction *actionTableView;
    QAction *actionSettings;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QTabWidget *interfaceTabs;
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
    QGroupBox *externalCalibrationGroupBox;
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
    QListWidget *channelSelectListWidget;
    QGroupBox *targetHintGroupBox;
    QLabel *targetHintAngleLabel;
    QDoubleSpinBox *targetHintAngleSpinBox;
    QLabel *targetHintDistanceLabel;
    QDoubleSpinBox *targetHintDistanceSpinBox;
    QCheckBox *distanceLogFileCheckbox;
    QWidget *controlTab;
    QGroupBox *recordPlayGroupBox;
    QPushButton *recordButton;
    QPushButton *playbackButton;
    QGroupBox *channelMaskGroupBox;
    QCheckBox *recordChannel7CheckBox;
    QCheckBox *recordChannel6CheckBox;
    QCheckBox *recordChannel5CheckBox;
    QCheckBox *recordChannel2CheckBox;
    QCheckBox *recordChannel3CheckBox;
    QCheckBox *recordChannel1CheckBox;
    QCheckBox *recordChannel4CheckBox;
    QLabel *frameRateLabel;
    QSpinBox *frameRateSpinBox;
    QPushButton *stopButton;
    QGroupBox *recordFileNameGroupBox;
    QLineEdit *recordFileNameEdit;
    QLabel *label;
    QLabel *label_2;
    QLineEdit *playbackFileNameEdit;
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
    QWidget *registersTab;
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
    QWidget *gpiosTab;
    QGroupBox *groupBox;
    QListWidget *registerGPIOListWidget;
    QPushButton *registerGPIOGetPushButton;
    QPushButton *registerGPIOSetPushButton;
    QWidget *AlgoTab;
    QGroupBox *algoGroupBox;
    QGroupBox *algoSelectGroupBox;
    QComboBox *algoSelectComboBox;
    QTableWidget *algoParametersTable;
    QPushButton *algoParametersSetPushButton;
    QPushButton *algoParametersGetPushButton;
    QGroupBox *globalParametersGroupBox;
    QTableWidget *globalParametersTable;
    QPushButton *globalParametersSetPushButton;
    QPushButton *globalParametersGetPushButton;
    QWidget *TrackerTab;
    QGroupBox *trackerGroupBox;
    QGroupBox *trackerSelectGroupBox;
    QComboBox *trackerSelectComboBox;
    QTableWidget *trackerParametersTable;
    QPushButton *trackerParametersSetPushButton;
    QPushButton *trackerParametersGetPushButton;
    QGridLayout *gridDisplayLayout;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *AWLQtDemoClass)
    {
        if (AWLQtDemoClass->objectName().isEmpty())
            AWLQtDemoClass->setObjectName(QStringLiteral("AWLQtDemoClass"));
        AWLQtDemoClass->setWindowModality(Qt::NonModal);
        AWLQtDemoClass->resize(972, 309);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AWLQtDemoClass->sizePolicy().hasHeightForWidth());
        AWLQtDemoClass->setSizePolicy(sizePolicy);
        AWLQtDemoClass->setSizeIncrement(QSize(5, 5));
        AWLQtDemoClass->setBaseSize(QSize(980, 409));
        QPalette palette;
        QBrush brush(QColor(222, 222, 222, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QLinearGradient gradient(0, 0, 0, 1);
        gradient.setSpread(QGradient::PadSpread);
        gradient.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient.setColorAt(0, QColor(0, 0, 0, 255));
        gradient.setColorAt(0.5, QColor(16, 16, 16, 255));
        gradient.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush1(gradient);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        QLinearGradient gradient1(0, 0, 0, 1);
        gradient1.setSpread(QGradient::PadSpread);
        gradient1.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient1.setColorAt(0, QColor(0, 0, 0, 255));
        gradient1.setColorAt(0.5, QColor(16, 16, 16, 255));
        gradient1.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush2(gradient1);
        palette.setBrush(QPalette::Active, QPalette::Base, brush2);
        QLinearGradient gradient2(0, 0, 0, 1);
        gradient2.setSpread(QGradient::PadSpread);
        gradient2.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient2.setColorAt(0, QColor(0, 0, 0, 255));
        gradient2.setColorAt(0.5, QColor(16, 16, 16, 255));
        gradient2.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush3(gradient2);
        palette.setBrush(QPalette::Active, QPalette::Window, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        QLinearGradient gradient3(0, 0, 0, 1);
        gradient3.setSpread(QGradient::PadSpread);
        gradient3.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient3.setColorAt(0, QColor(0, 0, 0, 255));
        gradient3.setColorAt(0.5, QColor(16, 16, 16, 255));
        gradient3.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush4(gradient3);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        QLinearGradient gradient4(0, 0, 0, 1);
        gradient4.setSpread(QGradient::PadSpread);
        gradient4.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient4.setColorAt(0, QColor(0, 0, 0, 255));
        gradient4.setColorAt(0.5, QColor(16, 16, 16, 255));
        gradient4.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush5(gradient4);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush5);
        QLinearGradient gradient5(0, 0, 0, 1);
        gradient5.setSpread(QGradient::PadSpread);
        gradient5.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient5.setColorAt(0, QColor(0, 0, 0, 255));
        gradient5.setColorAt(0.5, QColor(16, 16, 16, 255));
        gradient5.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush6(gradient5);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush6);
        QBrush brush7(QColor(64, 64, 64, 255));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush7);
        QLinearGradient gradient6(0, 0, 0, 1);
        gradient6.setSpread(QGradient::PadSpread);
        gradient6.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient6.setColorAt(0, QColor(0, 0, 0, 255));
        gradient6.setColorAt(0.5, QColor(32, 32, 32, 255));
        gradient6.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush8(gradient6);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush8);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush7);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush7);
        QLinearGradient gradient7(0, 0, 0, 1);
        gradient7.setSpread(QGradient::PadSpread);
        gradient7.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient7.setColorAt(0, QColor(0, 0, 0, 255));
        gradient7.setColorAt(0.5, QColor(32, 32, 32, 255));
        gradient7.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush9(gradient7);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush9);
        QLinearGradient gradient8(0, 0, 0, 1);
        gradient8.setSpread(QGradient::PadSpread);
        gradient8.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient8.setColorAt(0, QColor(0, 0, 0, 255));
        gradient8.setColorAt(0.5, QColor(32, 32, 32, 255));
        gradient8.setColorAt(1, QColor(0, 0, 0, 255));
        QBrush brush10(gradient8);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush10);
        AWLQtDemoClass->setPalette(palette);
        QFont font;
        AWLQtDemoClass->setFont(font);
        QIcon icon;
        icon.addFile(QStringLiteral("AWLQtDemo.ico"), QSize(), QIcon::Normal, QIcon::Off);
        AWLQtDemoClass->setWindowIcon(icon);
        AWLQtDemoClass->setLayoutDirection(Qt::LeftToRight);
        AWLQtDemoClass->setAutoFillBackground(false);
        AWLQtDemoClass->setStyleSheet(QLatin1String("QWidget\n"
"{\n"
"background:transparent;\n"
"color: #dedede;\n"
"border-color:#dedede;\n"
"font-size: 11px;/*background-color: #000000;*/\n"
"background-color:qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(0, 0, 0, 0), stop:0.5 rgba(64, 64, 64, 0) stop:1 rgba(0, 0, 0, 0));\n"
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
"QWidget:disabled\n"
"{\n"
"color: #404040;\n"
"/*background-color: #202020;*/\n"
"/*background-color: #000000;*/\n"
"background-color:qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(0, 0, 0, 255), stop:0.5 rgba(32, 32, 32, 255) stop:1 rgba(0, 0, 0, 255));\n"
"}\n"
"\n"
"QWidget:focus\n"
"{\n"
"/*border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);*"
                        "/\n"
"\n"
"}\n"
"\n"
"QFrame\n"
"{\n"
"background:transparent;\n"
"}\n"
"\n"
"QMainWindow\n"
"{\n"
"background-color:qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(0, 0, 0, 255), stop:0.5 rgba(16, 16, 16, 255) stop:1 rgba(0, 0, 0, 255));\n"
"}\n"
"\n"
"QMainWindow::separator\n"
"{\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #161616, stop: 0.5 #151515, stop: 0.6 #212121, stop:1 #343434);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #4c4c4c;\n"
"spacing: 10px; /* spacing between items in the tool bar */\n"
"}\n"
"\n"
"QMainWindow::separator:hover\n"
"{\n"
"\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d7801a, stop:0.5 #b56c17 stop:1 #ffa02f);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #6c6c6c;\n"
"spacing: 10px; /* spacing between items in the tool bar */\n"
"}\n"
"\n"
"/* Provide a padding for the layout inside the frame. The frame\n"
"   exists only to provide a padding for the top-right image, so we\n"
"  "
                        " explicitly disable the border. */\n"
"#mainFrame {\n"
"    padding-right: 30px;\n"
"    border-style: none;\n"
"    border-image: none; /* since we set a border-image below */\n"
"}\n"
"\n"
"/* mainFrame won't have this border-image since we have\n"
"   explicitly set it to 'none' using a more specific selector. */\n"
"QLineEdit, QComboBox[editable=\"true\"], QSpinBox, QAbstractSpinBox {\n"
"    border-image: url(:/Widgets/Images/Widgets/frame.png) 4;\n"
"    border-width: 3;\n"
"}\n"
"\n"
"/* color for hiliting selected text should be orange instead of default blue */\n"
"QLineEdit, QComboBox[editable=\"true\"], QSpinBox, QAbstractSpinBox, QAbstractItemView {\n"
"	selection-background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QLabel {\n"
"	background:transparent;\n"
"    border: none;\n"
"    border-image: none;\n"
"    padding: 0;\n"
"    background: none;\n"
"}\n"
"\n"
"QMenuBar\n"
"{\n"
"	font-size: 18px;\n"
"	spacing:20px;\n"
"}\n"
"\n"
"QMenuBa"
                        "r::item\n"
"{\n"
"background: transparent;\n"
"border: 1px solid #000000;\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"spacing:20px;\n"
"font-size: 18px;\n"
"}\n"
"\n"
"QMenuBar::item:selected\n"
"{\n"
"background: transparent;\n"
"border: 1px solid #ffaa00;\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"}\n"
"\n"
"QMenuBar::item:pressed\n"
"{\n"
"background: #444;\n"
"border: 1px solid #000000;\n"
"background-color: QLinearGradient(\n"
"x1:0, y1:0,\n"
"x2:0, y2:1,\n"
"stop:1 #d7801a,\n"
"stop:0.4 #ffa02f\n"
");\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"}\n"
"\n"
"QMenu\n"
"{\n"
"border: 2px solid #dedede;\n"
"background-color:#000000;\n"
"font-size: 18px;\n"
"}\n"
"\n"
"QMenu::item\n"
"{\n"
"padding: 2px 20px 2px 20px;\n"
"background-color:#000000;\n"
"font-size: 18px;\n"
"}\n"
"\n"
"QMenu::item:selected\n"
"{\n"
"color: #000000;\n"
"}\n"
"\n"
"QTextEdit\n"
"{\n"
"background-color: #101010;\n"
"}\n"
"\n"
"\n"
"QTextEdit:focus\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2:"
                        " 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QPlainTextEdit\n"
"{\n"
"background-color: #101010;\n"
"}\n"
"\n"
"/* Customize push buttons and comboboxes. Our read-only combobox\n"
"   is very similar to a push button, so they share the same border image. */\n"
"\n"
"QPushButton {\n"
"    min-width: 4em;\n"
"}\n"
"\n"
"QPushButton, QComboBox[editable=\"false\"],\n"
"QComboBox[editable=\"true\"]::drop-down, QSpinBox, QAbstractSpinBox {\n"
"    border-image:  url(:/Widgets/Images/Widgets/pushbutton.png) 5;\n"
"    border-width: 5;\n"
"}\n"
"\n"
"QPushButton:hover, QComboBox[editable=\"false\"]:hover,\n"
"QComboBox[editable=\"true\"]::drop-down:hover, QMenuBar::item:hover, QSpinBox:hover, QAbstractSpinBox:hover {\n"
"    border-image: url(:/Widgets/Images/Widgets/pushbutton_hover.png) 5;\n"
"    border-width: 5;\n"
"}\n"
"\n"
"QPushButton:pressed, QComboBox[editable=\"false\"]:on,\n"
"QComboBox[editable=\"true\"]::drop-down:on, QMenuBar::item:on , QSpinBox:pressed, QAbstractSpinBox:pressed{\n"
"   "
                        " border-image: url(:/Widgets/Images/Widgets/pushbutton_pressed.png) 5;\n"
"    border-width: 5;\n"
"}\n"
"\n"
"/* Customize read-only comboboxes. */\n"
"\n"
"QComboBox[editable=\"false\"] {\n"
"    padding-left: 3px;\n"
"    padding-right: 20px; /* space for the arrow */\n"
"}\n"
"\n"
"QComboBox[editable=\"false\"]::drop-down {\n"
"    subcontrol-origin: padding;\n"
"    subcontrol-position: top right;\n"
"    width: 15px;\n"
"    border-left-style: solid;\n"
"    border-left-color: darkgray;\n"
"    border-left-width: 1px;\n"
"}\n"
"\n"
"QComboBox[editable=\"false\"]::down-arrow {\n"
"    subcontrol-origin: content;\n"
"    subcontrol-position: center;\n"
"    position: relative;\n"
"    left: 1px; /* 1 pixel dropdown border */\n"
"}\n"
"\n"
"/* The combobox arrow is on when the popup is open. */\n"
"QComboBox[editable=\"false\"]::down-arrow:on {\n"
"    position: relative;\n"
"    top: 1px;\n"
"    left: 2px;\n"
"}\n"
"\n"
"/* Customize editable comboboxes. */\n"
"\n"
"QComboBox[editable=\"true\"] {\n"
"    "
                        "padding-right: 16px;\n"
"}\n"
"\n"
"QComboBox[editable=\"true\"]::drop-down {\n"
"    subcontrol-origin: border;\n"
"    subcontrol-position: top right;\n"
"    width: 13px;\n"
"    position: absolute;\n"
"    top: 2px;\n"
"    bottom: 2px;\n"
"    right: 2px;\n"
"}\n"
"\n"
"QComboBox[editable=\"true\"]::drop-down,\n"
"QComboBox[editable=\"true\"]::drop-down:hover,\n"
"QComboBox[editable=\"true\"]::drop-down:on {\n"
"    border-width: 0px;  \n"
"    border-left-width: 3px; /* we need only left and center part */\n"
"}\n"
"\n"
"/* Shift the arrow when it's open. */\n"
"QComboBox[editable=\"true\"]::down-arrow:on {\n"
"    position: relative;\n"
"    top: 1px;\n"
"    left: 1px;\n"
"}\n"
"\n"
"QGroupBox:focus\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"QScrollBar:horizontal {\n"
"border: 1px solid #222222;\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0.0 #121212, stop: 0.2 #282828, stop: 1 #484848);\n"
"height: 7"
                        "px;\n"
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
"width: 1px;\n"
"height: 1px;\n"
"background: white;\n"
"}\n"
"\n"
"QScrollBar::add-page:horizonta"
                        "l, QScrollBar::sub-page:horizontal\n"
"{\n"
"background: none;\n"
"}\n"
"\n"
"QScrollBar:vertical\n"
"{\n"
"background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0.0 #121212, stop: 0.2 #282828, stop: 1 #484848);\n"
"width: 30px;\n"
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
"background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #d7801a, stop: 1 #ffa02f);\n"
"height: 14px;\n"
"subcontrol-"
                        "position: top;\n"
"subcontrol-origin: margin;\n"
"}\n"
"\n"
"\n"
"QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical\n"
"{\n"
"background: none;\n"
"}\n"
"\n"
"\n"
"QHeaderView::section\n"
"{\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #616161, stop: 0.5 #505050, stop: 0.6 #434343, stop:1 #656565);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #6c6c6c;\n"
"}\n"
"\n"
"QDockWidget::title\n"
"{\n"
"text-align: center;\n"
"spacing: 3px; /* spacing between items in the tool bar */\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #808080, stop: 0.5 #242424, stop:1 #808080);\n"
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
"background: #2"
                        "42424;\n"
"}\n"
"\n"
"QDockWidget::close-button:pressed, QDockWidget::float-button:pressed\n"
"{\n"
"padding: 1px -1px -1px 1px;\n"
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
"background: transparent;\n"
"color: #b1b1b1;\n"
"border: 2px solid #dedede;\n"
"border-radius: 5px;\n"
"border-bottom-style: none;\n"
"background-color: #000000;\n"
"padding-left: 10px;\n"
"padding-right: 10px;\n"
"padding-top: 3px;\n"
"padding-bottom: 2px;\n"
"margin-right: -1px;\n"
"}\n"
"\n"
"QTabWidget::pane {\n"
"background: transparent;\n"
"border: 2px solid "
                        "#dedede;\n"
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
"background: transparent;\n"
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
"background: transparent;\n"
"}\n"
"\n"
"QTabBar::tab:!selected\n"
"{\n"
"color: #404040;\n"
"border-bottom-style: none;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-style:solid;\n"
"margin-top: 3px;\n"
"border-top-color: #404040;\n"
"border-left-color: #404040;\n"
"border-right-color: #404040;\n"
"/*background-color:#121212;*/\n"
"}\n"
"\n"
"QTab"
                        "Bar::tab:selected\n"
"{\n"
"color: #dedede;\n"
"border-top-left-radius: 5px;\n"
"border-top-right-radius: 5px;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-color: #000000;\n"
"background-color: #000000;\n"
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
"QTabBar::focus\n"
"{\n"
"border:transparent;\n"
"}\n"
"\n"
"QRadioButton::indicator:unchecked{\n"
"image: url(:/Widgets/Images/Widgets/radiobutton_unchecked.png);\n"
"}\n"
"\n"
"QRadioButton::indicator:unchecked:hover {\n"
"    image: url(:/Widgets/Images/Widgets/radiobutton_uncheck"
                        "ed_hover.png);\n"
"}\n"
"\n"
"QRadioButton::indicator:unchecked:pressed {\n"
"    image: url(:/Widgets/Images/Widgets/radiobutton_unchecked_pressed.png);\n"
"}\n"
"\n"
"\n"
"QRadioButton::indicator:checked\n"
"{\n"
"    image: url(:/Widgets/Images/Widgets/radiobutton_checked.png);\n"
"\n"
"}\n"
"\n"
"QRadioButton::indicator:checked:hover {\n"
"    image: url(:/Widgets/Images/Widgets/radiobutton_checked_hover.png);\n"
"}\n"
"\n"
"QRadioButton::indicator:checked:pressed {\n"
"    image: url(:/Widgets/Images/Widgets/radiobutton_checked_pressed.png);\n"
"}\n"
"\n"
"\n"
"QRadioButton::indicator\n"
"{\n"
"background: transparent;\n"
"}\n"
"\n"
" QRadioButton::indicator:disabled\n"
"{\n"
"border: 1px solid #444;\n"
"}\n"
"\n"
"QCheckBox::indicator:disabled\n"
"{\n"
"border: 1px solid #444;\n"
"}\n"
"\n"
"QCheckBox:disabled\n"
"{\n"
"color: #111111;\n"
"}\n"
"\n"
"QCheckBox::indicator{\n"
"background:#000000;\n"
"border: 0px solid #b1b1b1;\n"
"width: 13px;\n"
"height:13px;\n"
"}\n"
"\n"
"\n"
"QCheckBox::indicator:chec"
                        "ked\n"
"{\n"
"    image: url(:/Widgets/Images/Widgets/checkbox_checked.png);\n"
"}\n"
"\n"
"QCheckBox::indicator:unchecked {\n"
"    image: url(:/Widgets/Images/Widgets/checkbox_unchecked.png);\n"
"}\n"
"\n"
"QCheckBox::indicator:unchecked:hover {\n"
"    image: url(:/Widgets/Images/Widgets/checkbox_unchecked_hover.png);\n"
"}\n"
"\n"
"QCheckBox::indicator:unchecked:pressed {\n"
"    image: url(:/Widgets/Images/Widgets/checkbox_unchecked_pressed.png);\n"
"}\n"
"\n"
"QCheckBox::indicator:checked:hover {\n"
"    image: url(:/Widgets/Images/Widgets/checkbox_checked_hover.png);\n"
"}\n"
"\n"
"QCheckBox::indicator:checked:pressed {\n"
"    image: url(:/Widgets/Images/Widgets/checkbox_checked_pressed.png);\n"
"}\n"
"\n"
"/* Customize spin boxes. */\n"
"\n"
"QSpinBox { \n"
"    padding-right: 5px;\n"
"}\n"
"\n"
"QSpinBox::up-button {\n"
"    subcontrol-origin: border;\n"
"    subcontrol-position: top right;\n"
"\n"
"    width: 16px; /* 16 + 2*1px border-width = 15px padding + 3px parent border */\n"
"    border-image"
                        ": url(:/Widgets/Images/Widgets/spinup.png) 1;\n"
"    border-width: 1px;\n"
"}\n"
"\n"
"QSpinBox::up-button:hover {\n"
"    border-image: url(:/Widgets/Images/Widgets/spinup_hover.png) 1;\n"
"}\n"
"\n"
"QSpinBox::up-button:pressed {\n"
"    border-image: url(:/Widgets/Images/Widgets/spinup_pressed.png) 1;\n"
"top:1;\n"
"}\n"
"\n"
"QSpinBox::down-button {\n"
"    subcontrol-origin: border;\n"
"    subcontrol-position: bottom right;\n"
"\n"
"    width: 16px;\n"
"    border-image: url(:/Widgets/Images/Widgets/spindown.png) 1;\n"
"    border-width: 1px;\n"
"    border-top-width: 0;\n"
"}\n"
"\n"
"QSpinBox::down-button:hover {\n"
"    border-image: url(:/Widgets/Images/Widgets/spindown_hover.png) 1;\n"
"}\n"
"\n"
"QSpinBox::down-button:pressed {\n"
"    border-image: url(:/Widgets/Images/Widgets/spindown_pressed.png) 1;\n"
"   top:1;\n"
"}\n"
"\n"
"/* Customize double spin boxes. */\n"
"\n"
"QAbstractSpinBox { \n"
"    padding-right: 5px;\n"
"\n"
"}\n"
"\n"
"QAbstractSpinBox::up-button {\n"
"    subcontrol-origin: "
                        "border;\n"
"    subcontrol-position: top right;\n"
"\n"
"    width: 16px; /* 16 + 2*1px border-width = 15px padding + 3px parent border */\n"
"    border-image: url(:/Widgets/Images/Widgets/spinup.png) 1;\n"
"    border-width: 1px;\n"
"}\n"
"\n"
"QAbstractSpinBox::up-button:hover {\n"
"    border-image: url(:/Widgets/Images/Widgets/spinup_hover.png) 1;\n"
"}\n"
"\n"
"QAbstractSpinBox::up-button:pressed {\n"
"    border-image: url(:/Widgets/Images/Widgets/spinup_pressed.png) 1;\n"
"top:1;\n"
"}\n"
"\n"
"QAbstractSpinBox::down-button {\n"
"    subcontrol-origin: border;\n"
"    subcontrol-position: bottom right;\n"
"\n"
"    width: 16px;\n"
"    border-image: url(:/Widgets/Images/Widgets/spindown.png) 1;\n"
"    border-width: 1px;\n"
"    border-top-width: 0;\n"
"}\n"
"\n"
"QAbstractSpinBox::down-button:hover {\n"
"    border-image: url(:/Widgets/Images/Widgets/spindown_hover.png) 1;\n"
"}\n"
"\n"
"QAbstractSpinBox::down-button:pressed {\n"
"    border-image: url(:/Widgets/Images/Widgets/spindown_pressed.png) 1;"
                        "\n"
"    top: 1;\n"
"}\n"
"\n"
"QTableView\n"
"{\n"
"background:transparent;\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);\n"
"}\n"
"\n"
"/* Customize arrows. */\n"
"\n"
"*::down-arrow, *::menu-indicator {\n"
"    image: url(:/Widgets/Images/Widgets/down_arrow.png);\n"
"    width: 7px;\n"
"    height: 7px;\n"
"}\n"
"\n"
"*::down-arrow:disabled, *::down-arrow:off {\n"
"   image: url(:/Widgets/Images/Widgets/down_arrow_disabled.png);\n"
"}\n"
"\n"
"*::up-arrow {\n"
"    image: url(:/Widgets/Images/Widgets/up_arrow.png);\n"
"    width: 7px;\n"
"    height: 7px;\n"
"}\n"
"\n"
"*::up-arrow:disabled, *::up-arrow:off {\n"
"   image: url(:/WidgetImages/Iimages/up_arrow_disabled.png);\n"
"}\n"
"\n"
"/* Scrollbars use dark arrows. */\n"
"\n"
"QScrollBar::down-arrow {\n"
"    image: url(:/Widgets/Images/Widgets/down_arrow_dark.png);\n"
"    width: 9px;\n"
"    height: 9px;\n"
"}\n"
"\n"
"QScrollBar::down-arrow:pressed {\n"
"    image: url(:/Widgets/Images/Widge"
                        "ts/down_arrow_dark.png);\n"
"    width: 7px;\n"
"    height: 7px;\n"
"    top:1;\n"
"}\n"
"\n"
"\n"
"QScrollBar::up-arrow {\n"
"    image: url(:/Widgets/Images/Widgets/up_arrow_dark.png);\n"
"    width: 9px;\n"
"    height: 9px;\n"
"}\n"
"\n"
"QScrollBar::up-arrow:pressed {\n"
"    image: url(:/Widgets/Images/Widgets/up_arrow_dark.png);\n"
"    width: 7px;\n"
"    height: 7px;\n"
"    top:1;\n"
"}\n"
"\n"
"QToolbar\n"
"{\n"
"spacing: 10px;\n"
"}\n"
"\n"
"QToolBar::handle\n"
"{\n"
"spacing: 10px; /* spacing between items in the tool bar */\n"
"background: url(:/images/handle.png);\n"
"}\n"
"\n"
"QToolButton\n"
"{\n"
"border-width: 2px;\n"
"border-color: rgb(42, 42, 42);\n"
"border-style: outset;\n"
"border-radius: 6;\n"
"padding: 3px;\n"
"font-size: 12px;\n"
"padding-left: 5px;\n"
"padding-right: 5px;\n"
"font-size: 12px;\n"
"background-color: qlineargradient(spread:pad, x1:1, y1:1, x2:0, y2:0.017, stop:0.753769 rgba(0, 0, 0, 255), stop:1 rgba(72, 72, 72, 255));\n"
"}\n"
"\n"
"QToolButton:disabled\n"
"{\n"
"/*c"
                        "olor: #dedede;*/\n"
"border-width:  1 px;\n"
"border-color: #494949;\n"
"border-style: solid;\n"
"border-radius: 6;\n"
"padding: 3px;\n"
"font-size: 12px;\n"
"padding-left: 5px;\n"
"padding-right: 5px;\n"
"font-size: 12px;\n"
"background-color: #000000;\n"
"}\n"
"\n"
"QToolButton:pressed\n"
"{\n"
"border-width: 2px;\n"
"border-color: rgb(188, 188, 188);\n"
"border-style: inset;\n"
"background-color: qlineargradient(spread:pad, x1:0.553, y1:0.494, x2:1, y2:1, stop:0 rgba(0, 0, 0, 255), stop:0.98995 rgba(72, 72, 72, 255), stop:1 rgba(48, 48, 48, 255))\n"
"}\n"
"\n"
"QToolButton:checked\n"
"{\n"
"border-width: 2px;\n"
"border-color: rgb(188, 188, 188);\n"
"border-style: inset;\n"
"background-color: qlineargradient(spread:pad, x1:0.553, y1:0.494, x2:1, y2:1, stop:0 rgba(0, 0, 0, 255), stop:0.98995 rgba(72, 72, 72, 255), stop:1 rgba(48, 48, 48, 255))\n"
"}\n"
"\n"
"QToolButton:unchecked\n"
"{\n"
"border-width:2px;\n"
"border-color: rgb(42, 42, 42);\n"
"border-style: outset;\n"
"background-color: qlineargradient(spr"
                        "ead:pad, x1:1, y1:1, x2:0, y2:0.017, stop:0.753769 rgba(0, 0, 0, 255), stop:1 rgba(72, 72, 72, 255));\n"
"}\n"
"\n"
"\n"
"QToolButton:hover\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}"));
        actionQuitter = new QAction(AWLQtDemoClass);
        actionQuitter->setObjectName(QStringLiteral("actionQuitter"));
        actionQuitter->setIconVisibleInMenu(false);
        actionGraph = new QAction(AWLQtDemoClass);
        actionGraph->setObjectName(QStringLiteral("actionGraph"));
        actionGraph->setCheckable(true);
        action2D = new QAction(AWLQtDemoClass);
        action2D->setObjectName(QStringLiteral("action2D"));
        action2D->setCheckable(true);
        actionCamera = new QAction(AWLQtDemoClass);
        actionCamera->setObjectName(QStringLiteral("actionCamera"));
        actionCamera->setCheckable(true);
        actionTableView = new QAction(AWLQtDemoClass);
        actionTableView->setObjectName(QStringLiteral("actionTableView"));
        actionTableView->setCheckable(true);
        actionTableView->setChecked(false);
        actionSettings = new QAction(AWLQtDemoClass);
        actionSettings->setObjectName(QStringLiteral("actionSettings"));
        actionSettings->setCheckable(true);
        actionSettings->setEnabled(true);
        actionSettings->setIconVisibleInMenu(false);
        centralWidget = new QWidget(AWLQtDemoClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout->setContentsMargins(0, 0, -1, 0);
        interfaceTabs = new QTabWidget(centralWidget);
        interfaceTabs->setObjectName(QStringLiteral("interfaceTabs"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(interfaceTabs->sizePolicy().hasHeightForWidth());
        interfaceTabs->setSizePolicy(sizePolicy1);
        interfaceTabs->setMinimumSize(QSize(200, 253));
        interfaceTabs->setSizeIncrement(QSize(0, 200));
        interfaceTabs->setAutoFillBackground(false);
        interfaceTabs->setStyleSheet(QStringLiteral(""));
        interfaceTabs->setTabPosition(QTabWidget::North);
        interfaceTabs->setTabShape(QTabWidget::Rounded);
        interfaceTabs->setUsesScrollButtons(true);
        interfaceTabs->setDocumentMode(false);
        interfaceTabs->setMovable(true);
        calibrationTab = new QWidget();
        calibrationTab->setObjectName(QStringLiteral("calibrationTab"));
        internalCalibrationGroupBox = new QGroupBox(calibrationTab);
        internalCalibrationGroupBox->setObjectName(QStringLiteral("internalCalibrationGroupBox"));
        internalCalibrationGroupBox->setGeometry(QRect(550, 10, 201, 201));
        internalCalibrationGroupBox->setStyleSheet(QStringLiteral(""));
        calibrationChannelMaskGroupBox = new QGroupBox(internalCalibrationGroupBox);
        calibrationChannelMaskGroupBox->setObjectName(QStringLiteral("calibrationChannelMaskGroupBox"));
        calibrationChannelMaskGroupBox->setGeometry(QRect(10, 80, 171, 61));
        calibrationChannelMaskGroupBox->setStyleSheet(QStringLiteral(""));
        calibrationChannelMaskGroupBox->setFlat(true);
        calibrationChannelMaskGroupBox->setCheckable(false);
        calibrationChannel7CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel7CheckBox->setObjectName(QStringLiteral("calibrationChannel7CheckBox"));
        calibrationChannel7CheckBox->setGeometry(QRect(115, 20, 31, 17));
        calibrationChannel7CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationChannel6CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel6CheckBox->setObjectName(QStringLiteral("calibrationChannel6CheckBox"));
        calibrationChannel6CheckBox->setGeometry(QRect(75, 20, 31, 17));
        calibrationChannel6CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationChannel5CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel5CheckBox->setObjectName(QStringLiteral("calibrationChannel5CheckBox"));
        calibrationChannel5CheckBox->setGeometry(QRect(35, 20, 33, 17));
        calibrationChannel5CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationChannel5CheckBox->setChecked(true);
        calibrationChannel2CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel2CheckBox->setObjectName(QStringLiteral("calibrationChannel2CheckBox"));
        calibrationChannel2CheckBox->setGeometry(QRect(54, 40, 33, 17));
        calibrationChannel2CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationChannel3CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel3CheckBox->setObjectName(QStringLiteral("calibrationChannel3CheckBox"));
        calibrationChannel3CheckBox->setGeometry(QRect(96, 40, 33, 17));
        calibrationChannel3CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationChannel1CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel1CheckBox->setObjectName(QStringLiteral("calibrationChannel1CheckBox"));
        calibrationChannel1CheckBox->setGeometry(QRect(13, 40, 33, 17));
        calibrationChannel1CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationChannel1CheckBox->setChecked(true);
        calibrationChannel4CheckBox = new QCheckBox(calibrationChannelMaskGroupBox);
        calibrationChannel4CheckBox->setObjectName(QStringLiteral("calibrationChannel4CheckBox"));
        calibrationChannel4CheckBox->setGeometry(QRect(130, 40, 35, 17));
        calibrationChannel4CheckBox->setStyleSheet(QStringLiteral(""));
        calibrationBetaLabel = new QLabel(internalCalibrationGroupBox);
        calibrationBetaLabel->setObjectName(QStringLiteral("calibrationBetaLabel"));
        calibrationBetaLabel->setGeometry(QRect(20, 53, 31, 16));
        calibrationBetaLabel->setStyleSheet(QStringLiteral(""));
        calibrationBetaLabel->setFrameShape(QFrame::NoFrame);
        calibrationBetaDoubleSpinBox = new QDoubleSpinBox(internalCalibrationGroupBox);
        calibrationBetaDoubleSpinBox->setObjectName(QStringLiteral("calibrationBetaDoubleSpinBox"));
        calibrationBetaDoubleSpinBox->setGeometry(QRect(86, 50, 71, 22));
        calibrationBetaDoubleSpinBox->setStyleSheet(QStringLiteral(""));
        calibrationBetaDoubleSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        calibrationBetaDoubleSpinBox->setDecimals(1);
        calibrationBetaDoubleSpinBox->setMaximum(1);
        calibrationBetaDoubleSpinBox->setSingleStep(0.1);
        calibrationFrameQtySpinBox = new QSpinBox(internalCalibrationGroupBox);
        calibrationFrameQtySpinBox->setObjectName(QStringLiteral("calibrationFrameQtySpinBox"));
        calibrationFrameQtySpinBox->setGeometry(QRect(87, 20, 71, 22));
        calibrationFrameQtySpinBox->setStyleSheet(QStringLiteral(""));
        calibrationFrameQtySpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        calibrationFrameQtySpinBox->setMinimum(1);
        calibrationFrameQtySpinBox->setMaximum(255);
        calibrationFrameQtySpinBox->setValue(100);
        calibrationFrameQtyLabel = new QLabel(internalCalibrationGroupBox);
        calibrationFrameQtyLabel->setObjectName(QStringLiteral("calibrationFrameQtyLabel"));
        calibrationFrameQtyLabel->setGeometry(QRect(20, 23, 61, 16));
        calibrationFrameQtyLabel->setStyleSheet(QStringLiteral(""));
        calibrationFrameQtyLabel->setFrameShape(QFrame::NoFrame);
        calibrateButton = new QPushButton(internalCalibrationGroupBox);
        calibrateButton->setObjectName(QStringLiteral("calibrateButton"));
        calibrateButton->setGeometry(QRect(40, 170, 111, 20));
        calibrateButton->setStyleSheet(QStringLiteral(""));
        calibrateButton->setCheckable(true);
        calibrateButton->setChecked(false);
        calibrateButton->setAutoDefault(false);
        calibrateButton->setFlat(false);
        externalCalibrationGroupBox = new QGroupBox(calibrationTab);
        externalCalibrationGroupBox->setObjectName(QStringLiteral("externalCalibrationGroupBox"));
        externalCalibrationGroupBox->setGeometry(QRect(10, 10, 521, 201));
        externalCalibrationGroupBox->setStyleSheet(QStringLiteral(""));
        sensorPositionGroupBox = new QGroupBox(externalCalibrationGroupBox);
        sensorPositionGroupBox->setObjectName(QStringLiteral("sensorPositionGroupBox"));
        sensorPositionGroupBox->setGeometry(QRect(10, 20, 501, 51));
        sensorPositionGroupBox->setStyleSheet(QStringLiteral(""));
        sensorPositionGroupBox->setFlat(true);
        sensorHeightSpinBox = new QDoubleSpinBox(sensorPositionGroupBox);
        sensorHeightSpinBox->setObjectName(QStringLiteral("sensorHeightSpinBox"));
        sensorHeightSpinBox->setGeometry(QRect(90, 20, 71, 22));
        sensorHeightSpinBox->setStyleSheet(QStringLiteral(""));
        sensorHeightSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensorHeightSpinBox->setMinimum(0);
        sensorHeightSpinBox->setMaximum(10);
        sensorHeightSpinBox->setSingleStep(0.1);
        sensorHeightSpinBox->setValue(1.38);
        sensorHeightLabel = new QLabel(sensorPositionGroupBox);
        sensorHeightLabel->setObjectName(QStringLiteral("sensorHeightLabel"));
        sensorHeightLabel->setGeometry(QRect(10, 21, 71, 20));
        sensorHeightLabel->setStyleSheet(QStringLiteral(""));
        sensorDepthSpinBox = new QDoubleSpinBox(sensorPositionGroupBox);
        sensorDepthSpinBox->setObjectName(QStringLiteral("sensorDepthSpinBox"));
        sensorDepthSpinBox->setGeometry(QRect(262, 19, 81, 22));
        sensorDepthSpinBox->setStyleSheet(QStringLiteral(""));
        sensorDepthSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensorDepthSpinBox->setMinimum(-100);
        sensorDepthSpinBox->setMaximum(10);
        sensorDepthSpinBox->setSingleStep(0.1);
        sensorDepthSpinBox->setValue(-1);
        sensorDepthLabel = new QLabel(sensorPositionGroupBox);
        sensorDepthLabel->setObjectName(QStringLiteral("sensorDepthLabel"));
        sensorDepthLabel->setGeometry(QRect(180, 20, 81, 20));
        sensorDepthLabel->setStyleSheet(QStringLiteral(""));
        receiverCalibrationGroupBox = new QGroupBox(externalCalibrationGroupBox);
        receiverCalibrationGroupBox->setObjectName(QStringLiteral("receiverCalibrationGroupBox"));
        receiverCalibrationGroupBox->setGeometry(QRect(10, 150, 301, 51));
        receiverCalibrationGroupBox->setStyleSheet(QStringLiteral(""));
        receiverCalibrationGroupBox->setFlat(true);
        measurementOffsetLabel = new QLabel(receiverCalibrationGroupBox);
        measurementOffsetLabel->setObjectName(QStringLiteral("measurementOffsetLabel"));
        measurementOffsetLabel->setGeometry(QRect(10, 23, 71, 16));
        measurementOffsetLabel->setStyleSheet(QStringLiteral(""));
        measurementOffsetSpinBox = new QDoubleSpinBox(receiverCalibrationGroupBox);
        measurementOffsetSpinBox->setObjectName(QStringLiteral("measurementOffsetSpinBox"));
        measurementOffsetSpinBox->setGeometry(QRect(90, 20, 71, 22));
        measurementOffsetSpinBox->setStyleSheet(QStringLiteral(""));
        measurementOffsetSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        measurementOffsetSpinBox->setDecimals(1);
        measurementOffsetSpinBox->setMinimum(-40);
        measurementOffsetSpinBox->setMaximum(40);
        measurementOffsetSpinBox->setSingleStep(0.1);
        sensorRangeGroupBox = new QGroupBox(externalCalibrationGroupBox);
        sensorRangeGroupBox->setObjectName(QStringLiteral("sensorRangeGroupBox"));
        sensorRangeGroupBox->setGeometry(QRect(10, 80, 501, 51));
        sensorRangeGroupBox->setStyleSheet(QStringLiteral(""));
        sensorRangeGroupBox->setFlat(true);
        sensorRangeMinLabel = new QLabel(sensorRangeGroupBox);
        sensorRangeMinLabel->setObjectName(QStringLiteral("sensorRangeMinLabel"));
        sensorRangeMinLabel->setGeometry(QRect(10, 23, 61, 16));
        sensorRangeMinLabel->setStyleSheet(QStringLiteral(""));
        sensorRangeMaxLabel = new QLabel(sensorRangeGroupBox);
        sensorRangeMaxLabel->setObjectName(QStringLiteral("sensorRangeMaxLabel"));
        sensorRangeMaxLabel->setGeometry(QRect(180, 23, 61, 16));
        sensorRangeMaxLabel->setStyleSheet(QStringLiteral(""));
        sensorRangeMinSpinBox = new QDoubleSpinBox(sensorRangeGroupBox);
        sensorRangeMinSpinBox->setObjectName(QStringLiteral("sensorRangeMinSpinBox"));
        sensorRangeMinSpinBox->setGeometry(QRect(90, 20, 71, 22));
        sensorRangeMinSpinBox->setStyleSheet(QStringLiteral(""));
        sensorRangeMinSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensorRangeMinSpinBox->setDecimals(1);
        sensorRangeMinSpinBox->setMinimum(-1000);
        sensorRangeMinSpinBox->setMaximum(1000);
        sensorRangeMinSpinBox->setSingleStep(0.1);
        sensorRangeMaxSpinBox = new QDoubleSpinBox(sensorRangeGroupBox);
        sensorRangeMaxSpinBox->setObjectName(QStringLiteral("sensorRangeMaxSpinBox"));
        sensorRangeMaxSpinBox->setGeometry(QRect(262, 20, 71, 22));
        sensorRangeMaxSpinBox->setStyleSheet(QStringLiteral(""));
        sensorRangeMaxSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensorRangeMaxSpinBox->setDecimals(1);
        sensorRangeMaxSpinBox->setMaximum(1000);
        sensorRangeMaxSpinBox->setSingleStep(0.1);
        sensorRangeMaxSpinBox->setValue(30);
        channelSelectListWidget = new QListWidget(externalCalibrationGroupBox);
        channelSelectListWidget->setObjectName(QStringLiteral("channelSelectListWidget"));
        channelSelectListWidget->setGeometry(QRect(353, 100, 151, 91));
        channelSelectListWidget->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
        targetHintGroupBox = new QGroupBox(calibrationTab);
        targetHintGroupBox->setObjectName(QStringLiteral("targetHintGroupBox"));
        targetHintGroupBox->setEnabled(true);
        targetHintGroupBox->setGeometry(QRect(770, 10, 161, 111));
        targetHintGroupBox->setStyleSheet(QStringLiteral(""));
        targetHintAngleLabel = new QLabel(targetHintGroupBox);
        targetHintAngleLabel->setObjectName(QStringLiteral("targetHintAngleLabel"));
        targetHintAngleLabel->setGeometry(QRect(10, 53, 31, 16));
        targetHintAngleLabel->setStyleSheet(QStringLiteral(""));
        targetHintAngleLabel->setFrameShape(QFrame::NoFrame);
        targetHintAngleSpinBox = new QDoubleSpinBox(targetHintGroupBox);
        targetHintAngleSpinBox->setObjectName(QStringLiteral("targetHintAngleSpinBox"));
        targetHintAngleSpinBox->setGeometry(QRect(70, 50, 81, 22));
        targetHintAngleSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        targetHintAngleSpinBox->setDecimals(1);
        targetHintAngleSpinBox->setMinimum(-90);
        targetHintAngleSpinBox->setMaximum(90);
        targetHintAngleSpinBox->setSingleStep(0.1);
        targetHintDistanceLabel = new QLabel(targetHintGroupBox);
        targetHintDistanceLabel->setObjectName(QStringLiteral("targetHintDistanceLabel"));
        targetHintDistanceLabel->setGeometry(QRect(10, 23, 61, 16));
        targetHintDistanceLabel->setStyleSheet(QStringLiteral(""));
        targetHintDistanceLabel->setFrameShape(QFrame::NoFrame);
        targetHintDistanceSpinBox = new QDoubleSpinBox(targetHintGroupBox);
        targetHintDistanceSpinBox->setObjectName(QStringLiteral("targetHintDistanceSpinBox"));
        targetHintDistanceSpinBox->setGeometry(QRect(70, 20, 81, 22));
        targetHintDistanceSpinBox->setStyleSheet(QLatin1String("QToolTip\n"
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
"background-color: #000000;\n"
"font-size: 11px;\n"
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
"QWidget:disabled\n"
"{\n"
"/*color: #404040;\n"
"background-color: #202020;*/\n"
"color: #404040;\n"
"border: 1px solid #404040;\n"
"}\n"
"\n"
"QWidget:focus\n"
"{\n"
"border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"/*QAbstractItemView\n"
"{\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0.1 #646464, stop: 1 #5d5d5d);\n"
"}*/\n"
"\n"
"QMenuBar::item\n"
"{\n"
""
                        "background: transparent;\n"
"border: 1px solid #000000;\n"
"font-size: 12px;\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"}\n"
"\n"
"QMenuBar::item:selected\n"
"{\n"
"background: transparent;\n"
"border: 1px solid #ffaa00;\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"}\n"
"\n"
"QMenuBar::item:pressed\n"
"{\n"
"background: #444;\n"
"border: 1px solid #000000;\n"
"background-color: QLinearGradient(\n"
"x1:0, y1:0,\n"
"x2:0, y2:1,\n"
"stop:1 #d7801a,\n"
"stop:0.4 #ffa02f\n"
");\n"
"margin-bottom:-1px;\n"
"padding-bottom:1px;\n"
"}\n"
"\n"
"QMenu\n"
"{\n"
"border: 2px solid #dedede;\n"
"background-color:#000000\n"
"font-size: 12px;\n"
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
"QMenu::item\n"
"{\n"
"padding: 2px 20px 2px 20px;\n"
"background-color:#000000\n"
"fo"
                        "nt-size: 12px;\n"
"}\n"
"\n"
"QMenu::item:selected\n"
"{\n"
"color: #000000;\n"
"}\n"
"\n"
"QLineEdit\n"
"{\n"
"/*background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0 #646464, stop: 1 #5d5d5d);*/\n"
"padding: 1px;\n"
"border-style: solid;\n"
"border: 1px solid #bebebe;\n"
"border-radius: 5;\n"
"}\n"
"\n"
"QPushButton\n"
"{\n"
"/*color: #dedede;*/\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #1a1a1a, stop: 0.1 #101010, stop: 0.5 #0e0e0e, stop: 0.9 #0a0a0a, stop: 1 #000000);\n"
"border-width: 1px;\n"
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
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0.1 #4b4b4b, stop: 0.5 #494949, stop: 0.9 #484848, stop: 1 #454545);\n"
"}\n"
"\n"
"QComboBox\n"
"{\n"
"selection-background-color: #ffaa00;\n"
"background-color: QLine"
                        "arGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #1a1a1a, stop: 0.1 #101010, stop: 0.5 #0e0e0e, stop: 0.9 #0a0a0a, stop: 1 #000000);\n"
"border-style: solid;\n"
"border: 1px solid #bebebe;\n"
"border-radius: 5;\n"
"}\n"
"\n"
"QComboBox:hover,QPushButton:hover\n"
"{\n"
"border: 1px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
"}\n"
"\n"
"\n"
"QComboBox:on\n"
"{\n"
"padding-top: 3px;\n"
"padding-left: 4px;\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #0d0d0d, stop: 0.1 #0b0b0b, stop: 0.5 #090909, stop: 0.9 #080808, stop: 1 #050505);\n"
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
"border-left-color"
                        ": darkgray;\n"
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
"background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0.0 #121212, stop: 0.2 #282828, stop: 1 #484848);\n"
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
"background: QLin"
                        "earGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 1 #d7801a);\n"
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
"width: 1px;\n"
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
"background: QLinearGr"
                        "adient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 0.5 #d7801a, stop: 1 #ffa02f);\n"
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
"background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #d7801a, stop: 1 #ffa02f);\n"
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
"background-"
                        "color: #101010;\n"
"}\n"
"\n"
"QPlainTextEdit\n"
"{\n"
"background-color: #101010;\n"
"}\n"
"\n"
"QHeaderView::section\n"
"{\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #616161, stop: 0.5 #505050, stop: 0.6 #434343, stop:1 #656565);\n"
"color: white;\n"
"padding-left: 4px;\n"
"border: 1px solid #6c6c6c;\n"
"}\n"
"QDockWidget::title\n"
"{\n"
"text-align: center;\n"
"spacing: 3px; /* spacing between items in the tool bar */\n"
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #808080, stop: 0.5 #242424, stop:1 #808080);\n"
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
"QDockWidget::close-button:pressed, QDockWidget::float-button"
                        ":pressed\n"
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
"background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d7801a, stop:0.5 #b56c17 stop:1 #ffa02f);\n"
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
""
                        "}\n"
"\n"
"QTabBar::tab {\n"
"color: #b1b1b1;\n"
"border: 2px solid #dedede;\n"
"border-radius: 5px;\n"
"border-bottom-style: none;\n"
"background-color: #000000;\n"
"padding-left: 10px;\n"
"padding-right: 10px;\n"
"padding-top: 3px;\n"
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
"color: #404040;\n"
"border-bottom-style:"
                        " none;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-style:solid;\n"
"margin-top: 3px;\n"
"background-color: #000000;\n"
"}\n"
"\n"
"QTabBar::tab:selected\n"
"{\n"
"color: #dedede;\n"
"border-top-left-radius: 5px;\n"
"border-top-right-radius: 5px;\n"
"border-bottom-left-radius: 1px;\n"
"border-bottom-right-radius: 1px;\n"
"border-bottom-color: #000000;\n"
"background-color: #000000;\n"
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
"/*ba"
                        "ckground-color: #808080*/;\n"
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
"stop: 0.25 #ffaa00,\n"
"stop: 0.3 #000000\n"
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
"/*background-color: #808080;*/\n"
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
"color: #111111;\n"
"}\n"
"\n"
"QCheckBox::indicator{\n"
"color: #b1b1b1;\n"
"/*background-color: #808080;*/\n"
"border: 1px solid #b1b1b1;\n"
"width: 9px;\n"
"height: 9px;\n"
"}\n"
"\n"
""
                        "QCheckBox::indicator:checked, QCheckBox::indicator:unchecked{\n"
"color: #b1b1b1;\n"
"/*background-color: #808080;*/\n"
"border: 1px solid #b1b1b1;\n"
"border-radius: 6px;\n"
"}\n"
"\n"
"QCheckBox::indicator:checked\n"
"{\n"
"background-color: qradialgradient(\n"
"cx: 0.5, cy: 0.5,\n"
"fx: 0.5, fy: 0.5,\n"
"radius: 1.0,\n"
"stop: 0.25 #ffaa00,\n"
"stop: 0.3 #000000\n"
");\n"
"}\n"
"\n"
"QAbstractSpinBox {\n"
"color: #dedede;\n"
"background-color: #000000;\n"
"border: 1px solid #b1b1b1;\n"
"}\n"
"\n"
"QAbstractSpinBox::up-button {\n"
"color: #dedede;\n"
"background-color: #000000;\n"
"border: 1px solid #b1b1b1;\n"
"}\n"
"\n"
"QAbstractSpinBox::up-button:hover {\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #ca0619);\n"
"color: #000000;\n"
"}\n"
"\n"
"QAbstractSpinBox::up-button:pressed {\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0.1 #4b4b4b, stop: 0.5 #494949, stop: 0.9 #484848, stop: 1 #454545);\n"
"}\n"
"\n"
"QAbstra"
                        "ctSpinBox::up-arrow {\n"
"color: #dedede;\n"
"background-color: #000000;\n"
"\n"
"}\n"
"\n"
"QAbstractSpinBox::up-arrow:disabled, QSpinBox::up-arrow:off { /* off state when value is max */\n"
"color: #404040;\n"
"background-color: #202020;\n"
"border: 1px solid #b1b1b1;\n"
"}\n"
"\n"
"QAbstractSpinBox::down-button {\n"
"color: #dedede;\n"
"background-color: #000000;\n"
"border: 1px solid #b1b1b1;\n"
"}\n"
"\n"
"QAbstractSpinBox::down-button:hover {\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #ca0619);\n"
"color: #000000;\n"
" }\n"
"\n"
"QAbstractSpinBox::down-button:pressed {\n"
"background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #4d4d4d, stop: 0.1 #4b4b4b, stop: 0.5 #494949, stop: 0.9 #484848, stop: 1 #454545);\n"
"}\n"
"\n"
"QAbstractSpinBox::down-arrow {\n"
"color: #dedede;\n"
"background-color: #000000;\n"
"\n"
"}\n"
"\n"
"QAbstractSpinBox::down-arrow:disabled,\n"
"QAbstractSpinBox::down-arrow:off { /* off state when value in min */\n"
"co"
                        "lor: #404040;\n"
"background-color: #202020;\n"
"}\n"
""));
        targetHintDistanceSpinBox->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        targetHintDistanceSpinBox->setMinimum(0);
        targetHintDistanceSpinBox->setMaximum(100);
        targetHintDistanceSpinBox->setSingleStep(0.1);
        targetHintDistanceSpinBox->setValue(5);
        distanceLogFileCheckbox = new QCheckBox(targetHintGroupBox);
        distanceLogFileCheckbox->setObjectName(QStringLiteral("distanceLogFileCheckbox"));
        distanceLogFileCheckbox->setGeometry(QRect(10, 80, 141, 17));
        distanceLogFileCheckbox->setStyleSheet(QStringLiteral(""));
        distanceLogFileCheckbox->setCheckable(true);
        distanceLogFileCheckbox->setChecked(false);
        distanceLogFileCheckbox->setAutoRepeat(false);
        distanceLogFileCheckbox->setTristate(false);
        interfaceTabs->addTab(calibrationTab, QString());
        controlTab = new QWidget();
        controlTab->setObjectName(QStringLiteral("controlTab"));
        recordPlayGroupBox = new QGroupBox(controlTab);
        recordPlayGroupBox->setObjectName(QStringLiteral("recordPlayGroupBox"));
        recordPlayGroupBox->setGeometry(QRect(10, 10, 481, 211));
        recordButton = new QPushButton(recordPlayGroupBox);
        recordButton->setObjectName(QStringLiteral("recordButton"));
        recordButton->setGeometry(QRect(30, 184, 75, 20));
        recordButton->setCheckable(true);
        recordButton->setChecked(false);
        recordButton->setAutoDefault(false);
        recordButton->setFlat(false);
        playbackButton = new QPushButton(recordPlayGroupBox);
        playbackButton->setObjectName(QStringLiteral("playbackButton"));
        playbackButton->setGeometry(QRect(110, 184, 75, 20));
        playbackButton->setCheckable(true);
        playbackButton->setChecked(false);
        playbackButton->setAutoDefault(false);
        playbackButton->setFlat(false);
        channelMaskGroupBox = new QGroupBox(recordPlayGroupBox);
        channelMaskGroupBox->setObjectName(QStringLiteral("channelMaskGroupBox"));
        channelMaskGroupBox->setGeometry(QRect(10, 47, 461, 60));
        channelMaskGroupBox->setFlat(true);
        channelMaskGroupBox->setCheckable(false);
        recordChannel7CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel7CheckBox->setObjectName(QStringLiteral("recordChannel7CheckBox"));
        recordChannel7CheckBox->setGeometry(QRect(190, 20, 31, 17));
        recordChannel6CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel6CheckBox->setObjectName(QStringLiteral("recordChannel6CheckBox"));
        recordChannel6CheckBox->setGeometry(QRect(150, 20, 31, 17));
        recordChannel5CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel5CheckBox->setObjectName(QStringLiteral("recordChannel5CheckBox"));
        recordChannel5CheckBox->setGeometry(QRect(110, 20, 31, 17));
        recordChannel5CheckBox->setCheckable(true);
        recordChannel5CheckBox->setChecked(false);
        recordChannel2CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel2CheckBox->setObjectName(QStringLiteral("recordChannel2CheckBox"));
        recordChannel2CheckBox->setGeometry(QRect(130, 40, 31, 17));
        recordChannel3CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel3CheckBox->setObjectName(QStringLiteral("recordChannel3CheckBox"));
        recordChannel3CheckBox->setGeometry(QRect(170, 40, 31, 17));
        recordChannel1CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel1CheckBox->setObjectName(QStringLiteral("recordChannel1CheckBox"));
        recordChannel1CheckBox->setGeometry(QRect(90, 40, 31, 17));
        recordChannel1CheckBox->setChecked(false);
        recordChannel4CheckBox = new QCheckBox(channelMaskGroupBox);
        recordChannel4CheckBox->setObjectName(QStringLiteral("recordChannel4CheckBox"));
        recordChannel4CheckBox->setGeometry(QRect(200, 40, 31, 17));
        frameRateLabel = new QLabel(recordPlayGroupBox);
        frameRateLabel->setObjectName(QStringLiteral("frameRateLabel"));
        frameRateLabel->setGeometry(QRect(30, 20, 61, 16));
        frameRateLabel->setFrameShape(QFrame::NoFrame);
        frameRateSpinBox = new QSpinBox(recordPlayGroupBox);
        frameRateSpinBox->setObjectName(QStringLiteral("frameRateSpinBox"));
        frameRateSpinBox->setGeometry(QRect(100, 20, 71, 22));
        frameRateSpinBox->setMinimum(0);
        frameRateSpinBox->setMaximum(100);
        frameRateSpinBox->setValue(100);
        stopButton = new QPushButton(recordPlayGroupBox);
        stopButton->setObjectName(QStringLiteral("stopButton"));
        stopButton->setEnabled(false);
        stopButton->setGeometry(QRect(260, 184, 75, 20));
        stopButton->setCheckable(true);
        stopButton->setChecked(false);
        stopButton->setAutoDefault(false);
        stopButton->setFlat(false);
        recordFileNameGroupBox = new QGroupBox(recordPlayGroupBox);
        recordFileNameGroupBox->setObjectName(QStringLiteral("recordFileNameGroupBox"));
        recordFileNameGroupBox->setGeometry(QRect(10, 105, 461, 71));
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
        registersTab = new QWidget();
        registersTab->setObjectName(QStringLiteral("registersTab"));
        registerFPGAGroupBox = new QGroupBox(registersTab);
        registerFPGAGroupBox->setObjectName(QStringLiteral("registerFPGAGroupBox"));
        registerFPGAGroupBox->setGeometry(QRect(10, 10, 431, 141));
        registerFPGASetPushButton = new QPushButton(registerFPGAGroupBox);
        registerFPGASetPushButton->setObjectName(QStringLiteral("registerFPGASetPushButton"));
        registerFPGASetPushButton->setGeometry(QRect(50, 110, 75, 23));
        registerFPGASetPushButton->setFont(font);
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
        registerADCGroupBox = new QGroupBox(registersTab);
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
        interfaceTabs->addTab(registersTab, QString());
        gpiosTab = new QWidget();
        gpiosTab->setObjectName(QStringLiteral("gpiosTab"));
        groupBox = new QGroupBox(gpiosTab);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(20, 10, 311, 211));
        registerGPIOListWidget = new QListWidget(groupBox);
        registerGPIOListWidget->setObjectName(QStringLiteral("registerGPIOListWidget"));
        registerGPIOListWidget->setGeometry(QRect(20, 30, 261, 121));
        registerGPIOListWidget->setMidLineWidth(-1);
        registerGPIOListWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        registerGPIOListWidget->setProperty("showDropIndicator", QVariant(true));
        registerGPIOListWidget->setVerticalScrollMode(QAbstractItemView::ScrollPerItem);
        registerGPIOListWidget->setFlow(QListView::TopToBottom);
        registerGPIOListWidget->setSpacing(0);
        registerGPIOListWidget->setViewMode(QListView::ListMode);
        registerGPIOGetPushButton = new QPushButton(groupBox);
        registerGPIOGetPushButton->setObjectName(QStringLiteral("registerGPIOGetPushButton"));
        registerGPIOGetPushButton->setGeometry(QRect(170, 170, 75, 23));
        registerGPIOSetPushButton = new QPushButton(groupBox);
        registerGPIOSetPushButton->setObjectName(QStringLiteral("registerGPIOSetPushButton"));
        registerGPIOSetPushButton->setGeometry(QRect(60, 170, 75, 23));
        interfaceTabs->addTab(gpiosTab, QString());
        AlgoTab = new QWidget();
        AlgoTab->setObjectName(QStringLiteral("AlgoTab"));
        algoGroupBox = new QGroupBox(AlgoTab);
        algoGroupBox->setObjectName(QStringLiteral("algoGroupBox"));
        algoGroupBox->setGeometry(QRect(20, 10, 541, 211));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(1);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(algoGroupBox->sizePolicy().hasHeightForWidth());
        algoGroupBox->setSizePolicy(sizePolicy2);
        algoGroupBox->setMinimumSize(QSize(1, 0));
        algoSelectGroupBox = new QGroupBox(algoGroupBox);
        algoSelectGroupBox->setObjectName(QStringLiteral("algoSelectGroupBox"));
        algoSelectGroupBox->setGeometry(QRect(20, 20, 121, 171));
        algoSelectComboBox = new QComboBox(algoSelectGroupBox);
        algoSelectComboBox->setObjectName(QStringLiteral("algoSelectComboBox"));
        algoSelectComboBox->setGeometry(QRect(10, 30, 101, 22));
        algoSelectComboBox->setStyleSheet(QStringLiteral(""));
        algoParametersTable = new QTableWidget(algoGroupBox);
        if (algoParametersTable->columnCount() < 4)
            algoParametersTable->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        algoParametersTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        algoParametersTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        algoParametersTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        algoParametersTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        if (algoParametersTable->rowCount() < 8)
            algoParametersTable->setRowCount(8);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        __qtablewidgetitem4->setCheckState(Qt::Checked);
        __qtablewidgetitem4->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable|Qt::ItemIsUserCheckable|Qt::ItemIsEnabled);
        algoParametersTable->setItem(0, 0, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        __qtablewidgetitem5->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable|Qt::ItemIsEnabled);
        algoParametersTable->setItem(0, 1, __qtablewidgetitem5);
        algoParametersTable->setObjectName(QStringLiteral("algoParametersTable"));
        algoParametersTable->setGeometry(QRect(160, 30, 361, 131));
        algoParametersTable->setFont(font);
        algoParametersTable->setAutoFillBackground(true);
        algoParametersTable->setStyleSheet(QLatin1String("background-color: rgb(0,0,0);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        algoParametersTable->setFrameShape(QFrame::Panel);
        algoParametersTable->setFrameShadow(QFrame::Plain);
        algoParametersTable->setLineWidth(2);
        algoParametersTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        algoParametersTable->setGridStyle(Qt::SolidLine);
        algoParametersTable->setWordWrap(false);
        algoParametersTable->setCornerButtonEnabled(false);
        algoParametersTable->setRowCount(8);
        algoParametersTable->setColumnCount(4);
        algoParametersTable->horizontalHeader()->setVisible(false);
        algoParametersTable->horizontalHeader()->setCascadingSectionResizes(false);
        algoParametersTable->horizontalHeader()->setDefaultSectionSize(45);
        algoParametersTable->horizontalHeader()->setHighlightSections(true);
        algoParametersTable->horizontalHeader()->setMinimumSectionSize(10);
        algoParametersTable->horizontalHeader()->setStretchLastSection(false);
        algoParametersTable->verticalHeader()->setVisible(false);
        algoParametersTable->verticalHeader()->setDefaultSectionSize(21);
        algoParametersTable->verticalHeader()->setHighlightSections(true);
        algoParametersTable->verticalHeader()->setStretchLastSection(false);
        algoParametersSetPushButton = new QPushButton(algoGroupBox);
        algoParametersSetPushButton->setObjectName(QStringLiteral("algoParametersSetPushButton"));
        algoParametersSetPushButton->setGeometry(QRect(220, 170, 75, 23));
        algoParametersGetPushButton = new QPushButton(algoGroupBox);
        algoParametersGetPushButton->setObjectName(QStringLiteral("algoParametersGetPushButton"));
        algoParametersGetPushButton->setGeometry(QRect(380, 170, 75, 23));
        globalParametersGroupBox = new QGroupBox(AlgoTab);
        globalParametersGroupBox->setObjectName(QStringLiteral("globalParametersGroupBox"));
        globalParametersGroupBox->setGeometry(QRect(580, 10, 401, 211));
        globalParametersGroupBox->setMinimumSize(QSize(6, 0));
        globalParametersGroupBox->setCursor(QCursor(Qt::ArrowCursor));
        globalParametersGroupBox->setContextMenuPolicy(Qt::NoContextMenu);
        globalParametersGroupBox->setCheckable(false);
        globalParametersTable = new QTableWidget(globalParametersGroupBox);
        if (globalParametersTable->columnCount() < 4)
            globalParametersTable->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        globalParametersTable->setHorizontalHeaderItem(0, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        globalParametersTable->setHorizontalHeaderItem(1, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        globalParametersTable->setHorizontalHeaderItem(2, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        globalParametersTable->setHorizontalHeaderItem(3, __qtablewidgetitem9);
        if (globalParametersTable->rowCount() < 8)
            globalParametersTable->setRowCount(8);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        __qtablewidgetitem10->setCheckState(Qt::Checked);
        __qtablewidgetitem10->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable|Qt::ItemIsUserCheckable|Qt::ItemIsEnabled);
        globalParametersTable->setItem(0, 0, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        __qtablewidgetitem11->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable|Qt::ItemIsEnabled);
        globalParametersTable->setItem(0, 1, __qtablewidgetitem11);
        globalParametersTable->setObjectName(QStringLiteral("globalParametersTable"));
        globalParametersTable->setGeometry(QRect(20, 30, 361, 131));
        globalParametersTable->setFont(font);
        globalParametersTable->setAutoFillBackground(true);
        globalParametersTable->setStyleSheet(QLatin1String("background-color: rgb(0,0,0);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        globalParametersTable->setFrameShape(QFrame::Panel);
        globalParametersTable->setFrameShadow(QFrame::Plain);
        globalParametersTable->setLineWidth(2);
        globalParametersTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        globalParametersTable->setGridStyle(Qt::SolidLine);
        globalParametersTable->setWordWrap(false);
        globalParametersTable->setCornerButtonEnabled(false);
        globalParametersTable->setRowCount(8);
        globalParametersTable->setColumnCount(4);
        globalParametersTable->horizontalHeader()->setVisible(false);
        globalParametersTable->horizontalHeader()->setCascadingSectionResizes(false);
        globalParametersTable->horizontalHeader()->setDefaultSectionSize(45);
        globalParametersTable->horizontalHeader()->setHighlightSections(true);
        globalParametersTable->horizontalHeader()->setMinimumSectionSize(10);
        globalParametersTable->horizontalHeader()->setStretchLastSection(false);
        globalParametersTable->verticalHeader()->setVisible(false);
        globalParametersTable->verticalHeader()->setDefaultSectionSize(21);
        globalParametersTable->verticalHeader()->setHighlightSections(true);
        globalParametersTable->verticalHeader()->setStretchLastSection(false);
        globalParametersSetPushButton = new QPushButton(globalParametersGroupBox);
        globalParametersSetPushButton->setObjectName(QStringLiteral("globalParametersSetPushButton"));
        globalParametersSetPushButton->setGeometry(QRect(90, 170, 75, 23));
        globalParametersGetPushButton = new QPushButton(globalParametersGroupBox);
        globalParametersGetPushButton->setObjectName(QStringLiteral("globalParametersGetPushButton"));
        globalParametersGetPushButton->setGeometry(QRect(250, 170, 75, 23));
        interfaceTabs->addTab(AlgoTab, QString());
        TrackerTab = new QWidget();
        TrackerTab->setObjectName(QStringLiteral("TrackerTab"));
        trackerGroupBox = new QGroupBox(TrackerTab);
        trackerGroupBox->setObjectName(QStringLiteral("trackerGroupBox"));
        trackerGroupBox->setGeometry(QRect(20, 10, 541, 211));
        sizePolicy2.setHeightForWidth(trackerGroupBox->sizePolicy().hasHeightForWidth());
        trackerGroupBox->setSizePolicy(sizePolicy2);
        trackerGroupBox->setMinimumSize(QSize(1, 0));
        trackerSelectGroupBox = new QGroupBox(trackerGroupBox);
        trackerSelectGroupBox->setObjectName(QStringLiteral("trackerSelectGroupBox"));
        trackerSelectGroupBox->setGeometry(QRect(20, 20, 121, 171));
        trackerSelectGroupBox->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        trackerSelectComboBox = new QComboBox(trackerSelectGroupBox);
        trackerSelectComboBox->setObjectName(QStringLiteral("trackerSelectComboBox"));
        trackerSelectComboBox->setGeometry(QRect(10, 30, 101, 22));
        trackerSelectComboBox->setStyleSheet(QStringLiteral(""));
        trackerSelectComboBox->setMaxCount(2147483647);
        trackerParametersTable = new QTableWidget(trackerGroupBox);
        if (trackerParametersTable->columnCount() < 4)
            trackerParametersTable->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        trackerParametersTable->setHorizontalHeaderItem(0, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        trackerParametersTable->setHorizontalHeaderItem(1, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        trackerParametersTable->setHorizontalHeaderItem(2, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        trackerParametersTable->setHorizontalHeaderItem(3, __qtablewidgetitem15);
        if (trackerParametersTable->rowCount() < 8)
            trackerParametersTable->setRowCount(8);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        __qtablewidgetitem16->setCheckState(Qt::Checked);
        __qtablewidgetitem16->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable|Qt::ItemIsUserCheckable|Qt::ItemIsEnabled);
        trackerParametersTable->setItem(0, 0, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        __qtablewidgetitem17->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable|Qt::ItemIsEnabled);
        trackerParametersTable->setItem(0, 1, __qtablewidgetitem17);
        trackerParametersTable->setObjectName(QStringLiteral("trackerParametersTable"));
        trackerParametersTable->setGeometry(QRect(160, 30, 361, 131));
        trackerParametersTable->setFont(font);
        trackerParametersTable->setAutoFillBackground(true);
        trackerParametersTable->setStyleSheet(QLatin1String("background-color: rgb(0,0,0);\n"
"gridline-color: rgb(199, 199, 199);\n"
"border-color: rgb(255, 255, 255);\n"
"color: rgb(255,255,255);"));
        trackerParametersTable->setFrameShape(QFrame::Panel);
        trackerParametersTable->setFrameShadow(QFrame::Plain);
        trackerParametersTable->setLineWidth(2);
        trackerParametersTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        trackerParametersTable->setGridStyle(Qt::SolidLine);
        trackerParametersTable->setWordWrap(false);
        trackerParametersTable->setCornerButtonEnabled(false);
        trackerParametersTable->setRowCount(8);
        trackerParametersTable->setColumnCount(4);
        trackerParametersTable->horizontalHeader()->setVisible(false);
        trackerParametersTable->horizontalHeader()->setCascadingSectionResizes(false);
        trackerParametersTable->horizontalHeader()->setDefaultSectionSize(45);
        trackerParametersTable->horizontalHeader()->setHighlightSections(true);
        trackerParametersTable->horizontalHeader()->setMinimumSectionSize(10);
        trackerParametersTable->horizontalHeader()->setStretchLastSection(false);
        trackerParametersTable->verticalHeader()->setVisible(false);
        trackerParametersTable->verticalHeader()->setDefaultSectionSize(21);
        trackerParametersTable->verticalHeader()->setHighlightSections(true);
        trackerParametersTable->verticalHeader()->setStretchLastSection(false);
        trackerParametersSetPushButton = new QPushButton(trackerGroupBox);
        trackerParametersSetPushButton->setObjectName(QStringLiteral("trackerParametersSetPushButton"));
        trackerParametersSetPushButton->setGeometry(QRect(220, 170, 75, 23));
        trackerParametersGetPushButton = new QPushButton(trackerGroupBox);
        trackerParametersGetPushButton->setObjectName(QStringLiteral("trackerParametersGetPushButton"));
        trackerParametersGetPushButton->setGeometry(QRect(380, 170, 75, 23));
        interfaceTabs->addTab(TrackerTab, QString());

        verticalLayout->addWidget(interfaceTabs);

        gridDisplayLayout = new QGridLayout();
        gridDisplayLayout->setSpacing(6);
        gridDisplayLayout->setObjectName(QStringLiteral("gridDisplayLayout"));

        verticalLayout->addLayout(gridDisplayLayout);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

        AWLQtDemoClass->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(AWLQtDemoClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(40);
        sizePolicy3.setHeightForWidth(mainToolBar->sizePolicy().hasHeightForWidth());
        mainToolBar->setSizePolicy(sizePolicy3);
        mainToolBar->setMinimumSize(QSize(0, 0));
        mainToolBar->setBaseSize(QSize(0, 40));
        mainToolBar->setIconSize(QSize(40, 40));
        AWLQtDemoClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(AWLQtDemoClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        AWLQtDemoClass->setStatusBar(statusBar);
        QWidget::setTabOrder(sensorHeightSpinBox, sensorDepthSpinBox);
        QWidget::setTabOrder(sensorDepthSpinBox, sensorRangeMinSpinBox);
        QWidget::setTabOrder(sensorRangeMinSpinBox, sensorRangeMaxSpinBox);
        QWidget::setTabOrder(sensorRangeMaxSpinBox, measurementOffsetSpinBox);
        QWidget::setTabOrder(measurementOffsetSpinBox, calibrationFrameQtySpinBox);
        QWidget::setTabOrder(calibrationFrameQtySpinBox, calibrationBetaDoubleSpinBox);
        QWidget::setTabOrder(calibrationBetaDoubleSpinBox, calibrationChannel1CheckBox);
        QWidget::setTabOrder(calibrationChannel1CheckBox, calibrationChannel2CheckBox);
        QWidget::setTabOrder(calibrationChannel2CheckBox, calibrationChannel3CheckBox);
        QWidget::setTabOrder(calibrationChannel3CheckBox, calibrationChannel4CheckBox);
        QWidget::setTabOrder(calibrationChannel4CheckBox, calibrationChannel5CheckBox);
        QWidget::setTabOrder(calibrationChannel5CheckBox, calibrationChannel6CheckBox);
        QWidget::setTabOrder(calibrationChannel6CheckBox, calibrationChannel7CheckBox);
        QWidget::setTabOrder(calibrationChannel7CheckBox, calibrateButton);
        QWidget::setTabOrder(calibrateButton, targetHintDistanceSpinBox);
        QWidget::setTabOrder(targetHintDistanceSpinBox, targetHintAngleSpinBox);
        QWidget::setTabOrder(targetHintAngleSpinBox, distanceLogFileCheckbox);
        QWidget::setTabOrder(distanceLogFileCheckbox, recordButton);
        QWidget::setTabOrder(recordButton, playbackButton);
        QWidget::setTabOrder(playbackButton, recordChannel7CheckBox);
        QWidget::setTabOrder(recordChannel7CheckBox, recordChannel6CheckBox);
        QWidget::setTabOrder(recordChannel6CheckBox, recordChannel5CheckBox);
        QWidget::setTabOrder(recordChannel5CheckBox, recordChannel2CheckBox);
        QWidget::setTabOrder(recordChannel2CheckBox, recordChannel3CheckBox);
        QWidget::setTabOrder(recordChannel3CheckBox, recordChannel1CheckBox);
        QWidget::setTabOrder(recordChannel1CheckBox, recordChannel4CheckBox);
        QWidget::setTabOrder(recordChannel4CheckBox, frameRateSpinBox);
        QWidget::setTabOrder(frameRateSpinBox, stopButton);
        QWidget::setTabOrder(stopButton, recordFileNameEdit);
        QWidget::setTabOrder(recordFileNameEdit, playbackFileNameEdit);
        QWidget::setTabOrder(playbackFileNameEdit, versionEdit);
        QWidget::setTabOrder(versionEdit, temperatureEdit);
        QWidget::setTabOrder(temperatureEdit, voltageEdit);
        QWidget::setTabOrder(voltageEdit, bootReceiverCheckBox);
        QWidget::setTabOrder(bootReceiverCheckBox, bootEmitter1CheckBox);
        QWidget::setTabOrder(bootEmitter1CheckBox, bootAuxChecksumCheckBox);
        QWidget::setTabOrder(bootAuxChecksumCheckBox, bootEmitter2CheckBox);
        QWidget::setTabOrder(bootEmitter2CheckBox, bootMainChecksumCheckBox);
        QWidget::setTabOrder(bootMainChecksumCheckBox, bootMemoryCheckBox);
        QWidget::setTabOrder(bootMemoryCheckBox, bootDSPCheckBox);
        QWidget::setTabOrder(bootDSPCheckBox, bootChecksumCheckBox);
        QWidget::setTabOrder(bootChecksumCheckBox, statusSelfTestCheckBox);
        QWidget::setTabOrder(statusSelfTestCheckBox, statusShutdownCheckBox);
        QWidget::setTabOrder(statusShutdownCheckBox, statusSensorBlockedCheckBox);
        QWidget::setTabOrder(statusSensorBlockedCheckBox, statusReducedPerformanceCheckBox);
        QWidget::setTabOrder(statusReducedPerformanceCheckBox, statusSaturationCheckBox);
        QWidget::setTabOrder(statusSaturationCheckBox, statusSaturationCheckBox_2);
        QWidget::setTabOrder(statusSaturationCheckBox_2, statusSensorBlockedCheckBox_2);
        QWidget::setTabOrder(statusSensorBlockedCheckBox_2, statusShutdownCheckBox_2);
        QWidget::setTabOrder(statusShutdownCheckBox_2, statusReducedPerformanceCheckBox_2);
        QWidget::setTabOrder(statusReducedPerformanceCheckBox_2, statusSelfTestCheckBox_2);
        QWidget::setTabOrder(statusSelfTestCheckBox_2, hardwareReceiverCheckBox);
        QWidget::setTabOrder(hardwareReceiverCheckBox, hardwareEmitter1CheckBox);
        QWidget::setTabOrder(hardwareEmitter1CheckBox, hardwareEmitter2CheckBox);
        QWidget::setTabOrder(hardwareEmitter2CheckBox, hardwareMemoryCheckBox);
        QWidget::setTabOrder(hardwareMemoryCheckBox, hardwareDSPCheckBox);
        QWidget::setTabOrder(hardwareDSPCheckBox, receiverChannel3CheckBox);
        QWidget::setTabOrder(receiverChannel3CheckBox, receiverChannel1CheckBox);
        QWidget::setTabOrder(receiverChannel1CheckBox, receiverChannel2CheckBox);
        QWidget::setTabOrder(receiverChannel2CheckBox, receiverChannel5CheckBox);
        QWidget::setTabOrder(receiverChannel5CheckBox, receiverChannel4CheckBox);
        QWidget::setTabOrder(receiverChannel4CheckBox, receiverChannel7CheckBox);
        QWidget::setTabOrder(receiverChannel7CheckBox, receiverChannel6CheckBox);
        QWidget::setTabOrder(receiverChannel6CheckBox, registerFPGASetPushButton);
        QWidget::setTabOrder(registerFPGASetPushButton, registerFPGAGetPushButton);
        QWidget::setTabOrder(registerFPGAGetPushButton, registerFPGAAddressSetComboBox);
        QWidget::setTabOrder(registerFPGAAddressSetComboBox, registerFPGAValueSetLineEdit);
        QWidget::setTabOrder(registerFPGAValueSetLineEdit, registerFPGAAddressGetLineEdit);
        QWidget::setTabOrder(registerFPGAAddressGetLineEdit, registerFPGAValueGetLineEdit);
        QWidget::setTabOrder(registerFPGAValueGetLineEdit, registerADCSetPushButton);
        QWidget::setTabOrder(registerADCSetPushButton, registerADCGetPushButton);
        QWidget::setTabOrder(registerADCGetPushButton, registerADCAddressSetComboBox);
        QWidget::setTabOrder(registerADCAddressSetComboBox, registerADCValueSetLineEdit);
        QWidget::setTabOrder(registerADCValueSetLineEdit, registerADCAddressGetLineEdit);
        QWidget::setTabOrder(registerADCAddressGetLineEdit, registerADCValueGetLineEdit);
        QWidget::setTabOrder(registerADCValueGetLineEdit, registerGPIOListWidget);
        QWidget::setTabOrder(registerGPIOListWidget, registerGPIOGetPushButton);
        QWidget::setTabOrder(registerGPIOGetPushButton, registerGPIOSetPushButton);
        QWidget::setTabOrder(registerGPIOSetPushButton, algoParametersTable);
        QWidget::setTabOrder(algoParametersTable, algoParametersSetPushButton);
        QWidget::setTabOrder(algoParametersSetPushButton, algoParametersGetPushButton);
        QWidget::setTabOrder(algoParametersGetPushButton, globalParametersTable);
        QWidget::setTabOrder(globalParametersTable, globalParametersSetPushButton);
        QWidget::setTabOrder(globalParametersSetPushButton, globalParametersGetPushButton);

        retranslateUi(AWLQtDemoClass);
        QObject::connect(sensorHeightSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_sensorHeightSpin_editingFinished()));
        QObject::connect(sensorDepthSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_sensorDepthSpin_editingFinished()));
        QObject::connect(measurementOffsetSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_measurementOffsetSpin_editingFinished()));
        QObject::connect(globalParametersGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_globalParametersGetPushButton_clicked()));
        QObject::connect(recordButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_recordPushButton_clicked()));
        QObject::connect(sensorRangeMinSpinBox, SIGNAL(valueChanged(double)), AWLQtDemoClass, SLOT(on_calibrationRangeMinSpin_editingFinished()));
        QObject::connect(playbackButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_playbackPushButton_clicked()));
        QObject::connect(stopButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_stopPushButton_clicked()));
        QObject::connect(calibrateButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_calibratePushButton_clicked()));
        QObject::connect(registerFPGASetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerFPGASetPushButton_clicked()));
        QObject::connect(registerFPGAGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerFPGAGetPushButton_clicked()));
        QObject::connect(registerADCSetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerADCSetPushButton_clicked()));
        QObject::connect(registerADCGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerADCGetPushButton_clicked()));
        QObject::connect(registerGPIOSetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerGPIOSetPushButton_clicked()));
        QObject::connect(registerGPIOGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_registerGPIOGetPushButton_clicked()));
        QObject::connect(algoParametersGetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_algoParametersGetPushButton_clicked()));
        QObject::connect(algoParametersSetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_algoParametersSetPushButton_clicked()));
        QObject::connect(globalParametersSetPushButton, SIGNAL(clicked()), AWLQtDemoClass, SLOT(on_globalParametersSetPushButton_clicked()));
        QObject::connect(targetHintDistanceSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_targetHintDistanceSpin_editingFinished()));
        QObject::connect(targetHintAngleSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_targetHintAngleSpin_editingFinished()));
        QObject::connect(distanceLogFileCheckbox, SIGNAL(toggled(bool)), AWLQtDemoClass, SLOT(on_distanceLogCheckBox_setChecked(bool)));
        QObject::connect(sensorRangeMaxSpinBox, SIGNAL(editingFinished()), AWLQtDemoClass, SLOT(on_calibrationRangeMaxSpin_editingFinished()));
        QObject::connect(algoSelectComboBox, SIGNAL(currentIndexChanged(int)), AWLQtDemoClass, SLOT(on_algoSelectComboBox_indexChanged(int)));
        QObject::connect(trackerSelectComboBox, SIGNAL(currentIndexChanged(int)), AWLQtDemoClass, SLOT(on_trackerSelectComboBox_indexChanged(int)));
        QObject::connect(trackerParametersSetPushButton, SIGNAL(clicked(bool)), AWLQtDemoClass, SLOT(on_trackerParametersSetPushButton_clicked()));
        QObject::connect(trackerParametersGetPushButton, SIGNAL(clicked(bool)), AWLQtDemoClass, SLOT(on_trackerParametersGetPushButton_clicked()));

        interfaceTabs->setCurrentIndex(0);
        recordButton->setDefault(false);
        playbackButton->setDefault(false);
        stopButton->setDefault(false);


        QMetaObject::connectSlotsByName(AWLQtDemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *AWLQtDemoClass)
    {
        AWLQtDemoClass->setWindowTitle(QApplication::translate("AWLQtDemoClass", "AWLQtDemo", Q_NULLPTR));
        actionQuitter->setText(QApplication::translate("AWLQtDemoClass", "Quitter", Q_NULLPTR));
        actionGraph->setText(QApplication::translate("AWLQtDemoClass", "Scope", Q_NULLPTR));
        action2D->setText(QApplication::translate("AWLQtDemoClass", "2D", Q_NULLPTR));
        actionCamera->setText(QApplication::translate("AWLQtDemoClass", "Camera", Q_NULLPTR));
        actionTableView->setText(QApplication::translate("AWLQtDemoClass", "Table View", Q_NULLPTR));
        actionSettings->setText(QApplication::translate("AWLQtDemoClass", "Settings", Q_NULLPTR));
        internalCalibrationGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Internal Calibration", Q_NULLPTR));
        calibrationChannelMaskGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Channel mask", Q_NULLPTR));
        calibrationChannel7CheckBox->setText(QApplication::translate("AWLQtDemoClass", "7", Q_NULLPTR));
        calibrationChannel6CheckBox->setText(QApplication::translate("AWLQtDemoClass", "6", Q_NULLPTR));
        calibrationChannel5CheckBox->setText(QApplication::translate("AWLQtDemoClass", "5", Q_NULLPTR));
        calibrationChannel2CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 2", Q_NULLPTR));
        calibrationChannel3CheckBox->setText(QApplication::translate("AWLQtDemoClass", "3", Q_NULLPTR));
        calibrationChannel1CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 1", Q_NULLPTR));
        calibrationChannel4CheckBox->setText(QApplication::translate("AWLQtDemoClass", "4", Q_NULLPTR));
        calibrationBetaLabel->setText(QApplication::translate("AWLQtDemoClass", "Beta:", Q_NULLPTR));
        calibrationFrameQtySpinBox->setSuffix(QString());
        calibrationFrameQtyLabel->setText(QApplication::translate("AWLQtDemoClass", "Frame qty:", Q_NULLPTR));
        calibrateButton->setText(QApplication::translate("AWLQtDemoClass", "&Calibrate (Alt+C)", Q_NULLPTR));
        externalCalibrationGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Sensor Range and position", Q_NULLPTR));
        sensorPositionGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Sensor Position", Q_NULLPTR));
        sensorHeightSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", Q_NULLPTR));
        sensorHeightLabel->setText(QApplication::translate("AWLQtDemoClass", "Sensor height:", Q_NULLPTR));
        sensorDepthSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", Q_NULLPTR));
        sensorDepthLabel->setText(QApplication::translate("AWLQtDemoClass", "Sensor forward:", Q_NULLPTR));
        receiverCalibrationGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Measurement Offsets", Q_NULLPTR));
        measurementOffsetLabel->setText(QApplication::translate("AWLQtDemoClass", "Range offset:", Q_NULLPTR));
        measurementOffsetSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", Q_NULLPTR));
        sensorRangeGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Displayed Range", Q_NULLPTR));
        sensorRangeMinLabel->setText(QApplication::translate("AWLQtDemoClass", "Range min:", Q_NULLPTR));
        sensorRangeMaxLabel->setText(QApplication::translate("AWLQtDemoClass", "Range max:", Q_NULLPTR));
        sensorRangeMinSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", Q_NULLPTR));
        sensorRangeMaxSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", Q_NULLPTR));
        targetHintGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Obstacle log hint", Q_NULLPTR));
        targetHintAngleLabel->setText(QApplication::translate("AWLQtDemoClass", "Angle:", Q_NULLPTR));
        targetHintAngleSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", " deg.", Q_NULLPTR));
        targetHintDistanceLabel->setText(QApplication::translate("AWLQtDemoClass", "Distance:", Q_NULLPTR));
        targetHintDistanceSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "m.", Q_NULLPTR));
        distanceLogFileCheckbox->setText(QApplication::translate("AWLQtDemoClass", "&Logging Enabled (Alt+L)", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(calibrationTab), QApplication::translate("AWLQtDemoClass", "Calibration", Q_NULLPTR));
        recordPlayGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Record / Play", Q_NULLPTR));
        recordButton->setText(QApplication::translate("AWLQtDemoClass", "Record", Q_NULLPTR));
        playbackButton->setText(QApplication::translate("AWLQtDemoClass", "Playback", Q_NULLPTR));
        channelMaskGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Channel mask", Q_NULLPTR));
        recordChannel7CheckBox->setText(QApplication::translate("AWLQtDemoClass", "7", Q_NULLPTR));
        recordChannel6CheckBox->setText(QApplication::translate("AWLQtDemoClass", "6", Q_NULLPTR));
        recordChannel5CheckBox->setText(QApplication::translate("AWLQtDemoClass", "5", Q_NULLPTR));
        recordChannel2CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 2", Q_NULLPTR));
        recordChannel3CheckBox->setText(QApplication::translate("AWLQtDemoClass", "3", Q_NULLPTR));
        recordChannel1CheckBox->setText(QApplication::translate("AWLQtDemoClass", " 1", Q_NULLPTR));
        recordChannel4CheckBox->setText(QApplication::translate("AWLQtDemoClass", "4", Q_NULLPTR));
        frameRateLabel->setText(QApplication::translate("AWLQtDemoClass", "Frame rate:", Q_NULLPTR));
        frameRateSpinBox->setSuffix(QApplication::translate("AWLQtDemoClass", "Hz.", Q_NULLPTR));
        stopButton->setText(QApplication::translate("AWLQtDemoClass", "Stop", Q_NULLPTR));
        recordFileNameGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "File names", Q_NULLPTR));
        label->setText(QApplication::translate("AWLQtDemoClass", "Record:", Q_NULLPTR));
        label_2->setText(QApplication::translate("AWLQtDemoClass", "Playback:", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(controlTab), QApplication::translate("AWLQtDemoClass", "Control", Q_NULLPTR));
        VersionLabel->setText(QApplication::translate("AWLQtDemoClass", "Version:", Q_NULLPTR));
        TemperatureLabel->setText(QApplication::translate("AWLQtDemoClass", "Temp:", Q_NULLPTR));
        TemperatureLabel_2->setText(QApplication::translate("AWLQtDemoClass", "<html><head/><body><p><span style=\" vertical-align:super;\">o</span>C</p></body></html>", Q_NULLPTR));
        VoltageLabel->setText(QApplication::translate("AWLQtDemoClass", "Voltage:", Q_NULLPTR));
        VoltageLabel_2->setText(QApplication::translate("AWLQtDemoClass", "volts", Q_NULLPTR));
        bootGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Boot errors", Q_NULLPTR));
        bootReceiverCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Receiver", Q_NULLPTR));
        bootEmitter1CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 1", Q_NULLPTR));
        bootAuxChecksumCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Aux. checksum", Q_NULLPTR));
        bootEmitter2CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 2", Q_NULLPTR));
        bootMainChecksumCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Main checksum", Q_NULLPTR));
        bootMemoryCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Memory", Q_NULLPTR));
        bootDSPCheckBox->setText(QApplication::translate("AWLQtDemoClass", "DSP", Q_NULLPTR));
        bootChecksumCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Checksum", Q_NULLPTR));
        statusGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Status", Q_NULLPTR));
        statusSelfTestCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Self test", Q_NULLPTR));
        statusShutdownCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Shutdown", Q_NULLPTR));
        statusSensorBlockedCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Sensor blocked", Q_NULLPTR));
        statusReducedPerformanceCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Reduced performance", Q_NULLPTR));
        statusSaturationCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Saturation", Q_NULLPTR));
        statusSaturationCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Saturation", Q_NULLPTR));
        statusSensorBlockedCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Sensor blocked", Q_NULLPTR));
        statusShutdownCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Shutdown", Q_NULLPTR));
        statusReducedPerformanceCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Reduced performance", Q_NULLPTR));
        statusSelfTestCheckBox_2->setText(QApplication::translate("AWLQtDemoClass", "Self test", Q_NULLPTR));
        hardwareGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Hardware errors", Q_NULLPTR));
        hardwareReceiverCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Receiver", Q_NULLPTR));
        hardwareEmitter1CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 1", Q_NULLPTR));
        hardwareEmitter2CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Emitter 2", Q_NULLPTR));
        hardwareMemoryCheckBox->setText(QApplication::translate("AWLQtDemoClass", "Memory", Q_NULLPTR));
        hardwareDSPCheckBox->setText(QApplication::translate("AWLQtDemoClass", "DSP", Q_NULLPTR));
        receiverGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "ReceiverErrors", Q_NULLPTR));
        receiverChannel3CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 3", Q_NULLPTR));
        receiverChannel1CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 1", Q_NULLPTR));
        receiverChannel2CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 2", Q_NULLPTR));
        receiverChannel5CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 5", Q_NULLPTR));
        receiverChannel4CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 4", Q_NULLPTR));
        receiverChannel7CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 7", Q_NULLPTR));
        receiverChannel6CheckBox->setText(QApplication::translate("AWLQtDemoClass", "Channel 6", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(statusTab), QApplication::translate("AWLQtDemoClass", "Status", Q_NULLPTR));
        registerFPGAGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "AWL Registers", Q_NULLPTR));
        registerFPGASetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        registerFPGAGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", Q_NULLPTR));
        registerFPGASetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        registerFPGAAddressSetLabel->setText(QApplication::translate("AWLQtDemoClass", "AWL Register:", Q_NULLPTR));
        registerFPGAValueSetLabel->setText(QApplication::translate("AWLQtDemoClass", "Value:", Q_NULLPTR));
        registerFPGAValueSetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", Q_NULLPTR));
        registerFPGAValueSetLineEdit->setText(QApplication::translate("AWLQtDemoClass", "FF", Q_NULLPTR));
        registerFPGAGetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Confirm", Q_NULLPTR));
        registerFPGAAddressGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", Q_NULLPTR));
        registerFPGAValueGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", Q_NULLPTR));
        registerADCGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "ADC Registers", Q_NULLPTR));
        registerADCSetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        registerADCGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", Q_NULLPTR));
        registerADCSetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        registerADCAddressSetLabel->setText(QApplication::translate("AWLQtDemoClass", "ADC Register:", Q_NULLPTR));
        registerADCValueSetLabel->setText(QApplication::translate("AWLQtDemoClass", "Value:", Q_NULLPTR));
        registerADCValueSetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", Q_NULLPTR));
        registerADCValueSetLineEdit->setText(QApplication::translate("AWLQtDemoClass", "FF", Q_NULLPTR));
        registerADCGetGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Confirm", Q_NULLPTR));
        registerADCAddressGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", Q_NULLPTR));
        registerADCValueGetLineEdit->setInputMask(QApplication::translate("AWLQtDemoClass", "hhhhhhhH", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(registersTab), QApplication::translate("AWLQtDemoClass", "Registers", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("AWLQtDemoClass", "GPIOs", Q_NULLPTR));
        registerGPIOGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", Q_NULLPTR));
        registerGPIOSetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(gpiosTab), QApplication::translate("AWLQtDemoClass", "GPIOs", Q_NULLPTR));
        algoGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Algorithm parameters", Q_NULLPTR));
        algoSelectGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Detection Algorithm", Q_NULLPTR));
        algoSelectComboBox->setCurrentText(QString());
        QTableWidgetItem *___qtablewidgetitem = algoParametersTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("AWLQtDemoClass", "Select", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem1 = algoParametersTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("AWLQtDemoClass", "Description", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem2 = algoParametersTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("AWLQtDemoClass", "Value", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem3 = algoParametersTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("AWLQtDemoClass", "Confirm", Q_NULLPTR));

        const bool __sortingEnabled = algoParametersTable->isSortingEnabled();
        algoParametersTable->setSortingEnabled(false);
        algoParametersTable->setSortingEnabled(__sortingEnabled);

        algoParametersSetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        algoParametersGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", Q_NULLPTR));
        globalParametersGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Global Parameters", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem4 = globalParametersTable->horizontalHeaderItem(0);
        ___qtablewidgetitem4->setText(QApplication::translate("AWLQtDemoClass", "Select", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem5 = globalParametersTable->horizontalHeaderItem(1);
        ___qtablewidgetitem5->setText(QApplication::translate("AWLQtDemoClass", "Description", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem6 = globalParametersTable->horizontalHeaderItem(2);
        ___qtablewidgetitem6->setText(QApplication::translate("AWLQtDemoClass", "Value", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem7 = globalParametersTable->horizontalHeaderItem(3);
        ___qtablewidgetitem7->setText(QApplication::translate("AWLQtDemoClass", "Confirm", Q_NULLPTR));

        const bool __sortingEnabled1 = globalParametersTable->isSortingEnabled();
        globalParametersTable->setSortingEnabled(false);
        globalParametersTable->setSortingEnabled(__sortingEnabled1);

        globalParametersSetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        globalParametersGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(AlgoTab), QApplication::translate("AWLQtDemoClass", "Algo Control", Q_NULLPTR));
        trackerGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Tracker parameters", Q_NULLPTR));
        trackerSelectGroupBox->setTitle(QApplication::translate("AWLQtDemoClass", "Tracking Algorithm", Q_NULLPTR));
        trackerSelectComboBox->setCurrentText(QString());
        QTableWidgetItem *___qtablewidgetitem8 = trackerParametersTable->horizontalHeaderItem(0);
        ___qtablewidgetitem8->setText(QApplication::translate("AWLQtDemoClass", "Select", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem9 = trackerParametersTable->horizontalHeaderItem(1);
        ___qtablewidgetitem9->setText(QApplication::translate("AWLQtDemoClass", "Description", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem10 = trackerParametersTable->horizontalHeaderItem(2);
        ___qtablewidgetitem10->setText(QApplication::translate("AWLQtDemoClass", "Value", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem11 = trackerParametersTable->horizontalHeaderItem(3);
        ___qtablewidgetitem11->setText(QApplication::translate("AWLQtDemoClass", "Confirm", Q_NULLPTR));

        const bool __sortingEnabled2 = trackerParametersTable->isSortingEnabled();
        trackerParametersTable->setSortingEnabled(false);
        trackerParametersTable->setSortingEnabled(__sortingEnabled2);

        trackerParametersSetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Set", Q_NULLPTR));
        trackerParametersGetPushButton->setText(QApplication::translate("AWLQtDemoClass", "Get", Q_NULLPTR));
        interfaceTabs->setTabText(interfaceTabs->indexOf(TrackerTab), QApplication::translate("AWLQtDemoClass", "Tracker Control", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class AWLQtDemoClass: public Ui_AWLQtDemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AWLQTDEMO_H
