/********************************************************************************
** Form generated from reading UI file 'TableView.ui'
**
** Created by: Qt User Interface Compiler version 5.9.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TABLEVIEW_H
#define UI_TABLEVIEW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_AWLTableView
{
public:
    QVBoxLayout *verticalLayout;
    QTableWidget *distanceTable;

    void setupUi(QFrame *AWLTableView)
    {
        if (AWLTableView->objectName().isEmpty())
            AWLTableView->setObjectName(QStringLiteral("AWLTableView"));
        AWLTableView->resize(506, 190);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AWLTableView->sizePolicy().hasHeightForWidth());
        AWLTableView->setSizePolicy(sizePolicy);
        AWLTableView->setMinimumSize(QSize(506, 190));
        QFont font;
        font.setFamily(QStringLiteral("MS Shell Dlg 2"));
        AWLTableView->setFont(font);
        QIcon icon;
        icon.addFile(QStringLiteral("AWLQtDemo.ico"), QSize(), QIcon::Normal, QIcon::Off);
        AWLTableView->setWindowIcon(icon);
        AWLTableView->setStyleSheet(QStringLiteral(""));
        AWLTableView->setFrameShape(QFrame::NoFrame);
        AWLTableView->setLineWidth(2);
        verticalLayout = new QVBoxLayout(AWLTableView);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        distanceTable = new QTableWidget(AWLTableView);
        if (distanceTable->columnCount() < 8)
            distanceTable->setColumnCount(8);
        QFont font1;
        font1.setPointSize(14);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        __qtablewidgetitem->setFont(font1);
        distanceTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        __qtablewidgetitem1->setFont(font1);
        distanceTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        __qtablewidgetitem2->setFont(font1);
        distanceTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        __qtablewidgetitem3->setFont(font1);
        distanceTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        __qtablewidgetitem4->setFont(font1);
        distanceTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        __qtablewidgetitem5->setFont(font1);
        distanceTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        __qtablewidgetitem6->setFont(font1);
        distanceTable->setHorizontalHeaderItem(6, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        __qtablewidgetitem7->setFont(font1);
        distanceTable->setHorizontalHeaderItem(7, __qtablewidgetitem7);
        if (distanceTable->rowCount() < 8)
            distanceTable->setRowCount(8);
        QFont font2;
        font2.setPointSize(12);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        __qtablewidgetitem8->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem8->setFont(font2);
        distanceTable->setItem(0, 0, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        __qtablewidgetitem9->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem9->setFont(font2);
        distanceTable->setItem(0, 1, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        __qtablewidgetitem10->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem10->setFont(font2);
        distanceTable->setItem(0, 2, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        __qtablewidgetitem11->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem11->setFont(font2);
        distanceTable->setItem(0, 3, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        __qtablewidgetitem12->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem12->setFont(font2);
        distanceTable->setItem(0, 4, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        __qtablewidgetitem13->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem13->setFont(font2);
        distanceTable->setItem(0, 5, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        __qtablewidgetitem14->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem14->setFont(font2);
        distanceTable->setItem(0, 6, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        __qtablewidgetitem15->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem15->setFont(font2);
        distanceTable->setItem(0, 7, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        __qtablewidgetitem16->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem16->setFont(font2);
        distanceTable->setItem(1, 0, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        __qtablewidgetitem17->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem17->setFont(font2);
        distanceTable->setItem(1, 1, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        __qtablewidgetitem18->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem18->setFont(font2);
        distanceTable->setItem(1, 2, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        __qtablewidgetitem19->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem19->setFont(font2);
        distanceTable->setItem(1, 3, __qtablewidgetitem19);
        QTableWidgetItem *__qtablewidgetitem20 = new QTableWidgetItem();
        __qtablewidgetitem20->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem20->setFont(font2);
        distanceTable->setItem(1, 4, __qtablewidgetitem20);
        QTableWidgetItem *__qtablewidgetitem21 = new QTableWidgetItem();
        __qtablewidgetitem21->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem21->setFont(font2);
        distanceTable->setItem(1, 5, __qtablewidgetitem21);
        QTableWidgetItem *__qtablewidgetitem22 = new QTableWidgetItem();
        __qtablewidgetitem22->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem22->setFont(font2);
        distanceTable->setItem(1, 6, __qtablewidgetitem22);
        QTableWidgetItem *__qtablewidgetitem23 = new QTableWidgetItem();
        __qtablewidgetitem23->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem23->setFont(font2);
        distanceTable->setItem(1, 7, __qtablewidgetitem23);
        QTableWidgetItem *__qtablewidgetitem24 = new QTableWidgetItem();
        __qtablewidgetitem24->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem24->setFont(font2);
        distanceTable->setItem(2, 0, __qtablewidgetitem24);
        QTableWidgetItem *__qtablewidgetitem25 = new QTableWidgetItem();
        __qtablewidgetitem25->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem25->setFont(font2);
        distanceTable->setItem(2, 1, __qtablewidgetitem25);
        QTableWidgetItem *__qtablewidgetitem26 = new QTableWidgetItem();
        __qtablewidgetitem26->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem26->setFont(font2);
        distanceTable->setItem(2, 2, __qtablewidgetitem26);
        QTableWidgetItem *__qtablewidgetitem27 = new QTableWidgetItem();
        __qtablewidgetitem27->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem27->setFont(font2);
        distanceTable->setItem(2, 3, __qtablewidgetitem27);
        QTableWidgetItem *__qtablewidgetitem28 = new QTableWidgetItem();
        __qtablewidgetitem28->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem28->setFont(font2);
        distanceTable->setItem(2, 4, __qtablewidgetitem28);
        QTableWidgetItem *__qtablewidgetitem29 = new QTableWidgetItem();
        __qtablewidgetitem29->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem29->setFont(font2);
        distanceTable->setItem(2, 5, __qtablewidgetitem29);
        QTableWidgetItem *__qtablewidgetitem30 = new QTableWidgetItem();
        __qtablewidgetitem30->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem30->setFont(font2);
        distanceTable->setItem(2, 6, __qtablewidgetitem30);
        QTableWidgetItem *__qtablewidgetitem31 = new QTableWidgetItem();
        __qtablewidgetitem31->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem31->setFont(font2);
        distanceTable->setItem(2, 7, __qtablewidgetitem31);
        QTableWidgetItem *__qtablewidgetitem32 = new QTableWidgetItem();
        __qtablewidgetitem32->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem32->setFont(font2);
        distanceTable->setItem(3, 0, __qtablewidgetitem32);
        QTableWidgetItem *__qtablewidgetitem33 = new QTableWidgetItem();
        __qtablewidgetitem33->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem33->setFont(font2);
        distanceTable->setItem(3, 1, __qtablewidgetitem33);
        QTableWidgetItem *__qtablewidgetitem34 = new QTableWidgetItem();
        __qtablewidgetitem34->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem34->setFont(font2);
        distanceTable->setItem(3, 2, __qtablewidgetitem34);
        QTableWidgetItem *__qtablewidgetitem35 = new QTableWidgetItem();
        __qtablewidgetitem35->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem35->setFont(font2);
        distanceTable->setItem(3, 3, __qtablewidgetitem35);
        QTableWidgetItem *__qtablewidgetitem36 = new QTableWidgetItem();
        __qtablewidgetitem36->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem36->setFont(font2);
        distanceTable->setItem(3, 4, __qtablewidgetitem36);
        QTableWidgetItem *__qtablewidgetitem37 = new QTableWidgetItem();
        __qtablewidgetitem37->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem37->setFont(font2);
        distanceTable->setItem(3, 5, __qtablewidgetitem37);
        QTableWidgetItem *__qtablewidgetitem38 = new QTableWidgetItem();
        __qtablewidgetitem38->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem38->setFont(font2);
        distanceTable->setItem(3, 6, __qtablewidgetitem38);
        QTableWidgetItem *__qtablewidgetitem39 = new QTableWidgetItem();
        __qtablewidgetitem39->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem39->setFont(font2);
        distanceTable->setItem(3, 7, __qtablewidgetitem39);
        QTableWidgetItem *__qtablewidgetitem40 = new QTableWidgetItem();
        __qtablewidgetitem40->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem40->setFont(font2);
        distanceTable->setItem(4, 0, __qtablewidgetitem40);
        QTableWidgetItem *__qtablewidgetitem41 = new QTableWidgetItem();
        __qtablewidgetitem41->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem41->setFont(font2);
        distanceTable->setItem(4, 1, __qtablewidgetitem41);
        QTableWidgetItem *__qtablewidgetitem42 = new QTableWidgetItem();
        __qtablewidgetitem42->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem42->setFont(font2);
        distanceTable->setItem(4, 2, __qtablewidgetitem42);
        QTableWidgetItem *__qtablewidgetitem43 = new QTableWidgetItem();
        __qtablewidgetitem43->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem43->setFont(font2);
        distanceTable->setItem(4, 3, __qtablewidgetitem43);
        QTableWidgetItem *__qtablewidgetitem44 = new QTableWidgetItem();
        __qtablewidgetitem44->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem44->setFont(font2);
        distanceTable->setItem(4, 4, __qtablewidgetitem44);
        QTableWidgetItem *__qtablewidgetitem45 = new QTableWidgetItem();
        __qtablewidgetitem45->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem45->setFont(font2);
        distanceTable->setItem(4, 5, __qtablewidgetitem45);
        QTableWidgetItem *__qtablewidgetitem46 = new QTableWidgetItem();
        __qtablewidgetitem46->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem46->setFont(font2);
        distanceTable->setItem(4, 6, __qtablewidgetitem46);
        QTableWidgetItem *__qtablewidgetitem47 = new QTableWidgetItem();
        __qtablewidgetitem47->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem47->setFont(font2);
        distanceTable->setItem(4, 7, __qtablewidgetitem47);
        QTableWidgetItem *__qtablewidgetitem48 = new QTableWidgetItem();
        __qtablewidgetitem48->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem48->setFont(font2);
        distanceTable->setItem(5, 0, __qtablewidgetitem48);
        QTableWidgetItem *__qtablewidgetitem49 = new QTableWidgetItem();
        __qtablewidgetitem49->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem49->setFont(font2);
        distanceTable->setItem(5, 1, __qtablewidgetitem49);
        QTableWidgetItem *__qtablewidgetitem50 = new QTableWidgetItem();
        __qtablewidgetitem50->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem50->setFont(font2);
        distanceTable->setItem(5, 2, __qtablewidgetitem50);
        QTableWidgetItem *__qtablewidgetitem51 = new QTableWidgetItem();
        __qtablewidgetitem51->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem51->setFont(font2);
        distanceTable->setItem(5, 3, __qtablewidgetitem51);
        QTableWidgetItem *__qtablewidgetitem52 = new QTableWidgetItem();
        __qtablewidgetitem52->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem52->setFont(font2);
        distanceTable->setItem(5, 4, __qtablewidgetitem52);
        QTableWidgetItem *__qtablewidgetitem53 = new QTableWidgetItem();
        __qtablewidgetitem53->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem53->setFont(font2);
        distanceTable->setItem(5, 5, __qtablewidgetitem53);
        QTableWidgetItem *__qtablewidgetitem54 = new QTableWidgetItem();
        __qtablewidgetitem54->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem54->setFont(font2);
        distanceTable->setItem(5, 6, __qtablewidgetitem54);
        QTableWidgetItem *__qtablewidgetitem55 = new QTableWidgetItem();
        __qtablewidgetitem55->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem55->setFont(font2);
        distanceTable->setItem(5, 7, __qtablewidgetitem55);
        QTableWidgetItem *__qtablewidgetitem56 = new QTableWidgetItem();
        __qtablewidgetitem56->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem56->setFont(font2);
        distanceTable->setItem(6, 0, __qtablewidgetitem56);
        QTableWidgetItem *__qtablewidgetitem57 = new QTableWidgetItem();
        __qtablewidgetitem57->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem57->setFont(font2);
        distanceTable->setItem(6, 1, __qtablewidgetitem57);
        QTableWidgetItem *__qtablewidgetitem58 = new QTableWidgetItem();
        __qtablewidgetitem58->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem58->setFont(font2);
        distanceTable->setItem(6, 2, __qtablewidgetitem58);
        QTableWidgetItem *__qtablewidgetitem59 = new QTableWidgetItem();
        __qtablewidgetitem59->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem59->setFont(font2);
        distanceTable->setItem(6, 3, __qtablewidgetitem59);
        QTableWidgetItem *__qtablewidgetitem60 = new QTableWidgetItem();
        __qtablewidgetitem60->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem60->setFont(font2);
        distanceTable->setItem(6, 4, __qtablewidgetitem60);
        QTableWidgetItem *__qtablewidgetitem61 = new QTableWidgetItem();
        __qtablewidgetitem61->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem61->setFont(font2);
        distanceTable->setItem(6, 5, __qtablewidgetitem61);
        QTableWidgetItem *__qtablewidgetitem62 = new QTableWidgetItem();
        __qtablewidgetitem62->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem62->setFont(font2);
        distanceTable->setItem(6, 6, __qtablewidgetitem62);
        QTableWidgetItem *__qtablewidgetitem63 = new QTableWidgetItem();
        __qtablewidgetitem63->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem63->setFont(font2);
        distanceTable->setItem(6, 7, __qtablewidgetitem63);
        QTableWidgetItem *__qtablewidgetitem64 = new QTableWidgetItem();
        __qtablewidgetitem64->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem64->setFont(font2);
        distanceTable->setItem(7, 0, __qtablewidgetitem64);
        QTableWidgetItem *__qtablewidgetitem65 = new QTableWidgetItem();
        __qtablewidgetitem65->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem65->setFont(font2);
        distanceTable->setItem(7, 1, __qtablewidgetitem65);
        QTableWidgetItem *__qtablewidgetitem66 = new QTableWidgetItem();
        __qtablewidgetitem66->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem66->setFont(font2);
        distanceTable->setItem(7, 2, __qtablewidgetitem66);
        QTableWidgetItem *__qtablewidgetitem67 = new QTableWidgetItem();
        __qtablewidgetitem67->setTextAlignment(Qt::AlignCenter);
        __qtablewidgetitem67->setFont(font2);
        distanceTable->setItem(7, 3, __qtablewidgetitem67);
        QTableWidgetItem *__qtablewidgetitem68 = new QTableWidgetItem();
        __qtablewidgetitem68->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem68->setFont(font2);
        distanceTable->setItem(7, 4, __qtablewidgetitem68);
        QTableWidgetItem *__qtablewidgetitem69 = new QTableWidgetItem();
        __qtablewidgetitem69->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem69->setFont(font2);
        distanceTable->setItem(7, 5, __qtablewidgetitem69);
        QTableWidgetItem *__qtablewidgetitem70 = new QTableWidgetItem();
        __qtablewidgetitem70->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem70->setFont(font2);
        distanceTable->setItem(7, 6, __qtablewidgetitem70);
        QTableWidgetItem *__qtablewidgetitem71 = new QTableWidgetItem();
        __qtablewidgetitem71->setTextAlignment(Qt::AlignTrailing|Qt::AlignVCenter);
        __qtablewidgetitem71->setFont(font2);
        distanceTable->setItem(7, 7, __qtablewidgetitem71);
        distanceTable->setObjectName(QStringLiteral("distanceTable"));
        QSizePolicy sizePolicy1(QSizePolicy::Ignored, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(10);
        sizePolicy1.setVerticalStretch(10);
        sizePolicy1.setHeightForWidth(distanceTable->sizePolicy().hasHeightForWidth());
        distanceTable->setSizePolicy(sizePolicy1);
        distanceTable->setMinimumSize(QSize(400, 90));
        QFont font3;
        distanceTable->setFont(font3);
        distanceTable->setAutoFillBackground(true);
        distanceTable->setStyleSheet(QStringLiteral("font-size: 14px;"));
        distanceTable->setFrameShape(QFrame::NoFrame);
        distanceTable->setFrameShadow(QFrame::Plain);
        distanceTable->setLineWidth(1);
        distanceTable->setMidLineWidth(2);
        distanceTable->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        distanceTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        distanceTable->setAutoScroll(true);
        distanceTable->setTabKeyNavigation(false);
        distanceTable->setShowGrid(true);
        distanceTable->setGridStyle(Qt::SolidLine);
        distanceTable->setSortingEnabled(false);
        distanceTable->setWordWrap(false);
        distanceTable->setCornerButtonEnabled(false);
        distanceTable->setRowCount(8);
        distanceTable->setColumnCount(8);
        distanceTable->horizontalHeader()->setVisible(true);
        distanceTable->horizontalHeader()->setCascadingSectionResizes(true);
        distanceTable->horizontalHeader()->setDefaultSectionSize(85);
        distanceTable->horizontalHeader()->setHighlightSections(false);
        distanceTable->horizontalHeader()->setMinimumSectionSize(40);
        distanceTable->horizontalHeader()->setProperty("showSortIndicator", QVariant(false));
        distanceTable->horizontalHeader()->setStretchLastSection(true);
        distanceTable->verticalHeader()->setVisible(false);
        distanceTable->verticalHeader()->setCascadingSectionResizes(false);
        distanceTable->verticalHeader()->setDefaultSectionSize(21);
        distanceTable->verticalHeader()->setHighlightSections(true);
        distanceTable->verticalHeader()->setProperty("showSortIndicator", QVariant(false));
        distanceTable->verticalHeader()->setStretchLastSection(false);

        verticalLayout->addWidget(distanceTable);


        retranslateUi(AWLTableView);

        QMetaObject::connectSlotsByName(AWLTableView);
    } // setupUi

    void retranslateUi(QFrame *AWLTableView)
    {
        AWLTableView->setWindowTitle(QApplication::translate("AWLTableView", "TableView", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem = distanceTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("AWLTableView", "Rcv.", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem1 = distanceTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("AWLTableView", "Ch.", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem2 = distanceTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("AWLTableView", "Det.", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem3 = distanceTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("AWLTableView", "Track", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem4 = distanceTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QApplication::translate("AWLTableView", "Dist.", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem5 = distanceTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QApplication::translate("AWLTableView", "Velo.", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem6 = distanceTable->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QApplication::translate("AWLTableView", "Int.", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem7 = distanceTable->horizontalHeaderItem(7);
        ___qtablewidgetitem7->setText(QApplication::translate("AWLTableView", "Level", Q_NULLPTR));

        const bool __sortingEnabled = distanceTable->isSortingEnabled();
        distanceTable->setSortingEnabled(false);
        distanceTable->setSortingEnabled(__sortingEnabled);

    } // retranslateUi

};

namespace Ui {
    class AWLTableView: public Ui_AWLTableView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TABLEVIEW_H
