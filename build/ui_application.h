/********************************************************************************
** Form generated from reading UI file 'application.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_APPLICATION_H
#define UI_APPLICATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_RoomScanner
{
public:
    QAction *actionLoad_Point_Cloud;
    QAction *actionShow_keypoints;
    QAction *actionShow_captured_frames;
    QAction *actionClear;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_3;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_3;
    QVTKWidget *qvtkWidget;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_4;
    QLCDNumber *lcdNumber_p;
    QVBoxLayout *verticalLayout;
    QSlider *horizontalSlider_p;
    QCheckBox *checkBox;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_reset;
    QPushButton *pushButton_save;
    QPushButton *pushButton_reg;
    QPushButton *pushButton_poly;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_2;
    QVTKWidget *qvtkWidget_2;
    QMenuBar *menuBar;
    QMenu *menuMenu;

    void setupUi(QMainWindow *RoomScanner)
    {
        if (RoomScanner->objectName().isEmpty())
            RoomScanner->setObjectName(QStringLiteral("RoomScanner"));
        RoomScanner->resize(782, 717);
        RoomScanner->setMinimumSize(QSize(0, 0));
        RoomScanner->setMaximumSize(QSize(5000, 5000));
        actionLoad_Point_Cloud = new QAction(RoomScanner);
        actionLoad_Point_Cloud->setObjectName(QStringLiteral("actionLoad_Point_Cloud"));
        actionShow_keypoints = new QAction(RoomScanner);
        actionShow_keypoints->setObjectName(QStringLiteral("actionShow_keypoints"));
        actionShow_keypoints->setCheckable(true);
        actionShow_keypoints->setChecked(false);
        actionShow_captured_frames = new QAction(RoomScanner);
        actionShow_captured_frames->setObjectName(QStringLiteral("actionShow_captured_frames"));
        actionShow_captured_frames->setCheckable(true);
        actionClear = new QAction(RoomScanner);
        actionClear->setObjectName(QStringLiteral("actionClear"));
        centralwidget = new QWidget(RoomScanner);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout_3 = new QHBoxLayout(centralwidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy);
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        verticalLayout_3 = new QVBoxLayout(tab);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        qvtkWidget = new QVTKWidget(tab);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(1);
        sizePolicy1.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy1);
        qvtkWidget->setMinimumSize(QSize(640, 480));
        qvtkWidget->setSizeIncrement(QSize(0, 0));

        verticalLayout_3->addWidget(qvtkWidget);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(20, -1, -1, 0);
        label_4 = new QLabel(tab);
        label_4->setObjectName(QStringLiteral("label_4"));
        QFont font;
        font.setPointSize(16);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        label_4->setFont(font);

        horizontalLayout_2->addWidget(label_4);

        lcdNumber_p = new QLCDNumber(tab);
        lcdNumber_p->setObjectName(QStringLiteral("lcdNumber_p"));
        lcdNumber_p->setMaximumSize(QSize(50, 50));
        lcdNumber_p->setLayoutDirection(Qt::LeftToRight);
        lcdNumber_p->setDigitCount(1);
        lcdNumber_p->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_p->setProperty("intValue", QVariant(2));

        horizontalLayout_2->addWidget(lcdNumber_p);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetMaximumSize);
        verticalLayout->setContentsMargins(50, 25, 20, -1);
        horizontalSlider_p = new QSlider(tab);
        horizontalSlider_p->setObjectName(QStringLiteral("horizontalSlider_p"));
        horizontalSlider_p->setMinimumSize(QSize(200, 0));
        horizontalSlider_p->setMinimum(1);
        horizontalSlider_p->setMaximum(6);
        horizontalSlider_p->setValue(2);
        horizontalSlider_p->setOrientation(Qt::Horizontal);
        horizontalSlider_p->setTickPosition(QSlider::TicksBelow);

        verticalLayout->addWidget(horizontalSlider_p);

        checkBox = new QCheckBox(tab);
        checkBox->setObjectName(QStringLiteral("checkBox"));

        verticalLayout->addWidget(checkBox);


        horizontalLayout_2->addLayout(verticalLayout);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetMaximumSize);
        horizontalLayout->setContentsMargins(-1, -1, -1, 0);
        pushButton_reset = new QPushButton(tab);
        pushButton_reset->setObjectName(QStringLiteral("pushButton_reset"));

        horizontalLayout->addWidget(pushButton_reset);

        pushButton_save = new QPushButton(tab);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));

        horizontalLayout->addWidget(pushButton_save);

        pushButton_reg = new QPushButton(tab);
        pushButton_reg->setObjectName(QStringLiteral("pushButton_reg"));

        horizontalLayout->addWidget(pushButton_reg);

        pushButton_poly = new QPushButton(tab);
        pushButton_poly->setObjectName(QStringLiteral("pushButton_poly"));

        horizontalLayout->addWidget(pushButton_poly);


        verticalLayout_3->addLayout(horizontalLayout);

        tabWidget->addTab(tab, QString());
        qvtkWidget->raise();
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        verticalLayout_2 = new QVBoxLayout(tab_2);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        qvtkWidget_2 = new QVTKWidget(tab_2);
        qvtkWidget_2->setObjectName(QStringLiteral("qvtkWidget_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(1);
        sizePolicy2.setVerticalStretch(1);
        sizePolicy2.setHeightForWidth(qvtkWidget_2->sizePolicy().hasHeightForWidth());
        qvtkWidget_2->setSizePolicy(sizePolicy2);
        qvtkWidget_2->setMinimumSize(QSize(640, 480));
        qvtkWidget_2->setSizeIncrement(QSize(0, 0));

        verticalLayout_2->addWidget(qvtkWidget_2);

        tabWidget->addTab(tab_2, QString());

        horizontalLayout_3->addWidget(tabWidget);

        RoomScanner->setCentralWidget(centralwidget);
        menuBar = new QMenuBar(RoomScanner);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 782, 19));
        menuMenu = new QMenu(menuBar);
        menuMenu->setObjectName(QStringLiteral("menuMenu"));
        RoomScanner->setMenuBar(menuBar);
        QWidget::setTabOrder(pushButton_poly, checkBox);
        QWidget::setTabOrder(checkBox, pushButton_reset);
        QWidget::setTabOrder(pushButton_reset, pushButton_save);
        QWidget::setTabOrder(pushButton_save, tabWidget);
        QWidget::setTabOrder(tabWidget, horizontalSlider_p);

        menuBar->addAction(menuMenu->menuAction());
        menuMenu->addAction(actionLoad_Point_Cloud);
        menuMenu->addAction(actionShow_keypoints);
        menuMenu->addAction(actionShow_captured_frames);
        menuMenu->addAction(actionClear);

        retranslateUi(RoomScanner);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(RoomScanner);
    } // setupUi

    void retranslateUi(QMainWindow *RoomScanner)
    {
        RoomScanner->setWindowTitle(QApplication::translate("RoomScanner", "PCLViewer", 0));
        actionLoad_Point_Cloud->setText(QApplication::translate("RoomScanner", "Load Point Cloud", 0));
        actionShow_keypoints->setText(QApplication::translate("RoomScanner", "Show keypoints", 0));
        actionShow_captured_frames->setText(QApplication::translate("RoomScanner", "Show last captured frame", 0));
        actionClear->setText(QApplication::translate("RoomScanner", "Clear", 0));
        label_4->setText(QApplication::translate("RoomScanner", "Point size", 0));
        checkBox->setText(QApplication::translate("RoomScanner", "Show Coordinate System", 0));
        pushButton_reset->setText(QApplication::translate("RoomScanner", "Reset Camera", 0));
        pushButton_save->setText(QApplication::translate("RoomScanner", "Save Frame", 0));
        pushButton_reg->setText(QApplication::translate("RoomScanner", "Registrate", 0));
        pushButton_poly->setText(QApplication::translate("RoomScanner", "Polygonate", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("RoomScanner", "Point Cloud", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("RoomScanner", "Polygonated mesh", 0));
        menuMenu->setTitle(QApplication::translate("RoomScanner", "Menu", 0));
    } // retranslateUi

};

namespace Ui {
    class RoomScanner: public Ui_RoomScanner {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_APPLICATION_H
