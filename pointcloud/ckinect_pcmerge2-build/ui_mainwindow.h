/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Feb 15 15:02:26 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_6;
    QSlider *horizontalSlider_ws_origin_x;
    QDoubleSpinBox *doubleSpinBox_ws_origin_x;
    QSlider *horizontalSlider_ws_origin_y;
    QDoubleSpinBox *doubleSpinBox_ws_origin_y;
    QSlider *horizontalSlider_ws_origin_z;
    QDoubleSpinBox *doubleSpinBox_ws_origin_z;
    QLabel *label_15;
    QLabel *label_14;
    QLabel *label_13;
    QGroupBox *groupBox_6;
    QGridLayout *gridLayout_7;
    QLabel *label_16;
    QSlider *horizontalSlider_ws_size_x;
    QDoubleSpinBox *doubleSpinBox_ws_size_x;
    QLabel *label_17;
    QSlider *horizontalSlider_ws_size_y;
    QDoubleSpinBox *doubleSpinBox_ws_size_y;
    QLabel *label_18;
    QSlider *horizontalSlider_ws_size_z;
    QDoubleSpinBox *doubleSpinBox_ws_size_z;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(823, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox_3 = new QGroupBox(centralwidget);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_6 = new QGridLayout(groupBox_3);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        horizontalSlider_ws_origin_x = new QSlider(groupBox_3);
        horizontalSlider_ws_origin_x->setObjectName(QString::fromUtf8("horizontalSlider_ws_origin_x"));
        horizontalSlider_ws_origin_x->setMinimum(-500);
        horizontalSlider_ws_origin_x->setMaximum(500);
        horizontalSlider_ws_origin_x->setOrientation(Qt::Horizontal);

        gridLayout_6->addWidget(horizontalSlider_ws_origin_x, 0, 1, 1, 2);

        doubleSpinBox_ws_origin_x = new QDoubleSpinBox(groupBox_3);
        doubleSpinBox_ws_origin_x->setObjectName(QString::fromUtf8("doubleSpinBox_ws_origin_x"));
        doubleSpinBox_ws_origin_x->setMinimum(-5);
        doubleSpinBox_ws_origin_x->setMaximum(5);
        doubleSpinBox_ws_origin_x->setSingleStep(0.01);

        gridLayout_6->addWidget(doubleSpinBox_ws_origin_x, 0, 3, 1, 1);

        horizontalSlider_ws_origin_y = new QSlider(groupBox_3);
        horizontalSlider_ws_origin_y->setObjectName(QString::fromUtf8("horizontalSlider_ws_origin_y"));
        horizontalSlider_ws_origin_y->setMinimum(-500);
        horizontalSlider_ws_origin_y->setMaximum(500);
        horizontalSlider_ws_origin_y->setOrientation(Qt::Horizontal);

        gridLayout_6->addWidget(horizontalSlider_ws_origin_y, 1, 1, 1, 2);

        doubleSpinBox_ws_origin_y = new QDoubleSpinBox(groupBox_3);
        doubleSpinBox_ws_origin_y->setObjectName(QString::fromUtf8("doubleSpinBox_ws_origin_y"));
        doubleSpinBox_ws_origin_y->setMinimum(-5);
        doubleSpinBox_ws_origin_y->setMaximum(5);
        doubleSpinBox_ws_origin_y->setSingleStep(0.01);

        gridLayout_6->addWidget(doubleSpinBox_ws_origin_y, 1, 3, 1, 1);

        horizontalSlider_ws_origin_z = new QSlider(groupBox_3);
        horizontalSlider_ws_origin_z->setObjectName(QString::fromUtf8("horizontalSlider_ws_origin_z"));
        horizontalSlider_ws_origin_z->setMinimum(-500);
        horizontalSlider_ws_origin_z->setMaximum(500);
        horizontalSlider_ws_origin_z->setOrientation(Qt::Horizontal);

        gridLayout_6->addWidget(horizontalSlider_ws_origin_z, 2, 1, 1, 1);

        doubleSpinBox_ws_origin_z = new QDoubleSpinBox(groupBox_3);
        doubleSpinBox_ws_origin_z->setObjectName(QString::fromUtf8("doubleSpinBox_ws_origin_z"));
        doubleSpinBox_ws_origin_z->setMinimum(-5);
        doubleSpinBox_ws_origin_z->setMaximum(5);
        doubleSpinBox_ws_origin_z->setSingleStep(0.01);

        gridLayout_6->addWidget(doubleSpinBox_ws_origin_z, 2, 3, 1, 1);

        label_15 = new QLabel(groupBox_3);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_6->addWidget(label_15, 2, 0, 1, 1);

        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_6->addWidget(label_14, 1, 0, 1, 1);

        label_13 = new QLabel(groupBox_3);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_6->addWidget(label_13, 0, 0, 1, 1);


        verticalLayout->addWidget(groupBox_3);

        groupBox_6 = new QGroupBox(centralwidget);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        gridLayout_7 = new QGridLayout(groupBox_6);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        label_16 = new QLabel(groupBox_6);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout_7->addWidget(label_16, 0, 0, 1, 1);

        horizontalSlider_ws_size_x = new QSlider(groupBox_6);
        horizontalSlider_ws_size_x->setObjectName(QString::fromUtf8("horizontalSlider_ws_size_x"));
        horizontalSlider_ws_size_x->setMaximum(500);
        horizontalSlider_ws_size_x->setOrientation(Qt::Horizontal);

        gridLayout_7->addWidget(horizontalSlider_ws_size_x, 0, 1, 1, 2);

        doubleSpinBox_ws_size_x = new QDoubleSpinBox(groupBox_6);
        doubleSpinBox_ws_size_x->setObjectName(QString::fromUtf8("doubleSpinBox_ws_size_x"));
        doubleSpinBox_ws_size_x->setMaximum(5);
        doubleSpinBox_ws_size_x->setSingleStep(0.01);

        gridLayout_7->addWidget(doubleSpinBox_ws_size_x, 0, 3, 1, 1);

        label_17 = new QLabel(groupBox_6);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout_7->addWidget(label_17, 1, 0, 1, 1);

        horizontalSlider_ws_size_y = new QSlider(groupBox_6);
        horizontalSlider_ws_size_y->setObjectName(QString::fromUtf8("horizontalSlider_ws_size_y"));
        horizontalSlider_ws_size_y->setMaximum(500);
        horizontalSlider_ws_size_y->setOrientation(Qt::Horizontal);

        gridLayout_7->addWidget(horizontalSlider_ws_size_y, 1, 1, 1, 2);

        doubleSpinBox_ws_size_y = new QDoubleSpinBox(groupBox_6);
        doubleSpinBox_ws_size_y->setObjectName(QString::fromUtf8("doubleSpinBox_ws_size_y"));
        doubleSpinBox_ws_size_y->setMaximum(5);
        doubleSpinBox_ws_size_y->setSingleStep(0.01);

        gridLayout_7->addWidget(doubleSpinBox_ws_size_y, 1, 3, 1, 1);

        label_18 = new QLabel(groupBox_6);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        gridLayout_7->addWidget(label_18, 2, 0, 1, 1);

        horizontalSlider_ws_size_z = new QSlider(groupBox_6);
        horizontalSlider_ws_size_z->setObjectName(QString::fromUtf8("horizontalSlider_ws_size_z"));
        horizontalSlider_ws_size_z->setMaximum(500);
        horizontalSlider_ws_size_z->setOrientation(Qt::Horizontal);

        gridLayout_7->addWidget(horizontalSlider_ws_size_z, 2, 1, 1, 1);

        doubleSpinBox_ws_size_z = new QDoubleSpinBox(groupBox_6);
        doubleSpinBox_ws_size_z->setObjectName(QString::fromUtf8("doubleSpinBox_ws_size_z"));
        doubleSpinBox_ws_size_z->setMaximum(5);
        doubleSpinBox_ws_size_z->setSingleStep(0.01);

        gridLayout_7->addWidget(doubleSpinBox_ws_size_z, 2, 3, 1, 1);


        verticalLayout->addWidget(groupBox_6);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 823, 23));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Origin", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "z", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "x", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("MainWindow", "Size", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindow", "x", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("MainWindow", "z", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
