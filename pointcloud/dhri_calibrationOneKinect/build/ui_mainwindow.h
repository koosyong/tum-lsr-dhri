/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Feb 4 17:53:55 2013
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
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionStereo_Auto;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QTabWidget *tabWidget;
    QWidget *mono;
    QWidget *stereo_auto;
    QGridLayout *gridLayout_5;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout;
    QLabel *label_ir1;
    QGroupBox *groupBox_5;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_ir2;
    QPushButton *pushButton;
    QWidget *stereo_manual;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLabel *label;
    QSlider *horizontalSlider_camera1_yaw;
    QLabel *label_2;
    QSlider *horizontalSlider_camera1_pitch;
    QDoubleSpinBox *doubleSpinBox_camera1_pitch;
    QLabel *label_3;
    QSlider *horizontalSlider_camera1_roll;
    QDoubleSpinBox *doubleSpinBox_camera1_roll;
    QDoubleSpinBox *doubleSpinBox_camera1_yaw;
    QLabel *label_7;
    QSlider *horizontalSlider_camera1_x;
    QLabel *label_8;
    QLabel *label_9;
    QSlider *horizontalSlider_camera1_y;
    QSlider *horizontalSlider_camera1_z;
    QDoubleSpinBox *doubleSpinBox_camera1_x;
    QDoubleSpinBox *doubleSpinBox_camera1_y;
    QDoubleSpinBox *doubleSpinBox_camera1_z;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QLabel *label_4;
    QSlider *horizontalSlider_camera2_yaw;
    QDoubleSpinBox *doubleSpinBox_camera2_yaw;
    QLabel *label_5;
    QSlider *horizontalSlider_camera2_pitch;
    QDoubleSpinBox *doubleSpinBox_camera2_pitch;
    QLabel *label_6;
    QSlider *horizontalSlider_camera2_roll;
    QDoubleSpinBox *doubleSpinBox_camera2_roll;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QSlider *horizontalSlider_camera2_x;
    QSlider *horizontalSlider_camera2_y;
    QSlider *horizontalSlider_camera2_z;
    QDoubleSpinBox *doubleSpinBox_camera2_x;
    QDoubleSpinBox *doubleSpinBox_camera2_y;
    QDoubleSpinBox *doubleSpinBox_camera2_z;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(854, 606);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionStereo_Auto = new QAction(MainWindow);
        actionStereo_Auto->setObjectName(QString::fromUtf8("actionStereo_Auto"));
        actionStereo_Auto->setCheckable(true);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        mono = new QWidget();
        mono->setObjectName(QString::fromUtf8("mono"));
        tabWidget->addTab(mono, QString());
        stereo_auto = new QWidget();
        stereo_auto->setObjectName(QString::fromUtf8("stereo_auto"));
        gridLayout_5 = new QGridLayout(stereo_auto);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        groupBox_4 = new QGroupBox(stereo_auto);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        gridLayout = new QGridLayout(groupBox_4);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_ir1 = new QLabel(groupBox_4);
        label_ir1->setObjectName(QString::fromUtf8("label_ir1"));

        gridLayout->addWidget(label_ir1, 1, 0, 1, 1);


        gridLayout_5->addWidget(groupBox_4, 1, 0, 1, 1);

        groupBox_5 = new QGroupBox(stereo_auto);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        horizontalLayout_4 = new QHBoxLayout(groupBox_5);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_ir2 = new QLabel(groupBox_5);
        label_ir2->setObjectName(QString::fromUtf8("label_ir2"));

        horizontalLayout_4->addWidget(label_ir2);


        gridLayout_5->addWidget(groupBox_5, 1, 1, 1, 1);

        pushButton = new QPushButton(stereo_auto);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout_5->addWidget(pushButton, 0, 0, 1, 2);

        tabWidget->addTab(stereo_auto, QString());
        stereo_manual = new QWidget();
        stereo_manual->setObjectName(QString::fromUtf8("stereo_manual"));
        verticalLayout = new QVBoxLayout(stereo_manual);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(stereo_manual);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setFlat(false);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 3, 0, 1, 1);

        horizontalSlider_camera1_yaw = new QSlider(groupBox);
        horizontalSlider_camera1_yaw->setObjectName(QString::fromUtf8("horizontalSlider_camera1_yaw"));
        horizontalSlider_camera1_yaw->setMinimum(-18000);
        horizontalSlider_camera1_yaw->setMaximum(18000);
        horizontalSlider_camera1_yaw->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera1_yaw, 3, 3, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 4, 0, 1, 1);

        horizontalSlider_camera1_pitch = new QSlider(groupBox);
        horizontalSlider_camera1_pitch->setObjectName(QString::fromUtf8("horizontalSlider_camera1_pitch"));
        horizontalSlider_camera1_pitch->setMinimum(-18000);
        horizontalSlider_camera1_pitch->setMaximum(18000);
        horizontalSlider_camera1_pitch->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera1_pitch, 4, 3, 1, 1);

        doubleSpinBox_camera1_pitch = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera1_pitch->setObjectName(QString::fromUtf8("doubleSpinBox_camera1_pitch"));
        doubleSpinBox_camera1_pitch->setMinimum(-180);
        doubleSpinBox_camera1_pitch->setMaximum(180);
        doubleSpinBox_camera1_pitch->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_camera1_pitch, 4, 4, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 5, 0, 1, 1);

        horizontalSlider_camera1_roll = new QSlider(groupBox);
        horizontalSlider_camera1_roll->setObjectName(QString::fromUtf8("horizontalSlider_camera1_roll"));
        horizontalSlider_camera1_roll->setMinimum(-18000);
        horizontalSlider_camera1_roll->setMaximum(18000);
        horizontalSlider_camera1_roll->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera1_roll, 5, 3, 1, 1);

        doubleSpinBox_camera1_roll = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera1_roll->setObjectName(QString::fromUtf8("doubleSpinBox_camera1_roll"));
        doubleSpinBox_camera1_roll->setMinimum(-180);
        doubleSpinBox_camera1_roll->setMaximum(180);
        doubleSpinBox_camera1_roll->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_camera1_roll, 5, 4, 1, 1);

        doubleSpinBox_camera1_yaw = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera1_yaw->setObjectName(QString::fromUtf8("doubleSpinBox_camera1_yaw"));
        doubleSpinBox_camera1_yaw->setMinimum(-180);
        doubleSpinBox_camera1_yaw->setMaximum(180);
        doubleSpinBox_camera1_yaw->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_camera1_yaw, 3, 4, 1, 1);

        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 0, 0, 1, 1);

        horizontalSlider_camera1_x = new QSlider(groupBox);
        horizontalSlider_camera1_x->setObjectName(QString::fromUtf8("horizontalSlider_camera1_x"));
        horizontalSlider_camera1_x->setMinimum(-5000);
        horizontalSlider_camera1_x->setMaximum(5000);
        horizontalSlider_camera1_x->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera1_x, 0, 2, 1, 2);

        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 1, 0, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_2->addWidget(label_9, 2, 0, 1, 1);

        horizontalSlider_camera1_y = new QSlider(groupBox);
        horizontalSlider_camera1_y->setObjectName(QString::fromUtf8("horizontalSlider_camera1_y"));
        horizontalSlider_camera1_y->setMinimum(-5000);
        horizontalSlider_camera1_y->setMaximum(5000);
        horizontalSlider_camera1_y->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera1_y, 1, 2, 1, 2);

        horizontalSlider_camera1_z = new QSlider(groupBox);
        horizontalSlider_camera1_z->setObjectName(QString::fromUtf8("horizontalSlider_camera1_z"));
        horizontalSlider_camera1_z->setMaximum(5000);
        horizontalSlider_camera1_z->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera1_z, 2, 2, 1, 2);

        doubleSpinBox_camera1_x = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera1_x->setObjectName(QString::fromUtf8("doubleSpinBox_camera1_x"));
        doubleSpinBox_camera1_x->setMinimum(-5);
        doubleSpinBox_camera1_x->setMaximum(5);

        gridLayout_2->addWidget(doubleSpinBox_camera1_x, 0, 4, 1, 1);

        doubleSpinBox_camera1_y = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera1_y->setObjectName(QString::fromUtf8("doubleSpinBox_camera1_y"));
        doubleSpinBox_camera1_y->setMinimum(-5);
        doubleSpinBox_camera1_y->setMaximum(5);

        gridLayout_2->addWidget(doubleSpinBox_camera1_y, 1, 4, 1, 1);

        doubleSpinBox_camera1_z = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera1_z->setObjectName(QString::fromUtf8("doubleSpinBox_camera1_z"));
        doubleSpinBox_camera1_z->setMaximum(5);

        gridLayout_2->addWidget(doubleSpinBox_camera1_z, 2, 4, 1, 1);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(stereo_manual);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setFlat(false);
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_3->addWidget(label_4, 3, 0, 1, 1);

        horizontalSlider_camera2_yaw = new QSlider(groupBox_2);
        horizontalSlider_camera2_yaw->setObjectName(QString::fromUtf8("horizontalSlider_camera2_yaw"));
        horizontalSlider_camera2_yaw->setMinimum(-18000);
        horizontalSlider_camera2_yaw->setMaximum(18000);
        horizontalSlider_camera2_yaw->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider_camera2_yaw, 3, 3, 1, 1);

        doubleSpinBox_camera2_yaw = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_camera2_yaw->setObjectName(QString::fromUtf8("doubleSpinBox_camera2_yaw"));
        doubleSpinBox_camera2_yaw->setMinimum(-180);
        doubleSpinBox_camera2_yaw->setMaximum(180);
        doubleSpinBox_camera2_yaw->setSingleStep(0.01);

        gridLayout_3->addWidget(doubleSpinBox_camera2_yaw, 3, 4, 1, 1);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_3->addWidget(label_5, 4, 0, 1, 1);

        horizontalSlider_camera2_pitch = new QSlider(groupBox_2);
        horizontalSlider_camera2_pitch->setObjectName(QString::fromUtf8("horizontalSlider_camera2_pitch"));
        horizontalSlider_camera2_pitch->setMinimum(-18000);
        horizontalSlider_camera2_pitch->setMaximum(18000);
        horizontalSlider_camera2_pitch->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider_camera2_pitch, 4, 3, 1, 1);

        doubleSpinBox_camera2_pitch = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_camera2_pitch->setObjectName(QString::fromUtf8("doubleSpinBox_camera2_pitch"));
        doubleSpinBox_camera2_pitch->setMinimum(-180);
        doubleSpinBox_camera2_pitch->setMaximum(180);
        doubleSpinBox_camera2_pitch->setSingleStep(0.01);

        gridLayout_3->addWidget(doubleSpinBox_camera2_pitch, 4, 4, 1, 1);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_3->addWidget(label_6, 5, 0, 1, 1);

        horizontalSlider_camera2_roll = new QSlider(groupBox_2);
        horizontalSlider_camera2_roll->setObjectName(QString::fromUtf8("horizontalSlider_camera2_roll"));
        horizontalSlider_camera2_roll->setMinimum(-18000);
        horizontalSlider_camera2_roll->setMaximum(18000);
        horizontalSlider_camera2_roll->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider_camera2_roll, 5, 3, 1, 1);

        doubleSpinBox_camera2_roll = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_camera2_roll->setObjectName(QString::fromUtf8("doubleSpinBox_camera2_roll"));
        doubleSpinBox_camera2_roll->setMinimum(-180);
        doubleSpinBox_camera2_roll->setMaximum(180);
        doubleSpinBox_camera2_roll->setSingleStep(0.01);

        gridLayout_3->addWidget(doubleSpinBox_camera2_roll, 5, 4, 1, 1);

        label_10 = new QLabel(groupBox_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_3->addWidget(label_10, 0, 0, 1, 1);

        label_11 = new QLabel(groupBox_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_3->addWidget(label_11, 1, 0, 1, 1);

        label_12 = new QLabel(groupBox_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_3->addWidget(label_12, 2, 0, 1, 1);

        horizontalSlider_camera2_x = new QSlider(groupBox_2);
        horizontalSlider_camera2_x->setObjectName(QString::fromUtf8("horizontalSlider_camera2_x"));
        horizontalSlider_camera2_x->setMinimum(-5000);
        horizontalSlider_camera2_x->setMaximum(5000);
        horizontalSlider_camera2_x->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider_camera2_x, 0, 2, 1, 2);

        horizontalSlider_camera2_y = new QSlider(groupBox_2);
        horizontalSlider_camera2_y->setObjectName(QString::fromUtf8("horizontalSlider_camera2_y"));
        horizontalSlider_camera2_y->setMinimum(-5000);
        horizontalSlider_camera2_y->setMaximum(5000);
        horizontalSlider_camera2_y->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider_camera2_y, 1, 2, 1, 2);

        horizontalSlider_camera2_z = new QSlider(groupBox_2);
        horizontalSlider_camera2_z->setObjectName(QString::fromUtf8("horizontalSlider_camera2_z"));
        horizontalSlider_camera2_z->setMaximum(5000);
        horizontalSlider_camera2_z->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider_camera2_z, 2, 2, 1, 2);

        doubleSpinBox_camera2_x = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_camera2_x->setObjectName(QString::fromUtf8("doubleSpinBox_camera2_x"));
        doubleSpinBox_camera2_x->setMinimum(-5);
        doubleSpinBox_camera2_x->setMaximum(5);

        gridLayout_3->addWidget(doubleSpinBox_camera2_x, 0, 4, 1, 1);

        doubleSpinBox_camera2_y = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_camera2_y->setObjectName(QString::fromUtf8("doubleSpinBox_camera2_y"));
        doubleSpinBox_camera2_y->setMinimum(-5);
        doubleSpinBox_camera2_y->setMaximum(5);

        gridLayout_3->addWidget(doubleSpinBox_camera2_y, 1, 4, 1, 1);

        doubleSpinBox_camera2_z = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_camera2_z->setObjectName(QString::fromUtf8("doubleSpinBox_camera2_z"));
        doubleSpinBox_camera2_z->setMaximum(5);

        gridLayout_3->addWidget(doubleSpinBox_camera2_z, 2, 4, 1, 1);


        verticalLayout->addWidget(groupBox_2);

        tabWidget->addTab(stereo_manual, QString());

        horizontalLayout->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 854, 23));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);
        toolBar->addAction(actionStereo_Auto);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        actionStereo_Auto->setText(QApplication::translate("MainWindow", "Stereo Auto", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(mono), QApplication::translate("MainWindow", "Mono", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Camera1", 0, QApplication::UnicodeUTF8));
        label_ir1->setText(QString());
        groupBox_5->setTitle(QApplication::translate("MainWindow", "Camera2", 0, QApplication::UnicodeUTF8));
        label_ir2->setText(QString());
        pushButton->setText(QApplication::translate("MainWindow", "start", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(stereo_auto), QApplication::translate("MainWindow", "Stereo_Auto", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Camera1", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "yaw", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Pitch", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Roll", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "x", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "z", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Camera2", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "yaw", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Pitch", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Roll", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "x", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "z", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(stereo_manual), QApplication::translate("MainWindow", "Stereo_Manual", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
