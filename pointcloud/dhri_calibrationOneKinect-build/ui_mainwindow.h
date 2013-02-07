/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Feb 7 17:04:10 2013
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
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionSave;
    QAction *actionCalibration;
    QAction *actionSaveAs;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QLineEdit *lineEdit_yaw;
    QLabel *label_4;
    QLineEdit *lineEdit_roll;
    QLineEdit *lineEdit_y;
    QLineEdit *lineEdit_z;
    QLabel *label_3;
    QLabel *label;
    QLineEdit *lineEdit_pitch;
    QLabel *label_6;
    QLabel *label_5;
    QLabel *label_2;
    QLineEdit *lineEdit_x;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(360, 190);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionCalibration = new QAction(MainWindow);
        actionCalibration->setObjectName(QString::fromUtf8("actionCalibration"));
        actionCalibration->setCheckable(true);
        actionSaveAs = new QAction(MainWindow);
        actionSaveAs->setObjectName(QString::fromUtf8("actionSaveAs"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        lineEdit_yaw = new QLineEdit(groupBox);
        lineEdit_yaw->setObjectName(QString::fromUtf8("lineEdit_yaw"));
        lineEdit_yaw->setEnabled(true);
        lineEdit_yaw->setReadOnly(true);

        gridLayout->addWidget(lineEdit_yaw, 1, 1, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        lineEdit_roll = new QLineEdit(groupBox);
        lineEdit_roll->setObjectName(QString::fromUtf8("lineEdit_roll"));
        lineEdit_roll->setEnabled(true);
        lineEdit_roll->setReadOnly(true);

        gridLayout->addWidget(lineEdit_roll, 1, 5, 1, 1);

        lineEdit_y = new QLineEdit(groupBox);
        lineEdit_y->setObjectName(QString::fromUtf8("lineEdit_y"));
        lineEdit_y->setEnabled(true);
        lineEdit_y->setReadOnly(true);

        gridLayout->addWidget(lineEdit_y, 0, 3, 1, 1);

        lineEdit_z = new QLineEdit(groupBox);
        lineEdit_z->setObjectName(QString::fromUtf8("lineEdit_z"));
        lineEdit_z->setEnabled(true);
        lineEdit_z->setReadOnly(true);

        gridLayout->addWidget(lineEdit_z, 0, 5, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 0, 4, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        lineEdit_pitch = new QLineEdit(groupBox);
        lineEdit_pitch->setObjectName(QString::fromUtf8("lineEdit_pitch"));
        lineEdit_pitch->setEnabled(true);
        lineEdit_pitch->setReadOnly(true);

        gridLayout->addWidget(lineEdit_pitch, 1, 3, 1, 1);

        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 1, 4, 1, 1);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 1, 2, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 0, 2, 1, 1);

        lineEdit_x = new QLineEdit(groupBox);
        lineEdit_x->setObjectName(QString::fromUtf8("lineEdit_x"));
        lineEdit_x->setEnabled(true);
        lineEdit_x->setReadOnly(true);

        gridLayout->addWidget(lineEdit_x, 0, 1, 1, 1);


        horizontalLayout->addWidget(groupBox);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 360, 25));
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
        menuFile->addAction(actionSave);
        toolBar->addAction(actionSave);
        toolBar->addAction(actionSaveAs);
        toolBar->addAction(actionCalibration);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Calibration one kinect sensor", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        actionCalibration->setText(QApplication::translate("MainWindow", "Calibration", 0, QApplication::UnicodeUTF8));
        actionSaveAs->setText(QApplication::translate("MainWindow", "SaveAs", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Camera parameters", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "yaw :", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "z :", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "x :", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "roll :", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "pitcch :", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "y :", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
