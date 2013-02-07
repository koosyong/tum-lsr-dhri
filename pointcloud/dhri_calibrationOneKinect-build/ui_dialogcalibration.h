/********************************************************************************
** Form generated from reading UI file 'dialogcalibration.ui'
**
** Created: Thu Feb 7 16:49:55 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOGCALIBRATION_H
#define UI_DIALOGCALIBRATION_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DialogCalibration
{
public:
    QTabWidget *tabWidget;
    QWidget *stereo_auto;
    QGridLayout *gridLayout_5;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout;
    QLabel *label_ir;
    QPushButton *pushButton_start;
    QWidget *stereo_manual;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLabel *label;
    QSlider *horizontalSlider_camera_yaw;
    QLabel *label_2;
    QSlider *horizontalSlider_camera_pitch;
    QDoubleSpinBox *doubleSpinBox_camera_pitch;
    QLabel *label_3;
    QSlider *horizontalSlider_camera_roll;
    QDoubleSpinBox *doubleSpinBox_camera_roll;
    QDoubleSpinBox *doubleSpinBox_camera_yaw;
    QLabel *label_7;
    QSlider *horizontalSlider_camera_x;
    QLabel *label_8;
    QLabel *label_9;
    QSlider *horizontalSlider_camera_y;
    QSlider *horizontalSlider_camera_z;
    QDoubleSpinBox *doubleSpinBox_camera_x;
    QDoubleSpinBox *doubleSpinBox_camera_y;
    QDoubleSpinBox *doubleSpinBox_camera_z;

    void setupUi(QDialog *DialogCalibration)
    {
        if (DialogCalibration->objectName().isEmpty())
            DialogCalibration->setObjectName(QString::fromUtf8("DialogCalibration"));
        DialogCalibration->resize(688, 544);
        tabWidget = new QTabWidget(DialogCalibration);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 681, 538));
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setElideMode(Qt::ElideNone);
        tabWidget->setTabsClosable(false);
        tabWidget->setMovable(false);
        stereo_auto = new QWidget();
        stereo_auto->setObjectName(QString::fromUtf8("stereo_auto"));
        gridLayout_5 = new QGridLayout(stereo_auto);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        groupBox_4 = new QGroupBox(stereo_auto);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        gridLayout = new QGridLayout(groupBox_4);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_ir = new QLabel(groupBox_4);
        label_ir->setObjectName(QString::fromUtf8("label_ir"));

        gridLayout->addWidget(label_ir, 1, 0, 1, 1);


        gridLayout_5->addWidget(groupBox_4, 1, 0, 1, 1);

        pushButton_start = new QPushButton(stereo_auto);
        pushButton_start->setObjectName(QString::fromUtf8("pushButton_start"));

        gridLayout_5->addWidget(pushButton_start, 0, 0, 1, 2);

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

        horizontalSlider_camera_yaw = new QSlider(groupBox);
        horizontalSlider_camera_yaw->setObjectName(QString::fromUtf8("horizontalSlider_camera_yaw"));
        horizontalSlider_camera_yaw->setMinimum(-18000);
        horizontalSlider_camera_yaw->setMaximum(18000);
        horizontalSlider_camera_yaw->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera_yaw, 3, 3, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 4, 0, 1, 1);

        horizontalSlider_camera_pitch = new QSlider(groupBox);
        horizontalSlider_camera_pitch->setObjectName(QString::fromUtf8("horizontalSlider_camera_pitch"));
        horizontalSlider_camera_pitch->setMinimum(-18000);
        horizontalSlider_camera_pitch->setMaximum(18000);
        horizontalSlider_camera_pitch->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera_pitch, 4, 3, 1, 1);

        doubleSpinBox_camera_pitch = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera_pitch->setObjectName(QString::fromUtf8("doubleSpinBox_camera_pitch"));
        doubleSpinBox_camera_pitch->setMinimum(-180);
        doubleSpinBox_camera_pitch->setMaximum(180);
        doubleSpinBox_camera_pitch->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_camera_pitch, 4, 4, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 5, 0, 1, 1);

        horizontalSlider_camera_roll = new QSlider(groupBox);
        horizontalSlider_camera_roll->setObjectName(QString::fromUtf8("horizontalSlider_camera_roll"));
        horizontalSlider_camera_roll->setMinimum(-18000);
        horizontalSlider_camera_roll->setMaximum(18000);
        horizontalSlider_camera_roll->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera_roll, 5, 3, 1, 1);

        doubleSpinBox_camera_roll = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera_roll->setObjectName(QString::fromUtf8("doubleSpinBox_camera_roll"));
        doubleSpinBox_camera_roll->setMinimum(-180);
        doubleSpinBox_camera_roll->setMaximum(180);
        doubleSpinBox_camera_roll->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_camera_roll, 5, 4, 1, 1);

        doubleSpinBox_camera_yaw = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera_yaw->setObjectName(QString::fromUtf8("doubleSpinBox_camera_yaw"));
        doubleSpinBox_camera_yaw->setMinimum(-180);
        doubleSpinBox_camera_yaw->setMaximum(180);
        doubleSpinBox_camera_yaw->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_camera_yaw, 3, 4, 1, 1);

        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 0, 0, 1, 1);

        horizontalSlider_camera_x = new QSlider(groupBox);
        horizontalSlider_camera_x->setObjectName(QString::fromUtf8("horizontalSlider_camera_x"));
        horizontalSlider_camera_x->setMinimum(-5000);
        horizontalSlider_camera_x->setMaximum(5000);
        horizontalSlider_camera_x->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera_x, 0, 2, 1, 2);

        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 1, 0, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_2->addWidget(label_9, 2, 0, 1, 1);

        horizontalSlider_camera_y = new QSlider(groupBox);
        horizontalSlider_camera_y->setObjectName(QString::fromUtf8("horizontalSlider_camera_y"));
        horizontalSlider_camera_y->setMinimum(-5000);
        horizontalSlider_camera_y->setMaximum(5000);
        horizontalSlider_camera_y->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera_y, 1, 2, 1, 2);

        horizontalSlider_camera_z = new QSlider(groupBox);
        horizontalSlider_camera_z->setObjectName(QString::fromUtf8("horizontalSlider_camera_z"));
        horizontalSlider_camera_z->setMaximum(5000);
        horizontalSlider_camera_z->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_camera_z, 2, 2, 1, 2);

        doubleSpinBox_camera_x = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera_x->setObjectName(QString::fromUtf8("doubleSpinBox_camera_x"));
        doubleSpinBox_camera_x->setMinimum(-5);
        doubleSpinBox_camera_x->setMaximum(5);

        gridLayout_2->addWidget(doubleSpinBox_camera_x, 0, 4, 1, 1);

        doubleSpinBox_camera_y = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera_y->setObjectName(QString::fromUtf8("doubleSpinBox_camera_y"));
        doubleSpinBox_camera_y->setMinimum(-5);
        doubleSpinBox_camera_y->setMaximum(5);

        gridLayout_2->addWidget(doubleSpinBox_camera_y, 1, 4, 1, 1);

        doubleSpinBox_camera_z = new QDoubleSpinBox(groupBox);
        doubleSpinBox_camera_z->setObjectName(QString::fromUtf8("doubleSpinBox_camera_z"));
        doubleSpinBox_camera_z->setMaximum(5);

        gridLayout_2->addWidget(doubleSpinBox_camera_z, 2, 4, 1, 1);


        verticalLayout->addWidget(groupBox);

        tabWidget->addTab(stereo_manual, QString());

        retranslateUi(DialogCalibration);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(DialogCalibration);
    } // setupUi

    void retranslateUi(QDialog *DialogCalibration)
    {
        DialogCalibration->setWindowTitle(QApplication::translate("DialogCalibration", "Dialog", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("DialogCalibration", "Camera", 0, QApplication::UnicodeUTF8));
        label_ir->setText(QString());
        pushButton_start->setText(QApplication::translate("DialogCalibration", "start", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(stereo_auto), QApplication::translate("DialogCalibration", "Auto", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("DialogCalibration", "Camera", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("DialogCalibration", "yaw", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("DialogCalibration", "Pitch", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("DialogCalibration", "Roll", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("DialogCalibration", "x", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("DialogCalibration", "y", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("DialogCalibration", "z", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(stereo_manual), QApplication::translate("DialogCalibration", "Manual", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class DialogCalibration: public Ui_DialogCalibration {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGCALIBRATION_H
