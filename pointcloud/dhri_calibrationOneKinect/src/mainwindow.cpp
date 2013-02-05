#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <QTimer>
#define pi 3.141592

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ros::start();
    ros::NodeHandle n;

    ui->setupUi(this);
    readParams();
    initGui();

    camera_x = 0.0;
    camera_y = 0.0;
    camera_z = 0.0;
    camera_qx = 0.0;
    camera_qy = 0.0;
    camera_qz = 0.0;
    camera_qw = 0.0;



    calibration = new CKinectCalibrationStereo("/camera/rgb/image_mono");
}

MainWindow::~MainWindow()
{
    terminateStereoAuto();
    delete ui;
}

void MainWindow::readParams()
{
    bool isOK = 1;
    if(ros::param::has("/ckinect/calibration/camera_x"))
        ros::param::get("/ckinect/calibration/camera_x", camera_x);
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/camera_y"))
        ros::param::get("/ckinect/calibration/camera_y", camera_y);
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/camera_z"))
        ros::param::get("/ckinect/calibration/camera_z", camera_z);
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/camera_yaw"))
        ros::param::get("/ckinect/calibration/camera_yaw", camera_yaw);
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/camera_pitch"))
        ros::param::get("/ckinect/calibration/camera_pitch", camera_pitch);
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/camera_roll"))
        ros::param::get("/ckinect/calibration/camera_roll", camera_roll);
    else isOK = 0;


    if(isOK){
    }
    else{
        qDebug("Run 'rosparam load ckinect_calibration_stereo.yaml' first");
        return;
    }
}

void MainWindow::initGui()
{
    ui->horizontalSlider_camera_x->setValue(camera_x*1000);
    ui->doubleSpinBox_camera_x->setValue(camera_x);
    ui->horizontalSlider_camera_y->setValue(camera_y*1000);
    ui->doubleSpinBox_camera_y->setValue(camera_y);
    ui->horizontalSlider_camera_z->setValue(camera_z*1000);
    ui->doubleSpinBox_camera_z->setValue(camera_z);
    ui->horizontalSlider_camera_yaw->setValue(camera_yaw*100);
    ui->doubleSpinBox_camera_yaw->setValue(camera_yaw);
    ui->horizontalSlider_camera_pitch->setValue(camera_pitch*100);
    ui->doubleSpinBox_camera_pitch->setValue(camera_pitch);
    ui->horizontalSlider_camera_roll->setValue(camera_roll*100);
    ui->doubleSpinBox_camera_roll->setValue(camera_roll);

}

void MainWindow::pubTFCalib()
{
    tf.camera_x = camera_x;
    tf.camera_y = camera_y;
    tf.camera_z = camera_z;
    tf.camera_roll = camera_roll;
    tf.camera_pitch = camera_pitch;
    tf.camera_yaw = camera_yaw;

    tf.update();
}

void MainWindow::update_tf()
{
    pubTFCalib();
}

QImage MainWindow::IplImage2QImage(const IplImage *iplImage)
{
    int height = iplImage->height;
    int width = iplImage->width;

    if  (iplImage->depth == IPL_DEPTH_8U && iplImage->nChannels == 3)
    {
        const uchar *qImageBuffer = (const uchar*)iplImage->imageData;
        QImage img(qImageBuffer, width, height, QImage::Format_RGB888);
        return img.rgbSwapped();
    } else if  (iplImage->depth == IPL_DEPTH_8U && iplImage->nChannels == 1){
        const uchar *qImageBuffer = (const uchar*)iplImage->imageData;
        QImage img(qImageBuffer, width, height, QImage::Format_Indexed8);

        QVector<QRgb> colorTable;
        for (int i = 0; i < 256; i++){
            colorTable.push_back(qRgb(i, i, i));
        }
        img.setColorTable(colorTable);
        return img;
    }else{
        qDebug("Image cannot be converted.");
        return QImage();
    }
}

void MainWindow::initStereoAuto()
{
    calibration->start();
    connect(calibration, SIGNAL(emitIRImage(IplImage*)), this, SLOT(updateIR(IplImage*)));
    connect(calibration, SIGNAL(emitTF(CvMat*,CvMat*)), this, SLOT(updateTF(CvMat*, CvMat*)));

}

void MainWindow::terminateStereoAuto()
{
    disconnect(calibration, SIGNAL(emitIRImage(IplImage*)), this, SLOT(updateIR1(IplImage*)));
    calibration->quit();
}

void MainWindow::updateIR(IplImage *_irImage)
{
    irImage = _irImage;
    if(irImage->imageSize != 0)
        ui->label_ir->setPixmap(QPixmap::fromImage(IplImage2QImage(irImage)));
}

void MainWindow::updateTF(CvMat* rotMat, CvMat* transMat)
{
    camera_x = CV_MAT_ELEM(*transMat, float, 0, 0);
    camera_y = CV_MAT_ELEM(*transMat, float, 1, 0);
    camera_z = CV_MAT_ELEM(*transMat, float, 2, 0);

    double m00 = CV_MAT_ELEM(*rotMat, float, 0, 0);
    double m01 = CV_MAT_ELEM(*rotMat, float, 0, 1);
    double m02 = CV_MAT_ELEM(*rotMat, float, 0, 2);
    double m10 = CV_MAT_ELEM(*rotMat, float, 1, 0);
    double m11 = CV_MAT_ELEM(*rotMat, float, 1, 1);
    double m12 = CV_MAT_ELEM(*rotMat, float, 1, 2);
    double m20 = CV_MAT_ELEM(*rotMat, float, 2, 0);
    double m21 = CV_MAT_ELEM(*rotMat, float, 2, 1);
    double m22 = CV_MAT_ELEM(*rotMat, float, 2, 2);

    float tr = m00 + m11 + m22;
    if (tr > 0)
    {
        float S = sqrt(tr+1.0) * 2; // S=4*qw
        camera_qw = 0.25 * S;
        camera_qx = (m21 - m12) / S;
        camera_qy = (m02 - m20) / S;
        camera_qz = (m10 - m01) / S; }
    else if ((m00 > m11)&(m00 > m22)) {
        float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
        camera_qw = (m21 - m12) / S;
        camera_qx = 0.25 * S;
        camera_qy = (m01 + m10) / S;
        camera_qz = (m02 + m20) / S; }
    else if (m11 > m22) {
        float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
        camera_qw = (m02 - m20) / S;
        camera_qx = (m01 + m10) / S;
        camera_qy = 0.25 * S;
        camera_qz = (m12 + m21) / S;
    }
    else {
        float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
        camera_qw = (m10 - m01) / S;
        camera_qx = (m02 + m20) / S;
        camera_qy = (m12 + m21) / S;
        camera_qz = 0.25 * S;
    }

    double q0,q1,q2,q3;
    q0 = camera_qw;
    q1 = camera_qx;
    q2 = camera_qy;
    q3 = camera_qz;

    camera_roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))/pi*180;
    camera_pitch = asin(2*(q0*q2-q3*q1))/pi*180;
    camera_yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))/pi*180;

    initGui();
    update_tf();

}

void MainWindow::on_horizontalSlider_camera_x_valueChanged(int value)
{
    camera_x = (double)value/1000;
    ui->doubleSpinBox_camera_x->setValue(camera_x);
    update_tf();
}

void MainWindow::on_horizontalSlider_camera_y_valueChanged(int value)
{
    camera_y = (double)value/1000;
    ui->doubleSpinBox_camera_y->setValue(camera_y);
    update_tf();
}

void MainWindow::on_horizontalSlider_camera_z_valueChanged(int value)
{
    camera_z = (double)value/1000;
    ui->doubleSpinBox_camera_z->setValue(camera_z);
    update_tf();
}

void MainWindow::on_horizontalSlider_camera_yaw_valueChanged(int value)
{
    camera_yaw = (double)value/100;
    ui->doubleSpinBox_camera_yaw->setValue(camera_yaw);
    update_tf();
}

void MainWindow::on_horizontalSlider_camera_pitch_valueChanged(int value)
{
    camera_pitch = (double)value/100;
    ui->doubleSpinBox_camera_pitch->setValue(camera_pitch);
    update_tf();
}

void MainWindow::on_horizontalSlider_camera_roll_valueChanged(int value)
{
    camera_roll = (double)value/100;
    ui->doubleSpinBox_camera_roll->setValue(camera_roll);
    update_tf();
}


void MainWindow::on_actionStereo_Auto_triggered(bool checked)
{
    if(checked){
        ui->tabWidget->setCurrentIndex(1);
        initStereoAuto();
    }
    else{
        terminateStereoAuto();
    }
}

void MainWindow::on_pushButton_clicked()
{
    calibration->startCalibration();
}

