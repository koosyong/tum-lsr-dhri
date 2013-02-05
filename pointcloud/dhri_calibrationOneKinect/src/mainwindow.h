#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <tf/transform_broadcaster.h>
#include "ckinectcalibrationstereo.h"
#include "ckinecttfpublisher.h"

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void pubTF();
    void pubTFCalib();

private:
    void readParams();
    void initGui();
    void initStereoAuto();
    void terminateStereoAuto();
    void calQuaternion();
    QImage IplImage2QImage(const IplImage *iplImage);
    IplImage *irImage;

private slots:
    void update_tf();
    void updateIR(IplImage* _irImage);
    void updateTF(CvMat* rotMat, CvMat* transMat);

    void on_horizontalSlider_camera_x_valueChanged(int value);
    void on_horizontalSlider_camera_y_valueChanged(int value);
    void on_horizontalSlider_camera_z_valueChanged(int value);
    void on_horizontalSlider_camera_yaw_valueChanged(int value);
    void on_horizontalSlider_camera_pitch_valueChanged(int value);
    void on_horizontalSlider_camera_roll_valueChanged(int value);

    void on_actionStereo_Auto_triggered(bool checked);
    void on_pushButton_clicked();

private:

    Ui::MainWindow *ui;
    CKinectCalibrationStereo *calibration;
    QTimer *timer_tf;
    CKinectTFPublisher tf;

    double camera_x;
    double camera_y;
    double camera_z;
    double camera_qx;
    double camera_qy;
    double camera_qz;
    double camera_qw;
    double camera_roll;
    double camera_pitch;
    double camera_yaw;


};

#endif // MAINWINDOW_H
