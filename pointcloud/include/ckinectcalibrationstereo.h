#ifndef CKINECTCALIBRATIONSTEREO_H
#define CKINECTCALIBRATIONSTEREO_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <QThread>

#include <string>
using namespace std;

class CKinectCalibrationStereo : public QThread
{
    Q_OBJECT
public:
    CKinectCalibrationStereo(string _inputTopic, int _maxIter = 30);
    void run();
    void startCalibration();
    int getCnt(){return successes;};

public:
    void image_cb (const sensor_msgs::ImageConstPtr& input);

private:
    string inputTopic;
    IplImage *IRImage;
    IplImage *gray_image;
    bool isStarted;
    int cnt;
    int successes;
    int maxIter;
    CvMat* image_points;
    CvMat* object_points;
    int board_w;
    int board_h;
    float board_width;
    int board_n;
    CvMat* intrinsic_matrix;
    CvMat* distortion_coeffs;

signals:
    void emitIRImage(IplImage* irImage);
    void emitTF(CvMat* rotMat, CvMat* transMat);
};

#endif // CKINECTCALIBRATIONSTEREO_H
