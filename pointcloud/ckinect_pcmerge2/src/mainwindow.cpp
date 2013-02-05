#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
//    readParams();
//    initGui();

//   if(qnode->on_init())
//       ROS_INFO("SUCCESS");
//   else
//       ROS_INFO("ERROR");

//    pcmerge = new CKinectPCMerge(argc, argv, numSubTopics, subTopics, pubTopic, ws);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::readParams()
{
    bool isOK = 1;

    if(ros::param::has("/ckinect/calibration/ws/origin_x")){
        ros::param::get("/ckinect/calibration/ws/origin_x", ws_origin_x);
        ws.push_back(ws_origin_x);
    }
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/ws/origin_y")){
        ros::param::get("/ckinect/calibration/ws/origin_y", ws_origin_y);
        ws.push_back(ws_origin_y);
    }
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/ws/origin_z")){
        ros::param::get("/ckinect/calibration/ws/origin_z", ws_origin_z);
        ws.push_back(ws_origin_z);
    }
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/ws/size_x")){
        ros::param::get("/ckinect/calibration/ws/size_x", ws_size_x);
        ws.push_back(ws_size_x);
    }
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/ws/size_y")){
        ros::param::get("/ckinect/calibration/ws/size_y", ws_size_y);
        ws.push_back(ws_size_y);
    }
    else isOK = 0;
    if(ros::param::has("/ckinect/calibration/ws/size_z")){
        ros::param::get("/ckinect/calibration/ws/size_z", ws_size_z);
        ws.push_back(ws_size_z);
    }
    else isOK = 0;

    if(ros::param::has("/ckinect/calibration/numSubTopics"))
        ros::param::get("/ckinect/calibration/numSubTopics", numSubTopics);
    else isOK = 0;
    for(int i=0;i<numSubTopics;i++){
        string paramType = "/ckinect/calibration/subTopic";
        std::stringstream stream;
        stream << i+1;
        paramType += stream.str();
        string paramValue;
        if(ros::param::has(paramType))
            ros::param::get(paramType, paramValue);
        else isOK = 0;
        subTopics.push_back(paramValue);
    }
    if(ros::param::has("/ckinect/calibration/pubTopic"))
        ros::param::get("/ckinect/calibration/pubTopic", pubTopic);
    else isOK = 0;


    if(isOK){
    }
    else{
        qDebug("Run 'rosparam load ckinect_pcmerge2.yaml' first");
        return;
    }
}


void MainWindow::initGui()
{
    ui->horizontalSlider_ws_origin_x->setValue(ws_origin_x*100);
    ui->doubleSpinBox_ws_origin_x->setValue(ws_origin_x);
    ui->horizontalSlider_ws_origin_y->setValue(ws_origin_y*100);
    ui->doubleSpinBox_ws_origin_y->setValue(ws_origin_y);
    ui->horizontalSlider_ws_origin_z->setValue(ws_origin_z*100);
    ui->doubleSpinBox_ws_origin_z->setValue(ws_origin_z);
    ui->horizontalSlider_ws_size_x->setValue(ws_size_x*100);
    ui->doubleSpinBox_ws_size_x->setValue(ws_size_x);
    ui->horizontalSlider_ws_size_y->setValue(ws_size_y*100);
    ui->doubleSpinBox_ws_size_y->setValue(ws_size_y);
    ui->horizontalSlider_ws_size_z->setValue(ws_size_z*100);
    ui->doubleSpinBox_ws_size_z->setValue(ws_size_z);
}

void MainWindow::run()
{

}
