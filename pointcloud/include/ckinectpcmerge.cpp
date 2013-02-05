#include "ckinectpcmerge.h"

CKinectPCMerge::CKinectPCMerge(int _argc, char** _argv, int _numPC, vector<string> _subTopics, string _pubTopic, vector<double> _ws)
    : QNode(_argc, _argv, "pcmerge"), argc(_argc), argv(_argv), numPC(_numPC), subTopics(_subTopics), pubTopic(_pubTopic), ws(_ws)
{    

}

void CKinectPCMerge::infoDisplay()
{
    ROS_INFO("pubTopic : %s", pubTopic.data());
    ROS_INFO("workspace origin x : %f", ws.at(0));
    ROS_INFO("workspace origin y : %f", ws.at(1));
    ROS_INFO("workspace origin z : %f", ws.at(2));
    ROS_INFO("workspace size x : %f", ws.at(3));
    ROS_INFO("workspace size y : %f", ws.at(4));
    ROS_INFO("workspace size z : %f", ws.at(5));
}

void CKinectPCMerge::ros_comms_init()
{
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::PointCloud2> (pubTopic, 1000);
    tf_listener = new tf::TransformListener;
    infoDisplay();
    for(int i=0;i<numPC;i++){
        QString name("pcreceiver");
        name += QString::number(i);
        pcReceivers.push_back(new PCReceiver(argc, argv, name.toStdString(), subTopics.at(i)));
        if(pcReceivers.at(i)->on_init()) ROS_INFO("SUCCESS to open node : %s", name.toStdString().data());
        else ROS_INFO("FAIL to open node : %s", name.toStdString().data());
    }

}

void CKinectPCMerge::run()
{
    ros::Rate loop_rate(1000);
    int count = 0;
    while ( ros::ok() ) {
        bool isStarted = 1;
        for(int i=0;i<numPC;i++){
            if(pcReceivers.at(i)->isStarted != 1)
                isStarted = 0;
        }

        if(isStarted){
            pCloudOut.reset(new Cloud);

            for(int k=0;k<numPC;k++){
                PCReceiver *pc = pcReceivers.at(k);
                pCloudTransformed.reset(new Cloud);
                *(pc->pCloud) = *(pc->pCloudIn);

                // downsampling
                pCloudSampled.reset(new Cloud);
//                CloudConstPtr pCloudConst = (CloudConstPtr)(pc->pCloud);
//                filter.downSampling(pCloudConst, pCloudSampled, 0.05, 0.05, 0.05);

                ros::Time now = ros::Time::now();
                tf_listener->waitForTransform("/origin", pc->pCloud->header.frame_id, now, ros::Duration(5.0));
                pcl_ros::transformPointCloud("/origin", *(pc->pCloud), *pCloudTransformed, *tf_listener);

                for(int i=0;i<pCloudTransformed->points.size();i++){
                    PointT temp_point = pCloudTransformed->points[i];
                    if(temp_point.z>0.1 && temp_point.x>=-1 && temp_point.x<=1 && temp_point.y>=0 && temp_point.y<=2)
                        pCloudOut->points.push_back(pCloudTransformed->points[i]);
                }
                pCloudTransformed.reset();
                pCloudSampled.reset();
            }

            // downsampling
//            CloudConstPtr pCloudConst = (CloudConstPtr)pCloudOut;
//            filter.downSampling(pCloudConst, pCloudSampled, 0.01, 0.01, 0.01);
            // publish pointcloud

            pcl::toROSMsg(*pCloudOut, output);
            output.header.frame_id="/origin";
            pub.publish (output);

            pCloudOut.reset();
            pCloudSampled.reset();

        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
