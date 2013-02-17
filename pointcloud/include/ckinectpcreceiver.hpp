#include "ckinectpcreceiver.h"

template<typename PointT>
CKinectPCReceiver<PointT>::CKinectPCReceiver(int argc, char** argv, string _nodeName, string _subTopic)
    :QNode(argc,argv,_nodeName), nodeName(_nodeName), subTopic(_subTopic)
{
    isStarted = 0;
    isUsing = 0;

    pCloud.reset(new Cloud);
    pCloudIn.reset(new Cloud);
    pCloudSampled.reset(new Cloud);
    pCloudOut.reset(new Cloud);
}

template<typename PointT>
void CKinectPCReceiver<PointT>::ros_comms_init() {
    ros::NodeHandle n;
    sub = n.subscribe(subTopic, 1000, &CKinectPCReceiver<PointT>::cloud_cb, this);
}

template<typename PointT>
void CKinectPCReceiver<PointT>::run()
{
    ROS_INFO("Subscribe topic : %s",subTopic.data());

    ros::MultiThreadedSpinner spinner(10);
    spinner.spin();
    ROS_INFO("Subscribe topic : %s",subTopic.data());
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


template<typename PointT>
void CKinectPCReceiver<PointT>::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud


    pcl::fromROSMsg (*input, *pCloudIn);
    processing();

    isStarted = 1;

}

template<typename PointT>
void CKinectPCReceiver<PointT>::processing()
{

}
