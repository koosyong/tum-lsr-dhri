#include "ckinect.h"

template<typename PointT>
CKinect<PointT>* CKinect<PointT>::parent = NULL;

template<typename PointT>
CKinect<PointT>::CKinect(string _paramType_subTopic, string _paramType_pubTopic, string _paramValue_subTopic, string _paramValue_pubTopic)
    : paramType_subTopic(_paramType_subTopic), paramType_pubTopic(_paramType_pubTopic), paramValue_subTopic(_paramValue_subTopic), paramValue_pubTopic(_paramValue_pubTopic)
{    
    parent = this;
    isParam = 1;
    isPub = 1;
    init();
}

template<typename PointT>
void CKinect<PointT>::init()
{
    if(ros::param::has(paramType_subTopic))
        ros::param::get(paramType_subTopic, paramValue_subTopic);
    else{
        ROS_WARN("'%s' is not defined in `the parameter server!", paramType_subTopic.data());
        ROS_WARN("Parameters are set as default values");
        ROS_INFO("parameter of '%s'' : %s", paramType_subTopic.data(), paramValue_subTopic.data());
        isParam = 0;
    }
    if(ros::param::has(paramType_pubTopic))
        ros::param::get(paramType_pubTopic, paramValue_pubTopic);
    else{
        ROS_WARN("'%s' is not defined in the parameter server!", paramType_pubTopic.data());
        ROS_WARN("Parameters are set as default values");
        ROS_INFO("parameter of '%s'' : %s", paramType_pubTopic.data(), paramValue_pubTopic.data());
        isParam = 0;
    }

}

template<typename PointT>
void CKinect<PointT>::run()
{

    // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe (paramValue_subTopic, 1, cloud_cb);
//    pub = nh.advertise<sensor_msgs::PointCloud2> (paramValue_pubTopic, 1);
    ros::spin();
}

template<typename PointT>
void CKinect<PointT>::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    FPS_CALC ("transforming");

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    parent->pCloud_input.reset(new Cloud);
    parent->pCloud_output.reset(new Cloud);
    pcl::fromROSMsg (*input, *parent->pCloud_input);

    parent->processing();
    parent->isPub = 1;
    if(parent->isPub){
//        sensor_msgs::PointCloud2 output;
//        output.header.frame_id="/origin";
//        pcl::toROSMsg(*parent->pCloud_input, output);
//        parent->pub.publish (output);
    }
}
