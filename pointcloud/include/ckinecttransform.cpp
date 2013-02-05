#include "ckinecttransform.h"

CKinectTransform::CKinectTransform()
    : CKinect<PointT>::CKinect("/ckinect/transform/subTopic", "/ckinect/transform/pubTopic", "/camera/rgb/points", "/ckinect/pcloud/transformed")
{
    CKinect<PointT>::parent = this;
    paramValue_height = 0.67;
    paramValue_angle = 202.5;
    init();
}

void CKinectTransform::init()
{
    // get rosparam    
    if(ros::param::has("/ckinect/transform/height"))
        ros::param::get("/ckinect/transform/height", paramValue_height);
    else{
        ROS_WARN("'/ckinect/transform/height' is not defined in the parameter server!");        
        ROS_WARN("Parameters are set as default values");
        ROS_INFO("parameter of '/ckinect/transform/height' : %f", paramValue_height);
        isParam = 0;
    }
    if(ros::param::has("/ckinect/transform/angle"))
        ros::param::get("/ckinect/transform/angle", paramValue_angle);
    else{
        ROS_WARN("'/ckinect/transform/angle' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        ROS_INFO("parameter of '/ckinect/transform/angle' : %f", paramValue_angle);
        isParam = 0;
    }
    if(!isParam){
        ROS_WARN("Run 'rosparam load ckinect_transform.yaml' first");        
    }
}

void CKinectTransform::processing()
{
    // transformation
    double rotX = paramValue_angle;
    double offsetZ = paramValue_height;
    TransformList transformList;
    Transform transform;

    transform.type = PC_ROT_X;
    transform.value = rotX;
    transformList.push_back(transform);
    transform.type = PC_TRANS_Z;
    transform.value = offsetZ;
    transformList.push_back(transform);

    filter.transform(pCloud_input, pCloud_output, transformList);

}
