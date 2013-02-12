#include <ros/ros.h>
#include <ckinectfiltering.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "dhri_pointcloudFilter");

    CKinectFiltering filtering;
    filtering.run();

}

