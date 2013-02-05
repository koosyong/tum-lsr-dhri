#include <ros/ros.h>
#include <ckinectfiltering.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ckinect_filtering");

    CKinectFiltering filtering;
    filtering.run();

}

