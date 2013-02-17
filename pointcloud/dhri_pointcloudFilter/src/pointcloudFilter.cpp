#include <ros/ros.h>
#include <ckinectfiltering.h>

int main (int argc, char** argv)
{
    CKinectFiltering filtering;
    filtering.on_init();
    while ( ros::ok() ){}
}

