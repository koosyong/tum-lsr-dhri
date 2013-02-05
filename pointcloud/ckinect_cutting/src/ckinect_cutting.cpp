
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/GridCells.h>
// general
#include <iostream>
using namespace std;

// Useful macros
#define FPS_CALC(_WHAT_) \
    do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
{ \
    ROS_INFO("FPS of %s: %f", _WHAT_, double(count)/double(now - last)); \
    count = 0; \
    last = now; \
    } \
    }while(false)

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef struct {
    double left, right, top, bottom, ztop, zbottom;
    double margin;
} PC_TYPE_WS;

ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher pub_grid;

double paramValue_workspace_x = 0.;
double paramValue_workspace_y = 0.;
double paramValue_workspace_z = 0.;
double paramValue_workspace_height = 0.7;
double paramValue_workspace_zheight = 0.7;
double paramValue_workspace_width = 1;

CloudPtr pCloud_input;
CloudPtr pCloud_output;
CloudPtr pCloudTransformed;
sensor_msgs::PointCloud2 output;
tf::TransformListener* tf_listener;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ckinect_cutting");

    ros::NodeHandle nh;

    sub = nh.subscribe ("/camera/depth_registered/points", 10, cloud_cb);
    pub_grid = nh.advertise<nav_msgs::GridCells> ("/ckinect/workspace",10);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/ckinect/pcloud/cutted", 10);

    tf_listener = new tf::TransformListener();

    ros::spin();
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    FPS_CALC ("cutting");

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pCloud_input.reset(new Cloud);
    pCloud_output.reset(new Cloud);
    pCloudTransformed.reset(new Cloud);

    pcl::fromROSMsg (*input, *pCloud_input);

    ros::Time now = ros::Time::now();
    tf_listener->waitForTransform("/origin", pCloud_input->header.frame_id, now, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/origin", *pCloud_input, *pCloudTransformed, *tf_listener);

    PC_TYPE_WS ws;
    ws.top = paramValue_workspace_y + paramValue_workspace_height/2;
    ws.bottom = paramValue_workspace_y - paramValue_workspace_height/2;
    ws.left = paramValue_workspace_x - paramValue_workspace_width/2;
    ws.right = paramValue_workspace_x + paramValue_workspace_width/2;
    ws.zbottom = paramValue_workspace_z;
    ws.ztop = paramValue_workspace_z + paramValue_workspace_zheight;
    ws.margin = 0.01;

    for(int i=0;i<pCloudTransformed->points.size();i++){
        PointT temp_point = pCloudTransformed->points[i];
        // cut off
//        cout<<temp_point.x<<" "<<temp_point.y<<" "<<temp_point.z<<endl;
        if(temp_point.x<ws.right && temp_point.x>ws.left && temp_point.y<ws.top && temp_point.y>ws.bottom && temp_point.z>ws.zbottom && temp_point.z<ws.ztop){
//            if(temp_point.x<10 && temp_point.x>-10 && temp_point.y<10 && temp_point.y>-10 && temp_point.z>-10 && temp_point.z<10){
            //        if(temp_point.x<-0.01 || temp_point.x>0.01)
            pCloud_output->points.push_back(temp_point);
//            cout<<"FDAS"<<endl;
            }
    }
    cout<<pCloud_output->points.size()<<endl;


    pcl::toROSMsg(*pCloud_output, output);
    output.header.frame_id="/origin";
    pub.publish (output);

    nav_msgs::GridCells workspace;

    workspace.cell_height = paramValue_workspace_height;
    workspace.cell_width = paramValue_workspace_width;
    geometry_msgs::Point wsp;
    wsp.x = paramValue_workspace_x;
    wsp.y = paramValue_workspace_y;
    wsp.z = paramValue_workspace_z;
    workspace.cells.push_back(wsp);
    workspace.header.frame_id="/origin";
    pub_grid.publish (workspace);

    pCloud_output.reset();
    pCloud_input.reset();
    pCloudTransformed.reset();

}

