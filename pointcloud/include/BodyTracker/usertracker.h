#ifndef USERTRACKER_H
#define USERTRACKER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
//#include "SceneDrawer.h"

#include <fstream>
#include <string>
#include <vector>

typedef pcl::PointXYZ _PointT;
typedef pcl::PointCloud<_PointT> _PointCloudT;

#define CHECK_RC(nRetVal, what)										\
    if (nRetVal != XN_STATUS_OK)									\
    {																\
        printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
        return nRetVal;												\
    }

using namespace std;

class userTracker
{
public:
    userTracker(ros::NodeHandle &node);
    ~userTracker();
    // This function is called each frame
    void userMainLoop(std::string frame_id);

    XnStatus registerCall();
// TODO: try to move something in protected or private areas
    static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie);
 //  static void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie);

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg_ptr);

    xn::Context g_Context;
    xn::DepthGenerator g_DepthGenerator;
    xn::UserGenerator g_UserGenerator;
    xn::MockDepthGenerator mockDepth;
    xn::DepthMetaData depthMD;
    xn::DepthMetaData depthMD_cb;

    int sem_;

    XnBool g_bNeedPose;
    XnChar g_strPose[20];

protected:
    XnUserID getClosestUser();

    void publishTransform(XnUserID const& user,
                          XnSkeletonJoint joint,
                          std::string const& frame_id,
                          std::string const& child_frame_id);

    void publishMatToFloor(const string &frame_id,
                           const string &child_frame_id);

    void publishTransforms(std::string const& frame_id);
    void modifyDepthMD(xn::DepthMetaData& depthMD_);

private:
    void loadMatrixFromFile(string nome_file, int rows, int columns, double **out_mat);
    void multi(double** matSol, double** mat1, double** mat2);

    ros::NodeHandle n;
    ros::Publisher userIDpub;

    double cameraToFloorTrans[3];
    double cameraToFloorQuat[4];

    double **constant_transform;
    double **mat_camera_to_floor_temp;
    double **mat_camera_to_floor;

    bool isTracking_;
    XnUserID userTracked_;
};

#endif // USERTRACKER_H
