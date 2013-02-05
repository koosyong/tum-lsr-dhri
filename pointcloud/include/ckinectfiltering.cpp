#include "ckinectfiltering.h"

CKinectFiltering::CKinectFiltering()
    :CKinect<PointT>::CKinect("/ckinect/filtering/subTopic", "/ckinect/filtering/pubTopic", "/camera/depth_registered/points", "/ckinect/pcloud/filtered")
{
    CKinect<PointT>::parent = this;
    paramValue_pubTopic_segmentedRGB = "/ckinect/pcloud/segmentedRGB";
    paramValue_pubTopic_segmentedID = "/ckinect/pcloud/segmentedID";
    paramValue_pubTopic_workspace = "/ckinect/workspace";

    paramValue_workspace_on = 0;
    paramValue_workspace_x = 0.;
    paramValue_workspace_y = 0.;
    paramValue_workspace_z = 0.;
    paramValue_workspace_height = 0.5;
    paramValue_workspace_zheight = 0.5;
    paramValue_workspace_width = 0.5;

    paramValue_downsampling_on = 0;
    paramValue_downsampling_leaf = 0.02;

    paramValue_planeExtraction_on = 0;
    paramValue_planeExtraction_numPlane = 1;

    paramValue_segmentation_on = 0;
    paramValue_segmentation_tolerance = 0.02;
    paramValue_segmentation_minSize = 20;
    paramValue_segmentation_maxSize = 25000000;
       init();



}


void CKinectFiltering::run()
{
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2> (paramValue_pubTopic, 1);
    CKinect<PointT>::run();

}

void CKinectFiltering::init()
{
    if(ros::param::has("/ckinect/filtering/pubTopic/segmentedRGB"))
        ros::param::get("/ckinect/filtering/pubTopic/segmentedRGB", paramValue_pubTopic_segmentedRGB);
    else{
        ROS_WARN("'/ckinect/filtering/pubTopic/segmentedRGB' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/filtering/pubTopic/segmentedID"))
        ros::param::get("/ckinect/filtering/pubTopic/segmentedID", paramValue_pubTopic_segmentedID);
    else{
        ROS_WARN("'/ckinect/filtering/pubTopic/segmentedID' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/filtering/pubTopic/workspace"))
        ros::param::get("/ckinect/filtering/pubTopic/workspace", paramValue_pubTopic_workspace);
    else{
        ROS_WARN("'/ckinect/filtering/pubTopic/workspace' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/on"))
        ros::param::get("/ckinect/workspace/on", paramValue_workspace_on);
    else{
        ROS_WARN("'/ckinect/workspace/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/x"))
        ros::param::get("/ckinect/workspace/x", paramValue_workspace_x);
    else{
        ROS_WARN("'/ckinect/workspace/x' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/y"))
        ros::param::get("/ckinect/workspace/y", paramValue_workspace_y);
    else{
        ROS_WARN("'/ckinect/workspace/y' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/z"))
        ros::param::get("/ckinect/workspace/z", paramValue_workspace_z);
    else{
        ROS_WARN("'/ckinect/workspace/z' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/width"))
        ros::param::get("/ckinect/workspace/width", paramValue_workspace_width);
    else{
        ROS_WARN("'/ckinect/workspace/width' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/height"))
        ros::param::get("/ckinect/workspace/height", paramValue_workspace_height);
    else{
        pCloud_input = pCloud_output;
        ROS_WARN("'/ckinect/workspace/height' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/workspace/zheight"))
        ros::param::get("/ckinect/workspace/zheight", paramValue_workspace_zheight);
    else{
        ROS_WARN("'/ckinect/workspace/zheight' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/downsampling/on"))
        ros::param::get("/ckinect/downsampling/on", paramValue_downsampling_on);
    else{
        ROS_WARN("'/ckinect/downsampling/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/downsampling/leaf"))
        ros::param::get("/ckinect/downsampling/leaf", paramValue_downsampling_leaf);
    else{
        ROS_WARN("'/ckinect/downsampling/leaf' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/planeExtraction/on"))
        ros::param::get("/ckinect/planeExtraction/on", paramValue_planeExtraction_on);
    else{
        ROS_WARN("'/ckinect/planeExtraction/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/planeExtraction/numPlane"))
        ros::param::get("/ckinect/planeExtraction/numPlane", paramValue_planeExtraction_numPlane);
    else{
        ROS_WARN("'/ckinect/planeExtraction/numPlane' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/segmentation/on"))
        ros::param::get("/ckinect/segmentation/on", paramValue_segmentation_on);
    else{
        ROS_WARN("'/ckinect/segmentation/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/segmentation/tolerance"))
        ros::param::get("/ckinect/segmentation/tolerance", paramValue_segmentation_tolerance);
    else{
        ROS_WARN("'/ckinect/segmentation/tolerance' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/segmentation/minSize"))
        ros::param::get("/ckinect/segmentation/minSize", paramValue_segmentation_minSize);
    else{
        ROS_WARN("'/ckinect/segmentation/minSize' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }
    if(ros::param::has("/ckinect/segmentation/maxSize"))
        ros::param::get("/ckinect/segmentation/maxSize", paramValue_segmentation_maxSize);
    else{
        ROS_WARN("'/ckinect/segmentation/maxSize' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
    }

    if(!isParam){
        ROS_WARN("Run 'rosparam load ckinect_filtering.yaml' first");
    }

    if(paramValue_workspace_on){
        ROS_INFO("parameter of '/ckinect/filtering/pubTopic/workspace' : %s", paramValue_pubTopic_workspace.data());
        ROS_INFO("param_workspace_x : %f", paramValue_workspace_x);
        ROS_INFO("param_workspace_y : %f", paramValue_workspace_y);
        ROS_INFO("param_workspace_z : %f", paramValue_workspace_z);
        ROS_INFO("param_workspace_height : %f", paramValue_workspace_height);
        ROS_INFO("param_workspace_width : %f", paramValue_workspace_width);
        ROS_INFO("param_workspace_zheight : %f", paramValue_workspace_zheight);
    }
    if(paramValue_downsampling_on)
        ROS_INFO("param_downsampling_leaf : %f", paramValue_downsampling_leaf);
    if(paramValue_planeExtraction_on){
        ROS_INFO("param_planeExtraction_numPlane : %d", paramValue_planeExtraction_numPlane);
    }
    if(paramValue_segmentation_on){
        ROS_INFO("parameter of '/ckinect/filtering/pubTopic/segmentedRGB' : %s", paramValue_pubTopic_segmentedRGB.data());
        ROS_INFO("parameter of '/ckinect/filtering/pubTopic/segmentedID' : %s", paramValue_pubTopic_segmentedID.data());
        ROS_INFO("param_segmentation_tolerance : %f", paramValue_segmentation_tolerance);
        ROS_INFO("param_segmentation_minSize : %f", paramValue_segmentation_minSize);
        ROS_INFO("param_segmentation_maxSize : %f", paramValue_segmentation_maxSize);
    }

    // Create a ROS subscriber for the input point cloud

    if(paramValue_segmentation_on){
        pub_segmentedRGB = nh.advertise<sensor_msgs::PointCloud2> (paramValue_pubTopic_segmentedRGB, 1);
        pub_segmentedID = nh.advertise<sensor_msgs::PointCloud2> (paramValue_pubTopic_segmentedID, 1);
    }
    if(paramValue_workspace_on)
        pub_grid = nh.advertise<nav_msgs::GridCells> (paramValue_pubTopic_workspace,1);

    workspace.cell_height = paramValue_workspace_height;
    workspace.cell_width = paramValue_workspace_width;
    geometry_msgs::Point ws;
    ws.x = paramValue_workspace_x;
    ws.y = paramValue_workspace_y;
    ws.z = paramValue_workspace_z;
    workspace.cells.push_back(ws);
}

void CKinectFiltering::processing()
{
    // downsampling
    if(paramValue_downsampling_on){
        CloudConstPtr pCloudConst = (CloudConstPtr)pCloud_input;
        filter.downSampling(pCloudConst, pCloud_output, paramValue_downsampling_leaf, paramValue_downsampling_leaf, paramValue_downsampling_leaf);
        pCloud_input.reset(new Cloud);
        pCloud_input = pCloud_output;
        pCloud_output.reset(new Cloud);
    }

    // cut
    if(paramValue_workspace_on){
        isPub = 1;
        PC_TYPE_WS ws;
        ws.top = paramValue_workspace_y + paramValue_workspace_height/2;
        ws.bottom = paramValue_workspace_y - paramValue_workspace_height/2;
        ws.left = paramValue_workspace_x - paramValue_workspace_width/2;
        ws.right = paramValue_workspace_x + paramValue_workspace_width/2;
        ws.zbottom = paramValue_workspace_z;
        ws.ztop = paramValue_workspace_z + paramValue_workspace_zheight;
        ws.margin = 0.01;


        sensor_msgs::PointCloud2 output;
        output.header.frame_id="/origin";
        pcl::toROSMsg(*pCloud_input, output);
        pub_filtered.publish (output);

        filter.cut(pCloud_input, pCloud_output, ws);
        pCloud_input.reset(new Cloud);
        pCloud_input = pCloud_output;
        pCloud_output.reset(new Cloud);

        workspace.header.frame_id="/origin";
        pub_grid.publish (workspace);

    }

    // plane extraction
    if(paramValue_planeExtraction_on){
        std::vector<CloudPtr> pClouds_plane;
        for(int i=0;i<pClouds_plane.size();i++)
            pClouds_plane.at(i).reset();
        pClouds_plane.clear();

        filter.extractPlane(pCloud_input, pCloud_output, pClouds_plane, paramValue_planeExtraction_numPlane);
        if(paramValue_segmentation_on){
            isPub = 0;
            pCloud_input.reset(new Cloud);
            pCloud_input = pCloud_output;
            pCloud_output.reset(new Cloud);
        }
        else{
            isPub = 1;
        }
    }
    // segmentation
    if(paramValue_segmentation_on){
        vector<CloudPtr> pClouds_object;
        for(int i=0;i<pClouds_object.size();i++)
            pClouds_object.at(i).reset();
        pClouds_object.clear();
        filter.segmentation(pCloud_input, pClouds_object, paramValue_segmentation_tolerance, paramValue_segmentation_minSize, paramValue_segmentation_maxSize);

        // make a segmented cloud
        CloudPtr pCloudSegRGB (new Cloud);
        pCloudSegRGB->header = pCloud_input->header;
        pCloudSegRGB->is_dense = pCloud_input->is_dense;
        CloudPtrID pCloudSegID (new CloudID);
        pCloudSegID->header = pCloud_input->header;
        pCloudSegID->is_dense = pCloud_input->is_dense;
        for(int i=0;i<pClouds_object.size();i++){
            CloudPtr pCloud = pClouds_object.at(i);
            int id = i;
            for(int j=0;j<pCloud->points.size();j++){
                PointT point = pCloud->points[j];
                PointT pointRGB;
                PointID pointID;

                pointRGB.x = point.x;
                pointRGB.y = point.y;
                pointRGB.z = point.z;
                pointRGB.r = point.r;
                pointRGB.g = point.g;
                pointRGB.b = point.b;
                pCloudSegRGB->points.push_back(pointRGB);

                pointID.x = point.x;
                pointID.y = point.y;
                pointID.z = point.z;
                pointID.intensity = id;
                pCloudSegID->points.push_back(pointID);
            }
        }
        sensor_msgs::PointCloud2 output_segmentedRGB;
        pcl::toROSMsg(*pCloudSegRGB, output_segmentedRGB);
        output_segmentedRGB.header.frame_id="/origin";
        pub_segmentedRGB.publish (output_segmentedRGB);

        sensor_msgs::PointCloud2 output_segmentedID;
        pcl::toROSMsg(*pCloudSegID, output_segmentedID);
        output_segmentedID.header.frame_id="/origin";
        pub_segmentedID.publish (output_segmentedID);
    }
}
