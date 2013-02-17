#include "ckinectfiltering.h"

CKinectFiltering::CKinectFiltering()
    : QNode(0, 0, "dhri_pointcloudFilter")
{
}

void CKinectFiltering::ros_comms_init()
{
    ros::NodeHandle n;
    readParam();
    infoDisplay();
    // init subs
    for(int i=0;i<param_sub_num;i++){
        QString name("dhri_pointcloudFilter_sub_");
        name += QString::number(i+1);
        pcReceivers.push_back(new PCReceiver(0, 0, name.toStdString(), param_sub_topics.at(i)));
        if(pcReceivers.at(i)->on_init()) ROS_INFO("SUCCESS to open node : %s", name.toStdString().data());
        else ROS_INFO("FAIL to open node : %s", name.toStdString().data());
    }
    // init pubs
    pub_filtered = n.advertise<sensor_msgs::PointCloud2> (param_pub_topic, 1000);
    if(param_workspace_on == 1){
        pub_workspace = n.advertise<nav_msgs::GridCells> (param_workspace_topic, 1000);
    }
    if(param_segmentation_on == 1){
        pub_segmentedRGB = n.advertise<sensor_msgs::PointCloud2> (param_segmentation_topicRGB, 1);
        pub_segmentedID = n.advertise<sensor_msgs::PointCloud2> (param_segmentation_topicID, 1);
    }
    // tf listen
    tf_listener = new tf::TransformListener;

}

void CKinectFiltering::run()
{
    ros::Rate loop_rate(1000);
    int count = 0;
    while ( ros::ok() ) {
        bool isStarted = 1;
        for(int i=0;i<param_sub_num;i++){
            if(pcReceivers.at(i)->isStarted != 1)
                isStarted = 0;
        }
        if(isStarted){
            pCloudOut.reset(new Cloud);
            for(int k=0;k<param_sub_num;k++){
                PCReceiver *pc = pcReceivers.at(k);
                pCloud_output.reset(new Cloud);
                *(pc->pCloud) = *(pc->pCloudIn);

                // transform to the frame
                ros::Time now = ros::Time::now();
                string frame = param_sub_frames.at(k);
                tf_listener->waitForTransform(frame.data(), pc->pCloud->header.frame_id, now, ros::Duration(5.0));
                pcl_ros::transformPointCloud(frame.data(), *(pc->pCloud), *pCloud_output, *tf_listener);

                pCloud_input.reset(new Cloud);
                pCloud_input = pCloud_output;
                pCloud_output.reset(new Cloud);

                // workspace
                if(param_workspace_on == 1){
                    PC_TYPE_WS ws;
                    ws.top = param_workspace_y + param_workspace_height/2;
                    ws.bottom = param_workspace_y - param_workspace_height/2;
                    ws.left = param_workspace_x - param_workspace_width/2;
                    ws.right = param_workspace_x + param_workspace_width/2;
                    ws.zbottom = param_workspace_z;
                    ws.ztop = param_workspace_z + param_workspace_zheight;

                    filter.cut(pCloud_input, pCloud_output, ws);
                    pCloud_input.reset(new Cloud);
                    pCloud_input = pCloud_output;
                    pCloud_output.reset(new Cloud);

                    workspace.header.frame_id = frame.data();
                    pub_workspace.publish (workspace);

                }
                for(int i=0;i<pCloud_input->points.size();i++){
                    pCloudOut->points.push_back(pCloud_input->points[i]);
                }
            }

            pCloud_input.reset(new Cloud);
            pCloud_input = pCloudOut;
            pCloudOut.reset();

            // downsampling
            if(param_downsampling_on == 1){
                CloudConstPtr pCloudConst = (CloudConstPtr)pCloud_input;
                filter.downSampling(pCloudConst, pCloud_output, param_downsampling_leaf, param_downsampling_leaf, param_downsampling_leaf);
                pCloud_input.reset(new Cloud);
                pCloud_input = pCloud_output;
                pCloud_output.reset(new Cloud);
            }

            // plane extraction
            if(param_planeExtraction_on == 1){
                std::vector<CloudPtr> pClouds_plane;
                filter.extractPlane(pCloud_input, pCloud_output, pClouds_plane, param_planeExtraction_numPlane);
                pCloud_input.reset(new Cloud);
                pCloud_input = pCloud_output;
                pCloud_output.reset(new Cloud);
            }

            // publish filtered pointcloud if no segmentation
            if(pCloud_input->points.size() != 0){
                pcl::toROSMsg(*pCloud_input, output);
                output.header.frame_id = param_pub_frame.data();
                pub_filtered.publish (output);
                if(param_segmentation_on == 0)
                    pCloud_input.reset();
            }
            // segmentation
            if(param_segmentation_on == 1 && pCloud_input->points.size() != 0){
                vector<CloudPtr> pClouds_object;
                for(int i=0;i<pClouds_object.size();i++)
                    pClouds_object.at(i).reset();
                pClouds_object.clear();
                filter.segmentation(pCloud_input, pClouds_object, param_segmentation_tolerance, param_segmentation_minSize, param_segmentation_maxSize);

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
                if(pCloudSegRGB->points.size() != 0){
                    sensor_msgs::PointCloud2 output_segmentedRGB;
                    pcl::toROSMsg(*pCloudSegRGB, output_segmentedRGB);
                    output_segmentedRGB.header.frame_id = param_pub_frame.data();
                    pub_segmentedRGB.publish (output_segmentedRGB);
                }
                if(pCloudSegID->points.size() != 0){
                    sensor_msgs::PointCloud2 output_segmentedID;
                    pcl::toROSMsg(*pCloudSegID, output_segmentedID);
                    output_segmentedID.header.frame_id = param_pub_frame.data();
                    pub_segmentedID.publish (output_segmentedID);
                }
            }


        }
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

void CKinectFiltering::readParam()
{
    isParam = 1;
    // number of topic
    if(ros::param::has("/dhri/pointcloudFilter/sub/num"))
        ros::param::get("/dhri/pointcloudFilter/sub/num", param_sub_num);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/sub/num' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_sub_num = 1;
    }
    // sub_topics, sub_frames
    for(int i=0;i<param_sub_num;i++){
        stringstream ss_topic;
        ss_topic << "/dhri/pointcloudFilter/sub/topic" << i+1;
        string topic = ss_topic.str();
        stringstream ss_frame;
        ss_frame << "/dhri/pointcloudFilter/sub/frame" << i+1;
        string frame = ss_frame.str();

        string param_sub_topic;
        string param_sub_frame;

        if(ros::param::has(topic))
            ros::param::get(topic, param_sub_topic);
        else{
            ROS_WARN("'sub_topic'' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
            param_sub_topic = "/camera/depth_registered/points";
        }
        if(ros::param::has(frame))
            ros::param::get(frame, param_sub_frame);
        else{
            ROS_WARN("'sub_frame'' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
            param_sub_frame = "/origin";
        }
        param_sub_topics.push_back(param_sub_topic);
        param_sub_frames.push_back(param_sub_frame);
    }
    // pub_topic, pub_frame
    if(ros::param::has("/dhri/pointcloudFilter/pub/topic"))
        ros::param::get("/dhri/pointcloudFilter/pub/topic", param_pub_topic);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/pub/topic' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_pub_topic = "/dhri/points/filtered";
    }
    if(ros::param::has("/dhri/pointcloudFilter/pub/frame"))
        ros::param::get("/dhri/pointcloudFilter/pub/frame", param_pub_frame);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/pub/frame' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_pub_frame = "/origin";
    }
    // workspace
    if(ros::param::has("/dhri/pointcloudFilter/workspace/on"))
        ros::param::get("/dhri/pointcloudFilter/workspace/on", param_workspace_on);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/workspace/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_workspace_on = 0;
    }
    if(param_workspace_on == 1){
        if(ros::param::has("/dhri/pointcloudFilter/workspace/x"))
            ros::param::get("/dhri/pointcloudFilter/workspace/x", param_workspace_x);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/x' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/workspace/y"))
            ros::param::get("/dhri/pointcloudFilter/workspace/y", param_workspace_y);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/y' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/workspace/z"))
            ros::param::get("/dhri/pointcloudFilter/workspace/z", param_workspace_z);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/z' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/workspace/width"))
            ros::param::get("/dhri/pointcloudFilter/workspace/width", param_workspace_width);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/width' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/workspace/height"))
            ros::param::get("/dhri/pointcloudFilter/workspace/height", param_workspace_height);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/height' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/workspace/zheight"))
            ros::param::get("/dhri/pointcloudFilter/workspace/zheight", param_workspace_zheight);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/zheight' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/workspace/topic"))
            ros::param::get("/dhri/pointcloudFilter/workspace/topic", param_workspace_topic);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/workspace/topic' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        workspace.cell_width = param_workspace_width;
        workspace.cell_height = param_workspace_height;
        geometry_msgs::Point point;
        point.x = param_workspace_x;
        point.y = param_workspace_y;
        point.z = param_workspace_z;
        workspace.cells.push_back(point);
    }
    // downsampling
    if(ros::param::has("/dhri/pointcloudFilter/downsampling/on"))
        ros::param::get("/dhri/pointcloudFilter/downsampling/on", param_downsampling_on);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/downsampling/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_downsampling_on = 0;
    }
    if(param_downsampling_on == 1){
        if(ros::param::has("/dhri/pointcloudFilter/downsampling/leaf"))
            ros::param::get("/dhri/pointcloudFilter/downsampling/leaf", param_downsampling_leaf);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/downsampling/leaf' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
    }
    // planeExtraction
    if(ros::param::has("/dhri/pointcloudFilter/planeExtraction/on"))
        ros::param::get("/dhri/pointcloudFilter/planeExtraction/on", param_planeExtraction_on);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/planeExtraction/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_planeExtraction_on = 0;
    }
    if(param_planeExtraction_on == 1){
        if(ros::param::has("/dhri/pointcloudFilter/planeExtraction/numPlane"))
            ros::param::get("/dhri/pointcloudFilter/planeExtraction/numPlane", param_planeExtraction_numPlane);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/planeExtraction/numPlane' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
    }
    // segmentation
    if(ros::param::has("/dhri/pointcloudFilter/segmentation/on"))
        ros::param::get("/dhri/pointcloudFilter/segmentation/on", param_segmentation_on);
    else{
        ROS_WARN("'/dhri/pointcloudFilter/segmentation/on' is not defined in the parameter server!");
        ROS_WARN("Parameters are set as default values");
        isParam = 0;
        param_segmentation_on = 0;
    }
    if(param_segmentation_on == 1){
        if(ros::param::has("/dhri/pointcloudFilter/segmentation/tolerance"))
            ros::param::get("/dhri/pointcloudFilter/segmentation/tolerance", param_segmentation_tolerance);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/segmentation/tolerance' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/segmentation/minSize"))
            ros::param::get("/dhri/pointcloudFilter/segmentation/minSize", param_segmentation_minSize);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/segmentation/minSize' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/segmentation/maxSize"))
            ros::param::get("/dhri/pointcloudFilter/segmentation/maxSize", param_segmentation_maxSize);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/segmentation/maxSize' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/segmentation/topicRGB"))
            ros::param::get("/dhri/pointcloudFilter/segmentation/topicRGB", param_segmentation_topicRGB);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/segmentation/topicRGB' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
        if(ros::param::has("/dhri/pointcloudFilter/segmentation/topicID"))
            ros::param::get("/dhri/pointcloudFilter/segmentation/topicID", param_segmentation_topicID);
        else{
            ROS_WARN("'/dhri/pointcloudFilter/segmentation/topicID' is not defined in the parameter server!");
            ROS_WARN("Parameters are set as default values");
            isParam = 0;
        }
    }

    if(!isParam){
        ROS_WARN("Run 'rosparam load param.yaml' first");
    }
}


void CKinectFiltering::infoDisplay()
{
    ROS_INFO("param_sub_num : %d", param_sub_num);
    for(int i=0;i<param_sub_num;i++){
        ROS_INFO("param_sub_topic%d : %s", i+1, param_sub_topics.at(i).data());
        ROS_INFO("param_sub_frame%d : %s", i+1, param_sub_frames.at(i).data());
    }
    ROS_INFO("param_pub_topic : %s", param_pub_topic.data());
    ROS_INFO("param_pub_frame : %s", param_pub_frame.data());
    if(param_workspace_on == 1){
        ROS_INFO("param_workspace_x : %f", param_workspace_x);
        ROS_INFO("param_workspace_y : %f", param_workspace_y);
        ROS_INFO("param_workspace_z : %f", param_workspace_z);
        ROS_INFO("param_workspace_width : %f", param_workspace_width);
        ROS_INFO("param_workspace_height : %f", param_workspace_height);
        ROS_INFO("param_workspace_zheight : %f", param_workspace_zheight);
        ROS_INFO("param_workspace_topic : %s", param_workspace_topic.data());
    }
    if(param_downsampling_on == 1){
        ROS_INFO("param_downsampling_leaf : %f", param_downsampling_leaf);
    }
    if(param_planeExtraction_on == 1){
        ROS_INFO("param_downsampling_leaf : %d", param_planeExtraction_numPlane);
    }
    if(param_segmentation_on == 1){
        ROS_INFO("param_segmentation_tolerance : %f", param_segmentation_tolerance);
        ROS_INFO("param_segmentation_minSize : %f", param_segmentation_minSize);
        ROS_INFO("param_segmentation_maxSize : %f", param_downsampling_leaf);
        ROS_INFO("param_segmentation_topicRGB : %s", param_segmentation_topicRGB.data());
        ROS_INFO("param_segmentation_topicID : %s", param_segmentation_topicID.data());
    }
}
