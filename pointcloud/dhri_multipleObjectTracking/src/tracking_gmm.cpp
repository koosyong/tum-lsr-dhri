#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// PCTracking
#include "pctracking.h"

ros::Publisher pub_track;
ros::Publisher pub_trackID;
ros::Publisher pub_model;
ros::Publisher pub_scene;
ros::Publisher pub_gmms;
ros::Publisher pub_edges;

double cont_sampling;
double cont_simplify;

int  sumTotalPoint = 0;
int sumTruePoint  = 0;
int sumErrorPoint  = 0;
double computationT = 0;

void publishPointCloud(vnl_matrix<double> _transformed);

PCTracking *pctracking;

void publishPointCloud(vnl_matrix<double> _transformed)
{
    PCObject obj;
    for(int i=0;i<_transformed.rows();i++){
        Point p;
        p.pos[0] = _transformed.get(i,0);
        p.pos[1] = _transformed.get(i,1);
        p.pos[2] = _transformed.get(i,2);
        obj.insert(p);
    }
    //    obj.initialGMM(scale);
    // output
    CloudPtr pCloudOut;
    pCloudOut.reset(new Cloud);

    for(int j=0;j<obj.points.size();j++){
        PointT point;
        point.x = obj.points.at(j).pos[0];
        point.y = obj.points.at(j).pos[1];
        point.z = obj.points.at(j).pos[2];
        pCloudOut->points.push_back(point);
    }

    //    ROS_INFO("# of points: %d", pCloudOut->points.size());

    // publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pCloudOut, output);
    output.header.frame_id="/origin";
    pub_model.publish (output);

    // delete
    pCloudOut.reset();
}


void cb_rgb(const sensor_msgs::PointCloud2ConstPtr& input)
{
        ROS_INFO("I heard RGB");
    static int cnt = 0;

    // input
    CloudPtr pCloud;
    pCloud.reset(new Cloud);
    pcl::fromROSMsg (*input, *pCloud);

    // tracking
    double lastTime = pcl::getTime ();
    pctracking->run(pCloud);
    double now = pcl::getTime ();

    computationT += (now-lastTime);

    // output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudOut;
    pCloudOut.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    pctracking->tracks->toPointCloudXYZI(*pCloudOut);
    //    ROS_INFO("# of track points: %d", pCloudOut->points.size());

    // publish pointcloud(currentT) of tracks
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pCloudOut, output);
    output.header.frame_id="/origin";
    pub_track.publish (output);

    // publish gmms
    pub_gmms.publish(pctracking->tracks->toMarkerGMMs());
    pub_edges.publish(pctracking->tracks->toMarkerEdges());

    // publish delete id marker of deleted tracks
    visualization_msgs::MarkerArray trackIDs;
    vector<int> deletedTrackIDs = pctracking->tracks->deletedTrackIDs;
    for(int i=0;i<deletedTrackIDs.size();i++){
        visualization_msgs::Marker trackID;
        trackID.header.frame_id = "/origin";
        trackID.header.stamp = ros::Time();
        trackID.ns = "id";
        trackID.id = deletedTrackIDs.at(i);
        trackID.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trackID.action = visualization_msgs::Marker::DELETE;
        trackIDs.markers.push_back(trackID);
    }
    // publish ids of existing all tracks
    //    trackIDs.markers.clear();
    for(int i=0;i<pctracking->tracks->numTracks();i++){
        Point centroid = pctracking->tracks->tracks.at(i)->lastFrame().object.getCentroid();
        int id = pctracking->tracks->tracks.at(i)->id;

        stringstream strm;
        string sID;
        strm << id;
        strm >> sID;

        visualization_msgs::Marker trackID;
        trackID.header.frame_id = "/origin";
        trackID.header.stamp = ros::Time();
        trackID.ns = "id";
        trackID.id = id;
        trackID.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trackID.action = visualization_msgs::Marker::ADD;
        trackID.lifetime = ros::Duration(0);
        trackID.pose.position.x = centroid.pos[0];
        trackID.pose.position.y = centroid.pos[1];
        trackID.pose.position.z = centroid.pos[2];
        trackID.pose.orientation.x = 0.0;
        trackID.pose.orientation.y = 0.0;
        trackID.pose.orientation.z = 0.0;
        trackID.pose.orientation.w = 1.0;
        trackID.color.a = 1.0;
        trackID.color.r = 0.0;
        trackID.color.g = 0.0;
        trackID.color.b = 0.0;
        trackID.text = sID;
        trackID.scale.z = 0.05;
        //        trackID.
        trackIDs.markers.push_back(trackID);
    }

    pub_trackID.publish(trackIDs);
    // delete
    pCloud.reset();
    pCloudOut.reset();
    cnt++;

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "dhri_multipleObjectTracking");

    // read parameters
    string param_sub_topic;
    string param_pub_points_trackID;
    string param_pub_markers_trackID, param_pub_markers_gmms, param_pub_markers_edges;
    string param_pub_gmmreg_model, param_pub_gmmreg_scene;
    int param_3d6d;
    double param_samplingRatio, param_simplifyRatio, param_segmentTolerance;

    bool isOK = 1;
    if(ros::param::has("/dhri/multipleObjectTracking/sub/topic"))
        ros::param::get("/dhri/multipleObjectTracking/sub/topic", param_sub_topic);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/points/trackID"))
        ros::param::get("/dhri/multipleObjectTracking/pub/points/trackID", param_pub_points_trackID);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/markers/trackID"))
        ros::param::get("/dhri/multipleObjectTracking/pub/markers/trackID", param_pub_markers_trackID);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/markers/gmms"))
        ros::param::get("/dhri/multipleObjectTracking/pub/markers/gmms", param_pub_markers_gmms);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/markers/edges"))
        ros::param::get("/dhri/multipleObjectTracking/pub/markers/edges", param_pub_markers_edges);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/gmmreg/model"))
        ros::param::get("/dhri/multipleObjectTracking/pub/gmmreg/model", param_pub_gmmreg_model);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/gmmreg/scene"))
        ros::param::get("/dhri/multipleObjectTracking/pub/gmmreg/scene", param_pub_gmmreg_scene);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/3d6d"))
        ros::param::get("/dhri/multipleObjectTracking/param/3d6d", param_3d6d);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/samplingRatio"))
        ros::param::get("/dhri/multipleObjectTracking/param/samplingRatio", param_samplingRatio);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/siimplifyRatio"))
        ros::param::get("/dhri/multipleObjectTracking/param/siimplifyRatio", param_simplifyRatio);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/segmentTolerance"))
        ros::param::get("/dhri/multipleObjectTracking/param/segmentTolerance", param_segmentTolerance);
    else isOK = 0;

    if(isOK){
    }
    else{
        ROS_WARN("Run 'rosparam load param.yaml' first");
        return 0;
    }

    cont_sampling = param_samplingRatio / 1000.;
    cont_simplify = param_simplifyRatio / 100.;

    ros::NodeHandle n;
    ros::Subscriber sub_id = n.subscribe(param_sub_topic.data(), 1, cb_rgb);

    pub_track = n.advertise<sensor_msgs::PointCloud2>(param_pub_points_trackID.data(), 1000);
    pub_trackID = n.advertise<visualization_msgs::MarkerArray>(param_pub_markers_trackID.data(), 1000);
    pub_gmms = n.advertise<visualization_msgs::MarkerArray>(param_pub_markers_gmms.data(), 1000);
    pub_edges = n.advertise<visualization_msgs::Marker>(param_pub_markers_edges.data(), 1000);

    pub_model = n.advertise<sensor_msgs::PointCloud2>(param_pub_gmmreg_model.data(), 1000);
    pub_scene = n.advertise<sensor_msgs::PointCloud2>(param_pub_gmmreg_scene.data(), 1000);



    pctracking = new PCTracking(&pub_scene, &pub_model, &publishPointCloud, param_3d6d, cont_sampling, cont_simplify, param_segmentTolerance);
    ros::spin();

    delete pctracking;
    return 0;
}

