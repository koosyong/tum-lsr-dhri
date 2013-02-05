#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// PCTracking
#include "pctracking.h"
#include "pcpublisher.h"

ros::Publisher pub_track;
ros::Publisher pub_trackID;
ros::Publisher pub_model;
ros::Publisher pub_scene;
ros::Publisher pub_gmms;
ros::Publisher pub_edges;

void publishPointCloud(vnl_matrix<double> _transformed);

PCTracking pctracking(&pub_scene, &pub_model, &publishPointCloud);

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
    pctracking.run(pCloud);


    // output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudOut;
    pCloudOut.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    PCPublisher publisher(pctracking.tracks);
    publisher.toPointCloudXYZI(*pCloudOut);

    //    ROS_INFO("# of track points: %d", pCloudOut->points.size());

    // publish pointcloud(currentT) of tracks
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pCloudOut, output);
    output.header.frame_id="/origin";
    pub_track.publish (output);

    // publish gmms
    pub_gmms.publish(publisher.toMarkerGMMs());
    pub_edges.publish(publisher.toMarkerEdges());
//    pub_gmms.publish(pctracking.tracks->toMarkerGMMs());
//    pub_edges.publish(pctracking.tracks->toMarkerEdges());

    // publish delete id marker of deleted tracks
    visualization_msgs::MarkerArray trackIDs;
    vector<int> deletedTrackIDs = pctracking.tracks->deletedTrackIDs;
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
    for(int i=0;i<pctracking.tracks->numTracks();i++){
        Point centroid = pctracking.tracks->tracks.at(i)->lastFrame().object.getCentroid();
        int id = pctracking.tracks->tracks.at(i)->id;

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
        trackID.color.r = 1.0;
        trackID.color.g = 1.0;
        trackID.color.b = 1.0;
        trackID.text = sID;
        trackID.scale.z = 0.05;
        //        trackID.
        trackIDs.markers.push_back(trackID);
    }

    pub_trackID.publish(trackIDs);

    // delete
    pCloud.reset();
    pCloudOut.reset();
//    ROS_INFO("-------------------------------------------------------------");

    cnt++;

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "tracking_gmm");
    ros::NodeHandle n;
    ros::Subscriber sub_id = n.subscribe("/ckinect/pcloud/cutted", 1000, cb_rgb);
//    ros::Subscriber sub_id = n.subscribe("/ckinect/points/merged", 1000, cb_rgb);


    pub_track = n.advertise<sensor_msgs::PointCloud2>("/ckinect/pcloud/trackID", 1000);
    pub_model = n.advertise<sensor_msgs::PointCloud2>("/ckinect/pcloud/model", 1000);
    pub_scene = n.advertise<sensor_msgs::PointCloud2>("/ckinect/pcloud/scene", 1000);
    pub_trackID = n.advertise<visualization_msgs::MarkerArray>("/ckinect/marker/trackID", 1000);
    pub_gmms = n.advertise<visualization_msgs::MarkerArray>("/ckinect/pclud/gmms", 1000);
    pub_edges = n.advertise<visualization_msgs::Marker>("/ckinect/pclud/edges", 1000);
    ros::spin();
    return 0;
}

