#include "pcpublisher.h"

PCPublisher::PCPublisher()
{
}

PCPublisher::PCPublisher(shared_ptr< TrackContainer<PCObject> > _tracks)
{
    tracks = _tracks;
}


void PCPublisher::toPointCloudXYZI(Cloud &cloudOut)
{
    for(int i=0;i<tracks->numTracks();i++){
        if(tracks->tracks.at(i)->lastFrame().time == tracks->currentT){
            PCObject object;
            object = tracks->tracks.at(i)->lastFrame().object;
            int id = tracks->tracks.at(i)->id;
            for(int j=0;j<object.points.size();j++){
                PointT point;
                point.x = object.points.at(j).pos[0];
                point.y = object.points.at(j).pos[1];
                point.z = object.points.at(j).pos[2];
                point.r = tracks->r[id];
                point.g = tracks->g[id];
                point.b = tracks->b[id];
//                point.intensity = id;
                cloudOut.points.push_back(point);
            }
        }
    }
}

visualization_msgs::Marker PCPublisher::toMarkerEdges()
{
    visualization_msgs::Marker edgeMarker;

    edgeMarker.header.frame_id = "/origin";
    edgeMarker.header.stamp = ros::Time();
    edgeMarker.ns = "edge";
    edgeMarker.id = 1;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.lifetime = ros::Duration(0);
    edgeMarker.scale.x = 0.003;
    edgeMarker.color.a = 1;
    edgeMarker.color.r = 1;
    edgeMarker.color.g = 1;
    edgeMarker.color.b = 1;


    for(int i=0;i<tracks->numTracks();i++){
        if(tracks->tracks.at(i)->lastFrame().time == tracks->currentT){
            PCObject object;
            object = tracks->tracks.at(i)->lastFrame().object;
            for(int j=0;j<object.edges.size();j++){

                Point p = object.edges.at(j).u;
                geometry_msgs::Point point;
                point.x = p.pos[0];
                point.y = p.pos[1];
                point.z = p.pos[2];
//                cout<<p.pos<<endl;
                edgeMarker.points.push_back(point);
                p = object.edges.at(j).v;
                point.x = p.pos[0];
                point.y = p.pos[1];
                point.z = p.pos[2];
//                cout<<p.pos<<endl;
                edgeMarker.points.push_back(point);
            }
        }
    }
    return edgeMarker;
}

visualization_msgs::MarkerArray PCPublisher::toMarkerGMMs()
{

    double margin = 0.1;
    double stepsize = 0.01;

    visualization_msgs::MarkerArray gmmMarkers;
    int cnt = 0;
    for(int i=0;i<tracks->numTracks();i++){
//        if(tracks.at(i)->lastFrame().time == currentT){
            PCObject object;
            object = tracks->tracks.at(i)->lastFrame().object;
            int id = tracks->tracks.at(i)->id;
            // make a gmm eval points of the object
            for(int j=0;j<object.gmm.size();j++){
                cnt ++;
                visualization_msgs::Marker gmmMarker;
                gmmMarker.header.frame_id = "/origin";
                gmmMarker.header.stamp = ros::Time();
                gmmMarker.ns = "gmm";
                gmmMarker.id = cnt;

                Gaussian gmm = object.gmm.at(j);
                gmmMarker.type = visualization_msgs::Marker::SPHERE;
                gmmMarker.action = visualization_msgs::Marker::ADD;
                gmmMarker.lifetime = ros::Duration(0);
                gmmMarker.pose.position.x = gmm.mean[0];
                gmmMarker.pose.position.y = gmm.mean[1];
                gmmMarker.pose.position.z = gmm.mean[2];


                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(gmm.covariance);
                Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
                Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();

                Eigen::Matrix3d rotation = eigenvectors;

                double r11 = rotation.col(0)[0];
                double r21 = rotation.col(0)[1];
                double r31 = rotation.col(0)[2];
                double r12 = rotation.col(1)[0];
                double r22 = rotation.col(1)[1];
                double r32 = rotation.col(1)[2];
                double r13 = rotation.col(2)[0];
                double r23 = rotation.col(2)[1];
                double r33 = rotation.col(2)[2];


                double q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
                double q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
                double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
                double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
                if(q0 < 0.0f) q0 = 0.0f;
                if(q1 < 0.0f) q1 = 0.0f;
                if(q2 < 0.0f) q2 = 0.0f;
                if(q3 < 0.0f) q3 = 0.0f;
                q0 = sqrt(q0);
                q1 = sqrt(q1);
                q2 = sqrt(q2);
                q3 = sqrt(q3);
                if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
                    q0 *= +1.0f;
                    q1 *= SIGN(r32 - r23);
                    q2 *= SIGN(r13 - r31);
                    q3 *= SIGN(r21 - r12);
                } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
                    q0 *= SIGN(r32 - r23);
                    q1 *= +1.0f;
                    q2 *= SIGN(r21 + r12);
                    q3 *= SIGN(r13 + r31);
                } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
                    q0 *= SIGN(r13 - r31);
                    q1 *= SIGN(r21 + r12);
                    q2 *= +1.0f;
                    q3 *= SIGN(r32 + r23);
                } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
                    q0 *= SIGN(r21 - r12);
                    q1 *= SIGN(r31 + r13);
                    q2 *= SIGN(r32 + r23);
                    q3 *= +1.0f;
                } else {
                    printf("coding error\n");
                }
                float rq = NORM(q0, q1, q2, q3);
                q0 /= rq;
                q1 /= rq;
                q2 /= rq;
                q3 /= rq;

                gmmMarker.pose.orientation.w = q0;
                gmmMarker.pose.orientation.x = q1;
                gmmMarker.pose.orientation.y = q2;
                gmmMarker.pose.orientation.z = q3;

                gmmMarker.frame_locked = 0;

                gmmMarker.scale.x = sqrt(eigenvalues[0])*2;
                gmmMarker.scale.y = sqrt(eigenvalues[1])*2;
                gmmMarker.scale.z = sqrt(eigenvalues[2])*2;

//                std::cout<<" scalex "<<gmmMarker.scale.x;
//                std::cout<<" scaley "<<gmmMarker.scale.y;
//                std::cout<<" scalez "<<gmmMarker.scale.z;
//                std::cout<<std::endl;
//                gmmMarker.scale.x = 0.02;
//                gmmMarker.scale.y = 0.02;
//                gmmMarker.scale.z = 0.02;

                gmmMarker.color.a = gmm.weight*object.gmm.size()/2;
//                gmmMarker.color.a = 1.0;
                gmmMarker.color.r = ((double)tracks->r[id])/256;
                gmmMarker.color.g = ((double)tracks->g[id])/256;
                gmmMarker.color.b = ((double)tracks->b[id])/256;

//                gmmMarker.color.r = 1.;
//                gmmMarker.color.g = 1.;
//                gmmMarker.color.b = 1.;

                gmmMarkers.markers.push_back(gmmMarker);

            }
            // delete old marker
            if(cnt < tracks->oldCnt){
                for(int j=cnt+1;j<=tracks->oldCnt;j++){
                    visualization_msgs::Marker gmmMarker;
                    gmmMarker.header.frame_id = "/origin";
                    gmmMarker.header.stamp = ros::Time();
                    gmmMarker.ns = "gmm";
                    gmmMarker.id = j;
                    gmmMarker.type = visualization_msgs::Marker::SPHERE;
                    gmmMarker.action = visualization_msgs::Marker::DELETE;
                    gmmMarker.lifetime = ros::Duration(0);
                    gmmMarkers.markers.push_back(gmmMarker);
                }
            }
            tracks->oldCnt = cnt;
//        }
    }
    return gmmMarkers;
}
