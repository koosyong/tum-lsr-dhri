#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// PCTracking
#include "pcobjectcontainer.h"
#include "pctrackcontainer.h"
#include "imft.h"

double weight_l2(PCObject &o1, PCObject &o2);
double weight_energy2(PCObject &o1, PCObject &o2);
double weight_l2_ifgt(PCObject &o1, PCObject &o2);
double weight_cov(PCObject &o1, PCObject &o2);
double weight_l2_gmm1(PCObject &o1, PCObject &o2);

PCTrackContainer trackContainer(10);
IMFT<PCObject> imft(2, 15, &weight_l2, &weight_l2);

ros::Publisher pub_track;
ros::Publisher pub_trackID;
ros::Publisher pub_model;
ros::Publisher pub_scene;

PCObject f1, f2, g1;

// gmmreg
#include "gmmreg_rigid.h"


#include <Eigen/Dense>
#include <Eigen/LU>

typedef std::auto_ptr<gmmreg_rigid> gmmreg_rigid_Ptr;

#define SQR(X)  ((X)*(X))
#define pi 3.141592
typedef struct{
    Eigen::Matrix3d covariance;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    Eigen::Vector3d mean;
} GAUSS1;

GAUSS1 gauss1(PCObject &o)
{
    double covMin = 0.01;
    GAUSS1 gauss;
    int size = o.points.size();

    // mean
    double x = 0.;
    double y = 0.;
    double z = 0.;
    for(int j=0;j<size;j++){
        x += o.points.at(j).x;
        y += o.points.at(j).y;
        z += o.points.at(j).z;
    }
    gauss.mean[0] = x/(double)size;
    gauss.mean[1] = y/(double)size;
    gauss.mean[2] = z/(double)size;

    // covariance
    Eigen::Vector3d demean;
    Eigen::Vector3d xyz_centroid = gauss.mean;
    Eigen::Vector3d point;
    gauss.covariance.setZero();

    double demean_xy, demean_xz, demean_yz;
    // For each point in the cloud
    for (int idx = 0; idx < size; ++idx)
    {
        point[0] = o.points.at(idx).x;
        point[1] = o.points.at(idx).y;
        point[2] = o.points.at(idx).z;
        demean = point - xyz_centroid;

        demean_xy = demean[0] * demean[1];
        demean_xz = demean[0] * demean[2];
        demean_yz = demean[1] * demean[2];

        gauss.covariance(0, 0) += demean[0] * demean[0];
        gauss.covariance(0, 1) += demean_xy;
        gauss.covariance(0, 2) += demean_xz;

        gauss.covariance(1, 0) += demean_xy;
        gauss.covariance(1, 1) += demean[1] * demean[1];
        gauss.covariance(1, 2) += demean_yz;

        gauss.covariance(2, 0) += demean_xz;
        gauss.covariance(2, 1) += demean_yz;
        gauss.covariance(2, 2) += demean[2] * demean[2];
    }
    gauss.covariance = gauss.covariance / (double)size;


    //        std::cout << statistic.covariance << std::endl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(gauss.covariance);
    gauss.eigenvalues = eigensolver.eigenvalues();
    gauss.eigenvectors = eigensolver.eigenvectors();    // find an eigenvactor by calling .col(i)

    return gauss;
}

double weight_cov(PCObject &o1, PCObject &o2)
{
    if(o1.points.size() <= 1 || o2.points.size() <= 1)
        return 0.0;

    double maxX = 0.5;
    double maxY = 0.5;
    double maxZ = 0.5;
    double maxCovDist = 10;
    double alpha = 0.8; // weighting for mean

    // covariance matrix
    GAUSS1 o1_gauss = gauss1(o1);
    GAUSS1 o2_gauss = gauss1(o2);

    Eigen::Matrix3d cov1 = o1_gauss.covariance;
    Eigen::Matrix3d cov2 = o2_gauss.covariance;

    //        Eigen::Matrix3f A;
    // sqrt(inv(A))*B*sqrt(inv(A))
    Eigen::Matrix3d sqrtM, C;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver1(cov1.inverse());
    sqrtM = eigensolver1.operatorSqrt();
    C = sqrtM * cov2 * sqrtM;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver2(C);
    Eigen::Vector3d eigenvalues = eigensolver2.eigenvalues();
    double sum = 0;
    for(int j=0;j<3;j++){
        //            std::cout << log(eigenvalues[j]) << std::endl;
        sum += pow(log(eigenvalues[j]),2);
    }
    double covDist = 1 - sqrt(sum) / maxCovDist;
    double meanDist = 1 - sqrt(pow(o1_gauss.mean[0]-o2_gauss.mean[0],2) + pow(o1_gauss.mean[1]-o2_gauss.mean[1],2) + pow(o1_gauss.mean[2]-o2_gauss.mean[2],2))
            / sqrt(maxX*maxX + maxY*maxY + maxZ*maxZ);

    //    double alpha = 0.5;
    double weight = alpha * meanDist + (1-alpha) * covDist;

    return weight;
}

double weight_l2_gmm1(PCObject &o1, PCObject &o2)
{

    if(o1.points.size() <= 1 || o2.points.size() <= 1)
        return 0.0;

    double maxDist = 13.;
    // covariance matrix
    GAUSS1 o1_gauss = gauss1(o1);
    GAUSS1 o2_gauss = gauss1(o2);

    Eigen::Matrix3d cov1 = o1_gauss.covariance;
    Eigen::Matrix3d cov2 = o2_gauss.covariance;
    Eigen::Vector3d mean1 = o1_gauss.mean;
    Eigen::Vector3d mean2 = o2_gauss.mean;
    Eigen::Matrix3d cov;
    Eigen::Vector3d mean;
    double det;
    Eigen::Matrix3d inv;
    double a;

    cov = cov1+cov1;
    mean = mean1-mean1;
    det = cov.determinant();
    if(det<0.01) det = 0.0001;
    inv = cov.inverse();
    a = mean.transpose()*inv*mean;
    double energy1 = 1/sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    printf("energy1 %f\n", energy1);

    // f(x)g(x)
    cov = cov1+cov2;
    mean = mean1-mean2;
    det = cov.determinant();
    if(det<0.01) det = 0.0001;
    inv = cov.inverse();
    a = mean.transpose()*inv*mean;
    double energy2 = 1/sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    printf("energy2 %f\n", energy2);

    // g(x)^2
    cov = cov2+cov2;
    mean = mean2-mean2;
    det = cov.determinant();
    if(det<0.01) det = 0.0001;
    inv = cov.inverse();
    a = mean.transpose()*inv*mean;
    double energy3 = 1/sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    printf("energy3 %f\n", energy3);
    printf("weight %f\n", (energy1 - 2*energy2 + energy3));

    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
    //    return (energy1 - 2*energy2 + energy3);
}

double weight_l2(PCObject &o1, PCObject &o2)
{
    double maxDist = 0.8;
    if(o1.points.size() <= 0 || o2.points.size() <= 0)
        return 0.;

    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010

    double energy1 = 0;
    double energy2 = 0;
    double energy3 = 0;
    double cross_term;
    double scale = 0.05;
    int m = o1.points.size();
    int n = o2.points.size();

    //    scale = SQR(scale);
    cross_term = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o1.points[i].x - o1.points[j].x);
            dist_ij += SQR(o1.points[i].y - o1.points[j].y);
            dist_ij += SQR(o1.points[i].z - o1.points[j].z);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy1 = cross_term / (m * m) / (scale*2*pi*sqrt(4*pi));
    //    printf("energy1 %f\n", energy1);
    cross_term = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o1.points[i].x - o2.points[j].x);
            dist_ij += SQR(o1.points[i].y - o2.points[j].y);
            dist_ij += SQR(o1.points[i].z - o2.points[j].z);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy2 = cross_term / (m * n) / (scale*2*pi*sqrt(4*pi));
    //    printf("energy2 %f\n", energy2);
    cross_term = 0.;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o2.points[i].x - o2.points[j].x);
            dist_ij += SQR(o2.points[i].y - o2.points[j].y);
            dist_ij += SQR(o2.points[i].z - o2.points[j].z);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy3 = cross_term / (n * n) / (scale*2*pi*sqrt(4*pi));
    //    printf("energy3 %f\n", energy3);
    //    printf("weight %f\n", (maxDist - (energy1 - 2*energy2 + energy3))/maxDist);
    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
    //    return energy2;
}


double weight_energy2(PCObject &o1, PCObject &o2)
{
    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010

    double energy1 = 0;
    double energy2 = 0;
    double energy3 = 0;
    double cross_term;
    double scale = 0.01;
    int m = o1.points.size();
    int n = o2.points.size();

    cross_term = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o1.points[i].x - o2.points[j].x);
            dist_ij += SQR(o1.points[i].y - o2.points[j].y);
            dist_ij += SQR(o1.points[i].z - o2.points[j].z);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy2 = cross_term / (m * n) / (scale*2*pi*sqrt(2*pi));
    return energy2;
    //    return energy2;
}

double evalPoint2Object(PCObject::Point point, PCObject &object)
{
    double scale = 0.05;
    int n = object.points.size();

    double cross_term = 0.;
    for (int i = 0; i < n; ++i) {
        double dist_ij = 0;
        dist_ij += SQR(object.points[i].x - point.x);
        dist_ij += SQR(object.points[i].y - point.y);
        dist_ij += SQR(object.points[i].z - point.z);
        double cost_ij = exp(-0.5 * dist_ij / SQR(scale)*2);
        cross_term += cost_ij;
    }
    return cross_term / n/ (scale*2*pi*sqrt(2*pi));
}

void publishPointCloud(vnl_matrix<double> _transformed)
{
    PCObject obj;
    for(int i=0;i<_transformed.rows();i++){
        PCObject::Point p;
        p.x = _transformed.get(i,0);
        p.y = _transformed.get(i,1);
        p.z = _transformed.get(i,2);
        obj.insert(p);
    }
    // output
    CloudPtr pCloudOut;
    pCloudOut.reset(new Cloud);

    for(int j=0;j<obj.points.size();j++){
        PointT point;
        point.x = obj.points.at(j).x;
        point.y = obj.points.at(j).y;
        point.z = obj.points.at(j).z;
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

typedef vector<PCObject*> VecObjectPtr;
typedef vector<PCObject> VecObject;
typedef struct{
    VecObjectPtr models;
    PCObject* scene;
    VecObject trasformedModels;
} Scene;

void cb_id(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //    ROS_INFO("I heard id");

    // input
    CloudPtr pCloud;
    pCloud.reset(new Cloud);
    pcl::fromROSMsg (*input, *pCloud);

    // processing
    // making objects
    PCObjectContainer objectContainer(pCloud);
    //    ROS_INFO("# of objects: %d", objectContainer.numObjects());

    // tracking
    static int cnt = 0;
    bool isMerged = 0;
    bool isSeparated = 0;
    imft.setFrame(objectContainer.objects, cnt);
//        imft.confirmDGraph();
    imft.matching();
    imft.updateTracks();
//        imft.confirmDGraph();

    // modify point clouds

    // Mergence of unmatched objects
    // get unmatched object
    IMFT<PCObject>::VecObjectPtr unmatchedObjects = imft.getUnmatchedObjects();
    if(unmatchedObjects.size() != 0){
//        ROS_INFO("# of unmatched objects: %d", unmatchedObjects.size());
        for(int i=0;i<unmatchedObjects.size();i++){
//            ROS_INFO("Points of object %d: %d", i, unmatchedObjects.at(i)->points.size());
            // find a best matched track
            IMFT<PCObject>::ObjectPtr maxTrack, objectOrigin;
            double wOrigin, wMerge, wHypothesis;
            imft.getMaximumMatchedTrack(unmatchedObjects.at(i), maxTrack, wHypothesis, objectOrigin, wOrigin);
//            ROS_INFO("maxTrack points %d, weight %f, origin points %d, weights %f", maxTrack->points.size(), wHypothesis, objectOrigin->points.size(), wOrigin);

            // make a merged object
            PCObject mergeObject;
            for(int j=0;j<unmatchedObjects.at(i)->points.size();j++){
                mergeObject.insert(unmatchedObjects.at(i)->points.at(j));
            }
            for(int j=0;j<objectOrigin->points.size();j++){
                mergeObject.insert(objectOrigin->points.at(j));
            }

//            ROS_INFO("points of merged object : %d", mergeObject.points.size());

            // weight for merged object and track
            wMerge = weight_l2(mergeObject, *maxTrack);
//            ROS_INFO("weight for merged object and track : %f", wMerge);

            // test of mergence
//            if(wMerge > wOrigin*0.8 && wMerge > wHypothesis*0.8 && wHypothesis > 0){
                if(wHypothesis > 0.8 || wOrigin > 0.85){
                // delete origin object and unmatched object: objectOrigin
                objectContainer.deleteObject(objectOrigin->id);
                objectContainer.deleteObject(unmatchedObjects.at(i)->id);
                // put a merged object : mergeObject
                objectContainer.makeNewObject(mergeObject);
//                ROS_INFO("merged object is set");
                isMerged = 1;
            }
                if(isMerged){
                    // delete frame
//                    imft.confirmDGraph();
                    imft.deleteLastFrame();
                    // set modified point clouds
                    imft.cnt--;
                    imft.setFrame(objectContainer.objects, cnt);
                    imft.matching();
                    imft.updateTracks();
//                    imft.confirmDGraph();
                }
        }
    }



    // Separation of object for unmatched track
    //    IMFT<PCObject>::VecObjectPtr terminalNodes = imft.getTerminalNodes();
    //    IMFT<PCObject>::VecObjectPtr terminalNodes = imft.getTerminalNodesLastFrame();
    IMFT<PCObject>::VecObjectPtr terminalNodes = imft.getUnmatchedTracks();
    vector<Scene> scenes;
    if(terminalNodes.size() != 0){
//        ROS_INFO("# of unmatched tracks: %d", terminalNodes.size());
        for(int i=0;i<terminalNodes.size();i++){
//            ROS_INFO("Points of track %d: %d", i, terminalNodes.at(i)->points.size());
            // find a best matched object
            IMFT<PCObject>::ObjectPtr maxObject, trackOrigin, trackUnmatched;
            trackUnmatched = terminalNodes.at(i);
            double wHypothesis, wOrigin, wMerge;
            imft.getMaximumMatchedObject(trackUnmatched, maxObject, wHypothesis, trackOrigin, wOrigin);
//            ROS_INFO("maxObject points %d, weight %f, origin points %d, weights %f", maxObject->points.size(), wHypothesis, trackOrigin->points.size(), wOrigin);

            // make a merged object
            PCObject mergeObject;
            for(int j=0;j<trackUnmatched->points.size();j++){
                mergeObject.insert(trackUnmatched->points.at(j));
            }
            for(int j=0;j<trackOrigin->points.size();j++){
                mergeObject.insert(trackOrigin->points.at(j));
            }
//            ROS_INFO("points of merged object : %d", mergeObject.points.size());

            // weight for merged object and track
            wMerge = weight_l2(mergeObject, *maxObject);
//            ROS_INFO("weight for merged object and maxObject : %f", wMerge);

            // test of separation
            if(wMerge > wOrigin*0.9 && wMerge > wHypothesis*0.9 && wHypothesis > 0){
//            if(wHypothesis > 0.7){
                // make a new
                bool isSceneExist = 0;
                for(int k=0;k<scenes.size();k++){
                    if(scenes.at(k).scene->id == maxObject->id){
                        isSceneExist = 1;
                        // put models in the scenes.at(i)
                        // models : trackOrigin, trackUnmatched
                        // check whether the trackOrigin exists in the models
                        bool isOriginExist = 0;
                        for(int l=0;l<scenes.at(k).models.size();l++){
                            if(scenes.at(k).models.at(l)->id == trackOrigin->id){
                                isOriginExist = 1;
                                break;
                            }
                        }
                        // put trackOrigin into models
                        if(!isOriginExist)
                            scenes.at(k).models.push_back(trackOrigin);
                        // put trackUnmatched into models
                        scenes.at(k).models.push_back(trackUnmatched);
                        break;
                    }
                }
                if(!isSceneExist){
                    Scene newScene;
                    newScene.scene = maxObject;
                    // put trackOrigin into models
                    newScene.models.push_back(trackOrigin);
                    // put trackUnmatched into models
                    newScene.models.push_back(trackUnmatched);
                    scenes.push_back(newScene);
                }

                //                objectContainer.makeNewObject(mergeObject);
                //                ROS_INFO("merged object is set");
                //
            }
        }
        if(scenes.size() != 0){
            isSeparated = 1;
            for(int i=0;i<scenes.size();i++){
                Scene scene = scenes.at(i);
//                ROS_INFO("#### Scene : %d", i);

                // publish scene point cloud
                CloudPtr pCloudOut_scene;
                pCloudOut_scene.reset(new Cloud);
                for(int j=0;j<scene.scene->points.size();j++){
                    PointT point;
                    point.x = scene.scene->points.at(j).x;
                    point.y = scene.scene->points.at(j).y;
                    point.z = scene.scene->points.at(j).z;
                    pCloudOut_scene->points.push_back(point);
                }
                sensor_msgs::PointCloud2 output_scene;
                pcl::toROSMsg(*pCloudOut_scene, output_scene);
                output_scene.header.frame_id="/origin";
                pub_scene.publish (output_scene);
                pCloudOut_scene.reset();

                for(int j=0;j<scene.models.size();j++){
//                    ROS_INFO("########### model : %d", j);
                    PCObject* model = scene.models.at(j);
                    // registration model to scene

                    vnl_matrix<double> m(model->points.size(), 3), s(scene.scene->points.size(), 3), c(model->points.size(), 3);
                    for(int i=0;i<model->points.size();i++){
                        m.put(i,0,model->points[i].x);
                        m.put(i,1,model->points[i].y);
                        m.put(i,2,model->points[i].z);
                    }
                    for(int i=0;i<scene.scene->points.size();i++){
                        s.put(i,0,scene.scene->points[i].x);
                        s.put(i,1,scene.scene->points[i].y);
                        s.put(i,2,scene.scene->points[i].z);
                    }
                    gmmreg_rigid_Ptr gmmreg(new gmmreg_rigid(&publishPointCloud));
                    gmmreg->init(m, s, c, 1);
                    gmmreg->rigidInit(1, "0.01 .2 .1 0.01", "100 200 300 100");
                    gmmreg->run();

                    PCObject trasformedModel;
                    for(int i=0;i<gmmreg->transformed_model.rows();i++){
                        PCObject::Point p;
                        p.x = gmmreg->transformed_model.get(i,0);
                        p.y = gmmreg->transformed_model.get(i,1);
                        p.z = gmmreg->transformed_model.get(i,2);
                        trasformedModel.insert(p);
                    }
                    scene.trasformedModels.push_back(trasformedModel);

                }
                // separate the scene points to the transformed models
                int nClass = scene.models.size();
                PCObject *newObjects;
                newObjects = new PCObject[nClass];
                for(int n=0;n<scene.scene->points.size();n++){
                    PCObject::Point point = scene.scene->points.at(n);
                    double *closeness = new double[nClass];
                    // find maxCloseness
                    double max = 0.;
                    int maxObject = 0;
                    for(int o=0;o<nClass;o++){
                        closeness[o] = evalPoint2Object(point, scene.trasformedModels.at(o));
//                        closeness[o] = evalPoint2Object(point, *scene.models.at(o));
                        if(closeness[o] >= max){
                            max = closeness[o];
                            maxObject = o;
                        }
                    }
                    newObjects[maxObject].insert(point);
                    delete[] closeness;
                }
                // delete scene from the objects
                objectContainer.deleteObject(scene.scene->id);
//                ROS_INFO("delete a object");

                // put new objects
                for(int o=0;o<nClass;o++){
                    objectContainer.makeNewObject(newObjects[o]);
//                    ROS_INFO("make new object");
                }

                delete[] newObjects;
            }

            // publish empty scene and model point set
            CloudPtr pCloudOut_empty;
            pCloudOut_empty.reset(new Cloud);
            PointT point;
            point.x = 0.;
            point.y = 0.;
            point.z = 0.;
            pCloudOut_empty->points.push_back(point);
            sensor_msgs::PointCloud2 output_empty;
            pcl::toROSMsg(*pCloudOut_empty, output_empty);
            output_empty.header.frame_id="/origin";
            pub_scene.publish (output_empty);
            pub_model.publish (output_empty);
            pCloudOut_empty.reset();
        }
    }

    if(isSeparated){
        // delete frame
        imft.deleteLastFrame();
        // set modified point clouds
        imft.cnt--;
        imft.setFrame(objectContainer.objects, cnt);
        imft.matching();
        imft.updateTracks();
        //        imft.confirmDGraph();
    }

    //    imft.confirmDGraph();
    // set modified point clouds
    // update tracks
//    imft.updateTracks();
    //    imft.confirmDGraph();

    // extrack tracks from imft
    PCTrackContainer *imftTracks;
    imftTracks = (PCTrackContainer*)imft.extractTracks();
//    ROS_INFO("# of tracks: %d", imftTracks->numTracks());


    //    imft.deleteFrame(cnt);
    //    imft.setFrame(objectContainer.objects, cnt);
    //    imft.confirmDGraph();

    // update tracks

    cnt ++;


    // output
    CloudPtr pCloudOut;
    pCloudOut.reset(new Cloud);
    imftTracks->toPointCloudXYZI(*pCloudOut);
    //    ROS_INFO("# of track points: %d", pCloudOut->points.size());

    // publish pointcloud(currentT) of tracks
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pCloudOut, output);
    output.header.frame_id="/origin";
    pub_track.publish (output);

    // publish delete id marker of deleted tracks
    visualization_msgs::MarkerArray trackIDs;
    vector<int> deletedTrackIDs = imftTracks->deletedTrackIDs;
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
    for(int i=0;i<imftTracks->numTracks();i++){
        PCObject::Point centroid = imftTracks->tracks.at(i)->lastFrame().object.getCentroid();
        int id = imftTracks->tracks.at(i)->id;

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
        trackID.pose.position.x = centroid.x;
        trackID.pose.position.y = centroid.y;
        trackID.pose.position.z = centroid.z;
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

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "tracking_gmm1");
//    /*
//    if ( ! ros::master::check() ) {
//        return 0;
//    }
//    ros::start();
    ros::NodeHandle n;

    ros::Subscriber sub_id = n.subscribe("/ckinect/pcloud/segmentedID", 1, cb_id);
    pub_track = n.advertise<sensor_msgs::PointCloud2>("/ckinect/pcloud/trackID", 1000);
    pub_model = n.advertise<sensor_msgs::PointCloud2>("/ckinect/pcloud/model", 1000);
    pub_scene = n.advertise<sensor_msgs::PointCloud2>("/ckinect/pcloud/scene", 1000);
    pub_trackID = n.advertise<visualization_msgs::MarkerArray>("/ckinect/marker/trackID", 1000);

    ros::spin();

    return 0;
}

