#include "tompcfilter.h"
#define pi 3.141592

#include <Eigen/Dense>

template<typename T>
TOMPCFilter<T>::TOMPCFilter()
{
}

template<typename T>
bool TOMPCFilter<T>::downSampling(CloudConstPtr &cloud_in, CloudPtr &cloud_out, double leaf_x, double leaf_y, double leaf_z)
{
    //    std::cout << "Down sampling" << std::endl;
    // down sampling
    pcl::VoxelGrid<T> vg;
    //    vg.setInputCloud(cloud_preprocessed);
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(leaf_x, leaf_y, leaf_z); // down sampling using a leaf size of 1cm
    vg.filter(*cloud_out);
    return 1;
}

template<typename T>
bool TOMPCFilter<T>::transform(CloudPtr &cloud_in, CloudPtr &cloud_out, TransformList transformList)
{
    //    std::cout << "Transformation" << std::endl;
    for(int i=0;i<cloud_in->points.size();i++){
        T point = cloud_in->points[i];
        for(int j=0;j<transformList.size();j++){
            T temp_point = point;
            Transform transform = transformList.at(j);
            if(transform.type == TOM_TRANS_X){
                temp_point.x = point.x+transform.value;
            }
            else if(transform.type == TOM_TRANS_Y){
                temp_point.y = point.y+transform.value;
            }
            else if(transform.type == TOM_TRANS_Z){
                temp_point.z = point.z+transform.value;
            }
            else if(transform.type == TOM_ROT_X){
                temp_point.y = point.y*cos(transform.value*pi/180.) - point.z*sin(transform.value*pi/180.);
                temp_point.z = point.y*sin(transform.value*pi/180.) + point.z*cos(transform.value*pi/180.);
                temp_point.x = point.x;
            }
            else if(transform.type == TOM_ROT_Y){
                temp_point.y = point.y*cos(transform.value*pi/180.) - point.z*sin(transform.value*pi/180.);
                temp_point.z = point.y*sin(transform.value*pi/180.) + point.z*cos(transform.value*pi/180.);
                temp_point.x = point.x;
            }
            else if(transform.type == TOM_ROT_Z){
                temp_point.y = point.y*cos(transform.value*pi/180.) - point.z*sin(transform.value*pi/180.);
                temp_point.z = point.y*sin(transform.value*pi/180.) + point.z*cos(transform.value*pi/180.);
                temp_point.x = point.x;
            }
            else if(transform.type == TOM_REV_X){
                temp_point.x = -point.x;
            }
            else if(transform.type == TOM_REV_Y){
                temp_point.y = -point.y;
            }
            else if(transform.type == TOM_REV_Z){
                temp_point.z = -point.z;
            }
            point = temp_point;
        }
        cloud_out->points.push_back(point);
    }
    return 1;
}

template<typename T>
bool TOMPCFilter<T>::cut(CloudPtr &cloud_in, CloudPtr &cloud_out, TOM_TYPE_WS ws)
{
    //    std::cout << "Workspace cutting" << std::endl;
    for(int i=0;i<cloud_in->points.size();i++){
        T temp_point = cloud_in->points[i];
        // cut off
        if(temp_point.x<ws.right && temp_point.x>ws.left && temp_point.y<ws.top && temp_point.y>ws.bottom)
//            if(temp_point.x<-0.01 || temp_point.x>0.01)
                cloud_out->points.push_back(temp_point);
    }
    return 1;
}

template<typename T>
bool TOMPCFilter<T>::extractPlane(CloudPtr &cloud_in, CloudPtr &cloud_out, VecCloud &clouds_plane, int nPlane)
{
    cloud_out.reset (new Cloud);
    *cloud_out = *cloud_in;
    //    std::cout << "Plane Extraction" << std::endl;
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<T> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    // plane segmentation
    for(int i=0;i<nPlane;i++)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_out);
        seg.segment (*inliers, *coefficients); //*
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<T> extract;
        extract.setInputCloud (cloud_out);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        CloudPtr cloud_plane;
        cloud_plane.reset(new Cloud);
        extract.filter (*cloud_plane); //*
        clouds_plane.push_back(cloud_plane);
        //      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_out); //*
    }
//    cloud_out = cloud_in;
    return 1;
}

template<typename T>
bool TOMPCFilter<T>::segmentation(CloudPtr &cloud_in, VecCloud &clouds_out, double tolerance, double minSize, double maxSize)
{
    //    std::cout << "Object segmentation" << std::endl;
    // object clustering
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    tree->setInputCloud (cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T> ec;
    ec.setClusterTolerance (tolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud( cloud_in);
    ec.extract (cluster_indices);

    int j = 0;
    clouds_out.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new pcl::PointCloud<T>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
        clouds_out.push_back(cloud_cluster);
    }
    return 1;
}

template<typename T>
bool TOMPCFilter<T>::convexHull(VecCloud &clouds_in, VecCloud &clouds_out, VecHull &hulls)
{
    //    std::cout << "Convex Hull" << std::endl;
    for(int i=0 ; i<clouds_in.size(); i++){
        pcl::PointCloud<T>& point_cloud = *clouds_in.at(i);

        // assign points
        std::vector<Point_3> points;
        for(int j=0;j<point_cloud.size();j++){
            Point_3 point(point_cloud.points[j].x, point_cloud.points[j].y, point_cloud.points[j].z);
            points.push_back(point);
        }

        // define polyhedron to hold convex hull
        Polyhedron_3 poly;

        // compute convex hull of non-collinear points
        CGAL::convex_hull_3(points.begin(), points.end(), poly);
        hulls.push_back(poly);
        CloudPtr cloud_convex (new pcl::PointCloud<T>);

        Polyhedron_3::Vertex_iterator it;
        for(it = poly.vertices_begin(); it != poly.vertices_end(); ++it){
            T point;
            point.x = it->point()[0];
            point.y = it->point()[1];
            point.z = it->point()[2];
            cloud_convex->points.push_back(point);
        }
        clouds_out.push_back(cloud_convex);
    }
    return 1;
}


template<typename T>
bool TOMPCFilter<T>::statistic(VecCloud &clouds_in, std::vector<TOM_OBJECT> &clouds_statistic)
{
    for(int i=0;i<clouds_in.size();i++){
        int size = clouds_in.at(i)->size();
        TOM_OBJECT statistic;

        // mean
        double x = 0;
        double y = 0;
        double z = 0;
        for(int j=0;j<size;j++){
            x += clouds_in.at(i)->points[j].x;
            y += clouds_in.at(i)->points[j].y;
            z += clouds_in.at(i)->points[j].z;
        }
        statistic.mean[0] = x/(double)size;
        statistic.mean[1] = y/(double)size;
        statistic.mean[2] = z/(double)size;

        // covariance
        Eigen::Vector3d demean;
        Eigen::Vector3d xyz_centroid = statistic.mean;
        Eigen::Vector3d point;
        statistic.covariance.setZero();

        double demean_xy, demean_xz, demean_yz;
        // For each point in the cloud
        for (int idx = 0; idx < size; ++idx)
        {
            point[0] = clouds_in.at(i)->points[idx].x;
            point[1] = clouds_in.at(i)->points[idx].y;
            point[2] = clouds_in.at(i)->points[idx].z;
            demean = point - xyz_centroid;

            demean_xy = demean[0] * demean[1];
            demean_xz = demean[0] * demean[2];
            demean_yz = demean[1] * demean[2];

            statistic.covariance(0, 0) += demean[0] * demean[0];
            statistic.covariance(0, 1) += demean_xy;
            statistic.covariance(0, 2) += demean_xz;

            statistic.covariance(1, 0) += demean_xy;
            statistic.covariance(1, 1) += demean[1] * demean[1];
            statistic.covariance(1, 2) += demean_yz;

            statistic.covariance(2, 0) += demean_xz;
            statistic.covariance(2, 1) += demean_yz;
            statistic.covariance(2, 2) += demean[2] * demean[2];
        }
        statistic.covariance = statistic.covariance / (double)size;

        //        std::cout << statistic.covariance << std::endl;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(statistic.covariance);
        statistic.eigenvalues = eigensolver.eigenvalues();
        statistic.eigenvectors = eigensolver.eigenvectors();    // find an eigenvactor by calling .col(i)
        statistic.id = i;
        clouds_statistic.push_back(statistic);
    }
}

template<typename T>
void TOMPCFilter<T>::similarity(std::vector<TOM_OBJECT> &clouds_statistic1, std::vector<TOM_OBJECT> &clouds_statistic2)
{
    double maxX = 0.7;
    double maxY = 0.3;
    double maxZ = 0.3;
    double maxCovDist = 10;
    double alpha = 0.5; // weighting for mean
    // for object 1
    Eigen::MatrixXd sim(clouds_statistic1.size(), clouds_statistic2.size());
    Eigen::MatrixXd dist(clouds_statistic1.size(), clouds_statistic2.size());
    Eigen::MatrixXd weight(clouds_statistic1.size(), clouds_statistic2.size());

    for(int k=0;k<clouds_statistic1.size();k++){
        TOM_OBJECT statistic1 = clouds_statistic1.at(k);
        for(int i=0;i<clouds_statistic2.size();i++){
            TOM_OBJECT statistic2 = clouds_statistic2.at(i);
            Eigen::Matrix3d cov1 = statistic1.covariance;
            Eigen::Matrix3d cov2 = statistic2.covariance;

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
            double meanDist = 1 - sqrt(pow(statistic1.mean[0]-statistic2.mean[0],2) + pow(statistic1.mean[1]-statistic2.mean[1],2) + pow(statistic1.mean[2]-statistic2.mean[2],2))
                                    / sqrt(maxX*maxX + maxY*maxY + maxZ*maxZ);

//            std::cout <<k <<" "<< i <<" : " << dist << std::endl;
            sim(k,i) = covDist;
            dist(k,i) = meanDist;
            weight(k,i) = alpha * meanDist + (1-alpha) * covDist;
            //        A = sqrt(cov1.inverse())*cov2*sqrt(cov1.inverse());
            //        std::cout << A << std::endl;
            //        std::cout << sqrt(A) << std::endl;


        }
    }
    std::cout << sim << std::endl << std::endl;
    std::cout << dist << std::endl << std::endl;
    std::cout << weight << std::endl << std::endl;


    //    std::cout << statistic1.covariance << std::endl;
    //    std::cout << statistic2.covariance << std::endl;

}

// instantiate class for possible types
template class TOMPCFilter<pcl::PointXYZ>;
template class TOMPCFilter<pcl::PointXYZRGB>;
