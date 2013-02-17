#include "pcfilter.h"
#define pi 3.141592

#include <Eigen/Dense>

template<typename T>
PCFilter<T>::PCFilter()
{
}

template<typename T>
bool PCFilter<T>::downSampling(CloudConstPtr &cloud_in, CloudPtr &cloud_out, double leaf_x, double leaf_y, double leaf_z)
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
bool PCFilter<T>::transform(CloudPtr &cloud_in, CloudPtr &cloud_out, TransformList transformList)
{
    cloud_out->header = cloud_in->header;
    cloud_out->is_dense = cloud_in->is_dense;

    //    std::cout << "Transformation" << std::endl;
    for(int i=0;i<cloud_in->points.size();i++){
        T point = cloud_in->points[i];
        for(int j=0;j<transformList.size();j++){
            T temp_point = point;
            Transform transform = transformList.at(j);
            if(transform.type == PC_TRANS_X){
                temp_point.x = point.x+transform.value;
            }
            else if(transform.type == PC_TRANS_Y){
                temp_point.y = point.y+transform.value;
            }
            else if(transform.type == PC_TRANS_Z){
                temp_point.z = point.z+transform.value;
            }
            else if(transform.type == PC_ROT_X){
                temp_point.y = point.y*cos(transform.value*pi/180.) - point.z*sin(transform.value*pi/180.);
                temp_point.z = point.y*sin(transform.value*pi/180.) + point.z*cos(transform.value*pi/180.);
                temp_point.x = point.x;
            }
            else if(transform.type == PC_ROT_Y){
                temp_point.x = point.x*cos(transform.value*pi/180.) - point.z*sin(transform.value*pi/180.);
                temp_point.z = point.x*sin(transform.value*pi/180.) + point.z*cos(transform.value*pi/180.);
                temp_point.y = point.y;
            }
            else if(transform.type == PC_ROT_Z){
                temp_point.x = point.x*cos(transform.value*pi/180.) - point.y*sin(transform.value*pi/180.);
                temp_point.y = point.x*sin(transform.value*pi/180.) + point.y*cos(transform.value*pi/180.);
                temp_point.z = point.z;
            }
            else if(transform.type == PC_REV_X){
                temp_point.x = -point.x;
            }
            else if(transform.type == PC_REV_Y){
                temp_point.y = -point.y;
            }
            else if(transform.type == PC_REV_Z){
                temp_point.z = -point.z;
            }
            point = temp_point;
        }
        cloud_out->points.push_back(point);
    }
    return 1;
}

template<typename T>
bool PCFilter<T>::cut(CloudPtr &cloud_in, CloudPtr &cloud_out, PC_TYPE_WS ws)
{
    //    std::cout << "Workspace cutting" << std::endl;
    for(int i=0;i<cloud_in->points.size();i++){
        T temp_point = cloud_in->points[i];
        // cut off
        if(temp_point.x<ws.right && temp_point.x>ws.left && temp_point.y<ws.top && temp_point.y>ws.bottom && temp_point.z>ws.zbottom && temp_point.z<ws.ztop)
                cloud_out->points.push_back(temp_point);
    }
    return 1;
}

template<typename T>
bool PCFilter<T>::extractPlane(CloudPtr &cloud_in, CloudPtr &cloud_out, VecCloud &clouds_plane, int nPlane)
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
bool PCFilter<T>::segmentation(CloudPtr &cloud_in, VecCloud &clouds_out, double tolerance, double minSize, double maxSize)
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

// instantiate class for possible types
template class PCFilter<pcl::PointXYZ>;
template class PCFilter<pcl::PointXYZRGB>;
