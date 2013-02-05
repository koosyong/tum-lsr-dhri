/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Telerobotics and Control Laboratory, KAIST (http://robot.kaist.ac.kr)
 *  Author : Koosy (http://koosy.blogspot.com, koosyong@gmail.com)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef PCFILTER_H
#define PCFILTER_H

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/passthrough.h>

#include <vector>

typedef struct {
    double left, right, top, bottom, ztop, zbottom;
    double margin;
} PC_TYPE_WS;

typedef enum{
    PC_TRANS_X, PC_TRANS_Y, PC_TRANS_Z, PC_ROT_X, PC_ROT_Y, PC_ROT_Z, PC_REV_X, PC_REV_Y, PC_REV_Z
} PC_TYPE_TRANSFORM;


typedef struct{
    PC_TYPE_TRANSFORM type;
    double value;
} Transform;

typedef std::vector<Transform> TransformList;

//#define Cloud pcl::PointCloud<PointT>
//#define CloudPtr Cloud::Ptr
//#define VecCloudPtr std::vector<CloudPtr>

template<typename T>
class PCFilter
{

public:
    PCFilter();

    typedef pcl::PointCloud<T> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef std::vector<CloudPtr> VecCloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    EIGEN_ALIGN16 Eigen::Matrix3d a;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    bool downSampling(CloudConstPtr &cloud_in, CloudPtr &cloud_out, double leaf_x, double leaf_y, double leaf_z);
    bool transform(CloudPtr &cloud_in, CloudPtr &cloud_out, TransformList transformList);
    bool cut(CloudPtr &cloud_in, CloudPtr &cloud_out, PC_TYPE_WS ws);
    bool extractPlane(CloudPtr &cloud_in, CloudPtr &cloud_out, VecCloud &clouds_plane, int nPlane);
    bool segmentation(CloudPtr &cloud_in, VecCloud &clouds_out, double tolerance, double minSize, double maxSize);    
};

#include "pcfilter.hpp"
#endif // PCFILTER_H
