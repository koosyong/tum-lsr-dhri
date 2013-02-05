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
#ifndef TOMPCFILTER_H
#define TOMPCFILTER_H

// PCL
#include "tomheader.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// CGAL
#ifndef Q_MOC_RUN   // for preventing error message(parse error at "BOOST_JOIN") during compiling
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#endif
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
typedef std::vector<Polyhedron_3> VecHull;

typedef enum{
    TOM_TRANS_X, TOM_TRANS_Y, TOM_TRANS_Z, TOM_ROT_X, TOM_ROT_Y, TOM_ROT_Z, TOM_REV_X, TOM_REV_Y, TOM_REV_Z
} TOM_TYPE_TRANSFORM;


typedef struct{
    TOM_TYPE_TRANSFORM type;
    double value;
} Transform;

typedef std::vector<Transform> TransformList;

//#define Cloud pcl::PointCloud<PointT>
//#define CloudPtr Cloud::Ptr
//#define VecCloudPtr std::vector<CloudPtr>

template<typename T>
class TOMPCFilter
{

public:
    TOMPCFilter();

    typedef pcl::PointCloud<T> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef std::vector<CloudPtr> VecCloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    EIGEN_ALIGN16 Eigen::Matrix3d a;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    bool downSampling(CloudConstPtr &cloud_in, CloudPtr &cloud_out, double leaf_x, double leaf_y, double leaf_z);
    bool transform(CloudPtr &cloud_in, CloudPtr &cloud_out, TransformList transformList);
    bool cut(CloudPtr &cloud_in, CloudPtr &cloud_out, TOM_TYPE_WS ws);
    bool extractPlane(CloudPtr &cloud_in, CloudPtr &cloud_out, VecCloud &clouds_plane, int nPlane);
    bool segmentation(CloudPtr &cloud_in, VecCloud &clouds_out, double tolerance, double minSize, double maxSize);
    bool convexHull(VecCloud &clouds_in, VecCloud &clouds_out, VecHull &hulls);
    bool statistic(VecCloud &clouds_in, std::vector<TOM_OBJECT> &clouds_statistic);
    void similarity(std::vector<TOM_OBJECT> &clouds_statistic1, std::vector<TOM_OBJECT> &clouds_statistic2);
};

#endif // TOMPCFILTER_H
