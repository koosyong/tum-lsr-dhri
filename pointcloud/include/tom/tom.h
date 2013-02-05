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
#ifndef TOM_H
#define TOM_H

// EIGEN_ALIGN_STATICALLY is the true test whether we want to align arrays on the stack or not. It takes into account both the user choice to explicitly disable
// alignment (EIGEN_DONT_ALIGN_STATICALLY) and the architecture config (EIGEN_ARCH_WANTS_STACK_ALIGNMENT). Henceforth, only EIGEN_ALIGN_STATICALLY should be used.
#if EIGEN_ARCH_WANTS_STACK_ALIGNMENT && !defined(EIGEN_DONT_ALIGN_STATICALLY)
  #define EIGEN_ALIGN_STATICALLY 1
#else
  #define EIGEN_ALIGN_STATICALLY 0
  #ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
    #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
  #endif
#endif

#include <QTimer>
#include "pcviewerwidget.h"

// Boost
//#include <boost/thread/thread.hpp>

// PCL IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include "tomheader.h"
#include "tompcfilter.h"
#include "tomtracking.h"

typedef void(*pfCallBack)(IMFT<TOM_OBJECT>::VecTrack tracks);

class TOM : public QObject
{
    Q_OBJECT

public:
    TOM(PCViewerWidget* viewer_=0);
    ~TOM();
    bool openKinect();
    void closeKinect();
    void savePCD();
    bool loadPCD(std::string filename);
    void testDistance(std::string file1, std::string file2);
    void savingPCDs(bool isOn);
    void setRotX(double rotX_){rotX = rotX_;}
    void setRotZ(double rotZ_){rotZ = rotZ_;}
    void setOffsetZ(double offsetZ_){offsetZ = offsetZ_;}
    void setWS(TOM_TYPE_WS ws_){ ws = ws_;}
    void occlusion(bool isOn);
    double getRotX(){ return rotX;}
    double getRotZ(){ return rotZ;}
    double getOffsetZ(){ return offsetZ;}
    TOM_TYPE_WS getWS(){ return ws;}

    void showRawdata();
    void showSegments();
    void showObjects();
    void showTracks();
    void showTrackClouds();
    void showBackground();
    void registerCallbackStoreData(pfCallBack pfStoreData);
    void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_);

private:

//    void segmentation();
//    void bordering();
    void updateViewer();
    pfCallBack m_pfStoreData;

private:
    PCViewerWidget* viewer;
    pcl::Grabber* grabber;
    TOMTracking* tracker;
    TOMPCFilter<pcl::PointXYZ>* filter;
    TOM_SHOW m_showMode, m_showBGMode;

protected:
    std::string device_id_;    
    CloudPtr cloud_sampled;
    CloudPtr cloud_transformed;
    CloudPtr cloud_cut;
    CloudPtr cloud_outofplane;
    std::vector<CloudPtr> clouds_plane;
    std::vector<CloudPtr> clouds_object;
    std::vector<CloudPtr> clouds_object_sampled;
    std::vector<CloudPtr> clouds_hulls;
    std::vector<TOM_OBJECT> clouds_statistic;
    std::vector<CloudPtr> clouds_object_sampled_cut;
    std::vector<TOM_OBJECT> clouds_statistic_cut;
    std::vector<TOM_OBJECT> objects;
    IMFT<TOM_OBJECT>::VecTrack tracks;
    VecHull hulls;

    Cloud tCloudA, tCloudB;

//    EIGEN_ALIGN16 Eigen::Matrix3f c;

    std::vector<int> tracksID;

    CloudConstPtr cloud;

    int nPlane;
    double rotX, rotZ;
    double offsetZ;
    TOM_TYPE_WS ws;


private:    
    QTimer *timer_cal;
    bool m_isSavingPCDs;
    bool m_isOcclusionOn;
    bool m_isCBRegistered;

private slots:
    void calSlot();

};

#endif // TOM_H
