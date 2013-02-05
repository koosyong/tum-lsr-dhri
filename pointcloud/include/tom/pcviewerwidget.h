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
#ifndef PCVIEWERWIDGET_H
#define PCVIEWERWIDGET_H

#include <QTimer>
#include <QGLWidget>
#include "tompcfilter.h"
#include "tom/tomobjecttracker.h"

class PCViewerWidget : public QGLWidget
{
    Q_OBJECT
public:
    PCViewerWidget(QWidget *parent = 0);
    ~PCViewerWidget();

public:
    void updatePointCloudPlane(TOMPCFilter<pcl::PointXYZ>::CloudPtr &cloud_plane);
    void updatePointCloudObjects(TOMPCFilter<pcl::PointXYZ>::VecCloud &clouds_object);
    void updatePointCloudBorders(pcl::PointCloud<pcl::PointWithRange>::Ptr &border, pcl::PointCloud<pcl::PointWithRange>::Ptr &veil, pcl::PointCloud<pcl::PointWithRange>::Ptr &shadow);
    void updateConvexHulls(VecHull &hulls);
    void updateStatistics(std::vector<TOM_OBJECT> clouds_statistic);
    void updateTracksID(std::vector<int> &id);
    void updateTracks(TOMObjectTracker<TOM_OBJECT>::VecTrack tracks);
    void updateTCloud(Cloud &tCloudA, Cloud &tCloudB);
    void updateWorkSpace(TOM_TYPE_WS ws);
    void updatePointCloud(TOMPCFilter<pcl::PointXYZ>::CloudPtr &cloud);
    void updateShowMode(TOM_SHOW showMode);
    void updateShowBGMode(TOM_SHOW showBGMode);
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

private: // OpenGL Setting
    QColor bgColor;
    float thetaLook, angleLook, thetaLookPre, angleLookPre, rLookPre;
    float xLook, yLook, zLook, rLook, zVecLook;
    int xRot, yRot, zRot;
    QPoint lastPos;
    GLfloat whiteLight[4];
    GLfloat sourceLight[4];
    GLfloat lightPos[4];
    GLfloat lightPos1[4];
    GLfloat lightPos2[4];
    GLfloat specular[4];

private: // data
    TOM_TYPE_WS m_ws;
    TOMPCFilter<pcl::PointXYZ>::CloudPtr m_cloud;
    TOMPCFilter<pcl::PointXYZ>::CloudPtr m_cloud_plane;
    TOMPCFilter<pcl::PointXYZ>::VecCloud m_clouds_object;
    pcl::PointCloud<pcl::PointWithRange>::Ptr m_cloud_border;
    pcl::PointCloud<pcl::PointWithRange>::Ptr m_cloud_veil;
    pcl::PointCloud<pcl::PointWithRange>::Ptr m_cloud_shadow;
    std::vector<TOM_OBJECT> m_clouds_statistic;
    VecHull m_hulls;
    std::vector<int> m_id;
    TOMObjectTracker<TOM_OBJECT>::VecTrack m_tracks;
    Cloud m_tCloudA, m_tCloudB;
    float randRGB[20][3];
    bool isStart;
    TOM_SHOW m_showMode, m_showBGMode;


public slots:
    void update();

};

#endif // PCVIEWERWIDGET_H
