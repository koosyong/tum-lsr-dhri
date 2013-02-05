#include "pcviewerwidget.h"
#include <QtGui>
#include <GL/glu.h>
#include <iostream>

#define pi 3.141592

PCViewerWidget::PCViewerWidget(QWidget *parent) : QGLWidget(parent)
{
    whiteLight[0] = 0.2f;
    whiteLight[1] = 0.2f;
    whiteLight[2] = 0.2f;
    whiteLight[3] = 1.0f;

    sourceLight[0] = 0.8f;
    sourceLight[1] = 0.8f;
    sourceLight[2] = 0.8f;
    sourceLight[3] = 1.0f;

    lightPos[0] = 0.0f;
    lightPos[1] = 0.0f;
    lightPos[2] = -1000.0f;
    lightPos[3] = 0.0f;

    lightPos1[0] = 0.0f;
    lightPos1[1] = 0.0f;
    lightPos1[2] = 20.0f;
    lightPos1[3] = 0.0f;

    lightPos2[0] = 0.0f;
    lightPos2[1] = 0.0f;
    lightPos2[2] = 10.0f;
    lightPos2[3] = 0.0f;

    specular[0] = 0.3f;
    specular[1] = 0.3f;
    specular[2] = 0.3f;
    specular[3] = 0.3f;

    thetaLook = -140;
    angleLook = -90;
    zVecLook = 1;
    angleLookPre = 0;
    thetaLookPre = 0;
    rLook = 0.8;

    xLook = rLook * sin(thetaLook/180*pi) * sin(angleLook/180*pi);
    yLook = -rLook * cos(thetaLook/180*pi);
    zLook = rLook * sin(thetaLook/180*pi) * cos(angleLook/180*pi);

    isStart = 0;

    QTimer* timer = new QTimer (this);
    timer->start (200);
    connect (timer, SIGNAL (timeout ()), this, SLOT (update ()));

    bgColor.setRgb(0,200,200);

    for(int i=0;i<20;i++){
        randRGB[i][0] = (float)rand()/RAND_MAX/2+0.5;
        randRGB[i][1] = (float)rand()/RAND_MAX/2+0.5;
        randRGB[i][2] = (float)rand()/RAND_MAX/2+0.5;
    }

    m_ws.bottom = 0.;
    m_ws.top = 0.;
    m_cloud.reset(new  pcl::PointCloud<pcl::PointXYZ>);
    m_cloud_plane.reset(new  pcl::PointCloud<pcl::PointXYZ>);
    m_cloud_border.reset(new pcl::PointCloud<pcl::PointWithRange>);
    m_cloud_veil.reset(new pcl::PointCloud<pcl::PointWithRange>);
    m_cloud_shadow.reset(new pcl::PointCloud<pcl::PointWithRange>);
    m_showBGMode = SHOW_BACKGROUND_OFF;
    m_showMode = SHOW_TRACKS;
}

PCViewerWidget::~PCViewerWidget()
{

}

void PCViewerWidget::update()
{
    updateGL();
}

void PCViewerWidget::initializeGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);

    // line anti-aliasing
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,  GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);

    // light values and coordinates
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);

    // Enable lighting
    //    glEnable(GL_LIGHTING);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, whiteLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, sourceLight);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glEnable(GL_LIGHT0);

    // Enable color tracking
    glEnable(GL_COLOR_MATERIAL);

    // Set Material properties to follow glColor values
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    // Set Material shine with specular light
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMateriali(GL_FRONT, GL_SHININESS, 1);

    resizeGL(1280, 480);
}

void PCViewerWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(xLook, yLook, zLook, 0,0,0, 0,1,0);


    glRotatef(-90,1,0,0);
    glRotatef(90,0,0,1);
glTranslatef(0.1,0.1,0);

    //    glTranslated(0,0,-1);
    //    glRotatef(-90,0,1,0);
    //    glRotatef(90,1,0,0);


//        // Draw Axis
//        float a = 100;
//        glLineWidth(2);
//        glBegin(GL_LINES);{
//            glColor4f(1,0,0,1);
//            glVertex3f(0,0,0);
//            glVertex3f(.1,0,0);
//            glColor4f(0,1,0,1);
//            glVertex3f(0,0,0);
//            glVertex3f(0,.1,0);
//            glColor4f(0,0,1,1);
//            glVertex3f(0,0,0);
//            glVertex3f(0,0,.1);
//        }
//        glEnd();

    //    // Draw Grid
    //    glLineWidth(1);
    //    glColor4f(1,1,1,0.3);
    //    glBegin(GL_LINES);

    //    for(int i=0;i<20;i++){
    //        glVertex3f(i*.1-1, -1, 0);
    //        glVertex3f(i*.1-1, 1, 0);
    //    }
    //    for(int i=0;i<20;i++){
    //        glVertex3f(-1,i*.1-1,0);
    //        glVertex3f(1,i*.1-1,0);
    //    }
    //    glEnd();


    //    // Draw workspace
    //    if(m_ws.bottom != m_ws.top){
    //        glLineWidth(2);
    //        glColor4f(1,0,0,0.5);
    //        glBegin(GL_LINE_STRIP);{
    //            glVertex3f(m_ws.left, m_ws.top, 0);
    //            glVertex3f(m_ws.right, m_ws.top, 0);
    //            glVertex3f(m_ws.right, m_ws.bottom, 0);
    //            glVertex3f(m_ws.left, m_ws.bottom, 0);
    //            glVertex3f(m_ws.left, m_ws.top, 0);

    //        } glEnd();
    //        glBegin(GL_LINE_STRIP);{
    //            glVertex3f(m_ws.left+m_ws.margin, m_ws.top-m_ws.margin, 0);
    //            glVertex3f(m_ws.right-m_ws.margin, m_ws.top-m_ws.margin, 0);
    //            glVertex3f(m_ws.right-m_ws.margin, m_ws.bottom+m_ws.margin, 0);
    //            glVertex3f(m_ws.left+m_ws.margin, m_ws.bottom+m_ws.margin, 0);
    //            glVertex3f(m_ws.left+m_ws.margin, m_ws.top-m_ws.margin, 0);

    //        } glEnd();
    //    }

    // Draw points
    if(m_showMode == SHOW_RAWDATA){
        if(m_cloud->size() != 0){
            glPointSize(2);
            glColor4f(0.,0.,0.,1.);
            glBegin(GL_POINTS);
            for(int i=0;i<m_cloud->size();i++){
                glVertex3f(m_cloud->points[i].x,m_cloud->points[i].y,m_cloud->points[i].z);
            }
            glEnd();
        }
    }

    // Draw points plane
    if(m_showBGMode == SHOW_BACKGROUND_ON){
        if(m_cloud_plane->size() != 0){
            glPointSize(1.4);
            glColor4f(0.,0.,0.,1.);
            glBegin(GL_POINTS);
            for(int i=0;i<m_cloud_plane->size();i++){
                glVertex3f(m_cloud_plane->points[i].x,m_cloud_plane->points[i].y,m_cloud_plane->points[i].z);
            }
            glEnd();
        }
    }

    //     Draw objects
    if(m_showMode == SHOW_SEGMENTS || m_showMode == SHOW_OBJECTS){
        if(m_clouds_object.size()!=0){
            for(int i=0;i<m_clouds_object.size();i++){
                glPointSize(3.5);
                glColor4f(randRGB[i%20][0],randRGB[i%20][1],randRGB[i%20][2],1.);
                switch(i){
                case 0: glColor4f(1,0,0,1.); break;
                case 1: glColor4f(0,0,1,1.); break;
                case 2: glColor4f(0,1,0,1.); break;
                case 3: glColor4f(0,1,1,1.); break;
                }

                glBegin(GL_POINTS);
                for(int j=0;j<m_clouds_object.at(i)->size();j++){
                    glVertex3f(m_clouds_object.at(i)->points[j].x,m_clouds_object.at(i)->points[j].y,m_clouds_object.at(i)->points[j].z);
                }
                glEnd();
            }
        }
    }

    if(m_showMode == SHOW_OBJECTS){
        // Draw statistic
        if(m_clouds_statistic.size() != 0){
            for(int i=0;i<m_clouds_statistic.size();i++){
                // mean value
                TOM_OBJECT statistic = m_clouds_statistic.at(i);
                int id = m_clouds_statistic.at(i).id;
                glPointSize(7);
                glColor4f(randRGB[i%20][0],randRGB[i%20][1],randRGB[i%20][2],1.);
                switch(i){
                case 0: glColor4f(1,0,0,1.); break;
                case 1: glColor4f(0,0,1,1.); break;
                case 2: glColor4f(0,1,0,1.); break;
                case 3: glColor4f(0,1,1,1.); break;
                }
                glBegin(GL_POINTS);
                glVertex3f(statistic.mean[0],statistic.mean[1],statistic.mean[2]);
                glEnd();

                // covariance -> ellipsoid
                double scale = 1;
                GLfloat         mat[16];
                GLUquadricObj   *obj = gluNewQuadric();
                gluQuadricDrawStyle( obj, GLU_LINE);    // GLU_FILL

                //  A homogeneous transformation matrix, in this order:
                //
                //     0  4  8  12
                //     1  5  9  13
                //     2  6  10 14
                //     3  7  11 15
                //
                mat[3] = mat[7] = mat[11] = 0;
                mat[15] = 1;
                mat[12] = mat[13] = mat[14] = 0;

                mat[0] = statistic.eigenvectors.col(0)[0]; mat[1] = statistic.eigenvectors.col(0)[1]; mat[2] = statistic.eigenvectors.col(0)[2]; // New X-axis
                mat[4] = statistic.eigenvectors.col(1)[0]; mat[5] = statistic.eigenvectors.col(1)[1]; mat[6] = statistic.eigenvectors.col(1)[2]; // New X-axis
                mat[8] = statistic.eigenvectors.col(2)[0]; mat[9] = statistic.eigenvectors.col(2)[1]; mat[10] = statistic.eigenvectors.col(2)[2];        // New X-axis
                glPushMatrix();
                glTranslatef(statistic.mean[0], statistic.mean[1], statistic.mean[2]);

                glMultMatrixf( mat );
                glScalef(sqrt(statistic.eigenvalues[0])*scale, sqrt(statistic.eigenvalues[1])*scale, sqrt(statistic.eigenvalues[2])*scale);

                gluSphere( obj, 1, 10, 10);
                glPopMatrix();

                gluDeleteQuadric(obj);
                glColor4f(1,1,1,1);
                QFont font("Times", 50, QFont::Bold);
                //                renderText(statistic.mean[0], statistic.mean[1], statistic.mean[2], QString::number(i), font);

            }
        }
    }
    // Draw tracks
    if(m_showMode == SHOW_TRACKS || m_showMode == SHOW_TRACKCLOUDS){
        if(m_tracks.size() != 0){
            for(int i=0;i<m_tracks.size();i++){
                TOM_OBJECT object = m_tracks.at(i).ptrLast();
                int id = m_tracks.at(i).num();
                if(m_showMode == SHOW_TRACKS){
                    // mean value
                    glPointSize(7);
                    glColor4f(randRGB[id%20][0],randRGB[id%20][1],randRGB[id%20][2],1.);

                    switch(id){
                    case 1: glColor4f(1,0,0,1.); break;
                    case 2: glColor4f(0,0,1,1.); break;
                    case 3: glColor4f(0,1,0,1.); break;
                    case 4: glColor4f(0,1,1,1.); break;
                    }
                    glBegin(GL_POINTS);
                    glVertex3f(object.mean[0],object.mean[1],object.mean[2]);
                    glEnd();
                    // covariance -> ellipsoid
                    double scale = 1;
                    GLfloat         mat[16];
                    GLUquadricObj   *obj = gluNewQuadric();
                    gluQuadricDrawStyle( obj, GLU_LINE);    // GLU_FILL

                    //  A homogeneous transformation matrix, in this order:
                    //
                    //     0  4  8  12
                    //     1  5  9  13
                    //     2  6  10 14
                    //     3  7  11 15
                    //
                    mat[3] = mat[7] = mat[11] = 0;
                    mat[15] = 1;
                    mat[12] = mat[13] = mat[14] = 0;

                    mat[0] = object.eigenvectors.col(0)[0]; mat[1] = object.eigenvectors.col(0)[1]; mat[2] = object.eigenvectors.col(0)[2]; // New X-axis
                    mat[4] = object.eigenvectors.col(1)[0]; mat[5] = object.eigenvectors.col(1)[1]; mat[6] = object.eigenvectors.col(1)[2]; // New X-axis
                    mat[8] = object.eigenvectors.col(2)[0]; mat[9] = object.eigenvectors.col(2)[1]; mat[10] = object.eigenvectors.col(2)[2];        // New X-axis
                    glPushMatrix();
                    glTranslatef(object.mean[0], object.mean[1], object.mean[2]);

                    glMultMatrixf( mat );
                    glScalef(sqrt(object.eigenvalues[0])*scale, sqrt(object.eigenvalues[1])*scale, sqrt(object.eigenvalues[2])*scale);

                    gluSphere( obj, 1, 10, 10);
                    glPopMatrix();

                    gluDeleteQuadric(obj);
                    glColor4f(0,0,0,1);
                    QFont font("Times", 30, QFont::Bold);
                    renderText(object.mean[0], object.mean[1], object.mean[2], QString::number(id), font);
                }
                // draw point cloud
                glPointSize(3.5);
                glColor4f(randRGB[id%20][0],randRGB[id%20][1],randRGB[id%20][2],1.);
                switch(id){
                case 1: glColor4f(1,0,0,1.); break;
                case 2: glColor4f(0,0,1,1.); break;
                case 3: glColor4f(0,1,0,1.); break;
                case 4: glColor4f(0,1,1,1.); break;
                }
                glBegin(GL_POINTS);

                for(int k=0;k<m_tracks.at(i).getCloud().size();k++){
                    glVertex3f(m_tracks.at(i).getCloud().points[k].x,m_tracks.at(i).getCloud().points[k].y,m_tracks.at(i).getCloud().points[k].z);
                }
                glEnd();

                //             draw points
                //            for(int j=0;j<m_tracks.at(i).size();j++){
                //                glPointSize(2);
                //                glColor4f(randRGB[i%20][0],randRGB[i%20][1],randRGB[i%20][2],1.);
                //                glBegin(GL_POINTS);
                //                for(int k=0;k<m_tracks.at(i).ptrAt(j).cloud.size();k++){
                //                    glVertex3f(m_tracks.at(i).ptrAt(j).cloud.points[k].x,m_tracks.at(i).ptrAt(j).cloud.points[k].y,m_tracks.at(i).ptrAt(j).cloud.points[k].z);
                //                }
                //                glEnd();
                //            }
            }
        }
    }

    // Draw convex hull points
    if(m_hulls.size() != 0){
        for(int i=0;i<m_hulls.size();i++){
            Polyhedron_3 poly = m_hulls.at(i);

            // draw points
            //            Polyhedron_3::Vertex_iterator it;
            //            glPointSize(7);
            //            glColor4f(randRGB[i][0],randRGB[i][0],randRGB[i][0],0.5);
            //            glBegin(GL_POINTS);
            //            for(it = poly.vertices_begin(); it != poly.vertices_end(); ++it){
            //                glVertex3f(it->point()[0],it->point()[1],it->point()[2]);
            //            }
            //            glEnd();

            // draw edges
            glColor4f(1,1,1,0.3);
            glBegin(GL_LINES);
            Polyhedron_3::Halfedge_iterator it_edge;
            for(it_edge = poly.halfedges_begin(); it_edge != poly.halfedges_end(); ++it_edge){
                Polyhedron_3::Halfedge_handle h = it_edge->prev();
                Polyhedron_3::Vertex_handle vs = h->vertex();
                Polyhedron_3::Vertex_handle ve = it_edge->vertex();
                glVertex3f(vs->point()[0],vs->point()[1],vs->point()[2]);
                glVertex3f(ve->point()[0],ve->point()[1],ve->point()[2]);
            }
            glEnd();
        }
    }


    if(m_tCloudA.size() != 0){
        // draw point cloud
        glPointSize(2);
        glColor4f(1,0,0,1.);
        glBegin(GL_POINTS);

        for(int k=0;k<m_tCloudA.size();k++){
            glVertex3f(m_tCloudA.points[k].x,m_tCloudA.points[k].y,m_tCloudA.points[k].z);
        }
        glEnd();
    }

    if(m_tCloudB.size() != 0){
        // draw point cloud
        glPointSize(2);
        glColor4f(0,0,1,1.);
        glBegin(GL_POINTS);

        for(int k=0;k<m_tCloudB.size();k++){
            glVertex3f(m_tCloudB.points[k].x,m_tCloudB.points[k].y,m_tCloudB.points[k].z);
        }
        glEnd();
    }

}

void PCViewerWidget::resizeGL(int w, int h)
{
    int side = qMax(w, h);
    glViewport((w-side)/2,(h-side)/2,side,side);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //    glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);

    gluPerspective(90.f, 1.2f,0.1f,2000.0f);
    glMatrixMode(GL_MODELVIEW);

}

void PCViewerWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
    angleLookPre = angleLook;
    thetaLookPre = thetaLook;
    rLookPre = rLook;
}

void PCViewerWidget::mouseMoveEvent(QMouseEvent *event)
{
    float dx = event->x() - lastPos.x();
    float dy = event->y() - lastPos.y();

    if(event->buttons() & Qt::LeftButton){
        angleLook = angleLookPre - dx;
        thetaLook = thetaLookPre - dy;

        if(thetaLook > 360) thetaLook = thetaLook - 360;
        if(thetaLook < -360) thetaLook = thetaLook + 360;
        if(angleLook > 360) angleLook = angleLook - 360;
        if(angleLook < -360) angleLook = angleLook + 360;

        if(abs(thetaLook) <= 90 || abs(thetaLook) > 270) zVecLook = 1;
        else zVecLook = -1;

        //        std::cout<<"thetaLook " << thetaLook <<" angleLook : "<<angleLook<<std::endl;
    }
    else if(event->buttons() & Qt::RightButton){
        rLook = rLookPre + dy/10;
        //        printf("%d", dy);
        //        std::cout<<"rLook " << rLook <<std::endl;
    }
    xLook = rLook * sin(thetaLook/180*pi) * sin(angleLook/180*pi);
    yLook = -rLook * cos(thetaLook/180*pi);
    zLook = rLook * sin(thetaLook/180*pi) * cos(angleLook/180*pi);

    updateGL();
}


void PCViewerWidget::updatePointCloudPlane(TOMPCFilter<pcl::PointXYZ>::CloudPtr &cloud_plane)
{
    m_cloud_plane = cloud_plane;
}

void PCViewerWidget::updatePointCloudObjects(TOMPCFilter<pcl::PointXYZ>::VecCloud &clouds_object)
{
    m_clouds_object = clouds_object;
}

void PCViewerWidget::updatePointCloudBorders(pcl::PointCloud<pcl::PointWithRange>::Ptr &border, pcl::PointCloud<pcl::PointWithRange>::Ptr &veil, pcl::PointCloud<pcl::PointWithRange>::Ptr &shadow)
{
    m_cloud_border = border;
    m_cloud_veil = veil;
    m_cloud_shadow = shadow;
    std::cout << "border " << m_cloud_border->points.size() << "veil " << m_cloud_veil->points.size() << "shadow " << m_cloud_shadow->points.size() << std::endl;
}

void PCViewerWidget::updateConvexHulls(VecHull &hulls)
{
    m_hulls = hulls;
}

void PCViewerWidget::updateStatistics(std::vector<TOM_OBJECT> clouds_statistic)
{
    m_clouds_statistic = clouds_statistic;

}

void PCViewerWidget::updateTracksID(std::vector<int> &id)
{
    m_id = id;
}

void PCViewerWidget::updateTracks(TOMObjectTracker<TOM_OBJECT>::VecTrack tracks)
{
    m_tracks = tracks;
}

void PCViewerWidget::updateTCloud(Cloud &tCloudA, Cloud &tCloudB)
{
    m_tCloudA = tCloudA;
    m_tCloudB = tCloudB;
}

void PCViewerWidget::updateWorkSpace(TOM_TYPE_WS ws)
{
    m_ws = ws;
}

void PCViewerWidget::updatePointCloud(TOMPCFilter<pcl::PointXYZ>::CloudPtr &cloud)
{
    m_cloud = cloud;
}

void PCViewerWidget::updateShowMode(TOM_SHOW showMode)
{
    m_showMode = showMode;
    update();
}
void PCViewerWidget::updateShowBGMode(TOM_SHOW showBGMode)
{
    m_showBGMode = showBGMode;
    update();
}
