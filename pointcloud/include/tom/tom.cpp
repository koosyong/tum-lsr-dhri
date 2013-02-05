#include "tom.h"
#define pi 3.141592

TOM::TOM(PCViewerWidget* viewer_) : viewer(viewer_)
{
    grabber = 0;
    tracker = 0;
    m_isOcclusionOn = 0;
    filter = 0;
    filter = new TOMPCFilter<pcl::PointXYZ>();

    //    cloud = 0;
    // parameter
    nPlane = 1; // assume there is one plane
    rotX = 36.;
    rotZ = 0.;
    offsetZ = -0.85;  //0.9
    ws.left = -0.4; // x : -0.35
    ws.right = 0.4; // x : 0.2
    ws.top = 0.18;   // y : 0.15
    ws.bottom = -0.2;   // y
    ws.margin = 0.02;
    tracker = new TOMTracking(ws);
    viewer->updateWorkSpace(ws);
    m_isSavingPCDs = 0;
    m_showMode = SHOW_TRACKS;
    m_isCBRegistered = 0;
    m_showBGMode = SHOW_BACKGROUND_OFF;

    timer_cal = new QTimer(this);
    connect(timer_cal, SIGNAL (timeout()), this, SLOT (calSlot()));

}

TOM::~TOM()
{    
    if(grabber) {
        if (grabber->isRunning ())
            grabber->stop ();
        delete grabber;
    }
    if(tracker)
        delete tracker;
    if(filter) delete filter;

}

void TOM::registerCallbackStoreData(pfCallBack pfStoreData)
{
    m_isCBRegistered = 1;
    m_pfStoreData = pfStoreData;
}

bool TOM::openKinect()
{
    if(grabber == 0){
        grabber = new pcl::OpenNIGrabber();

        // Check if an RGB stream is provided
        if (!grabber->providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
        {
            PCL_ERROR ("Device #1 does not provide an RGB stream!\n");
            return (-1);
        }
        // Start the OpenNI data acquision
        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
                boost::bind (&TOM::cloud_cb, this, _1);
        grabber->registerCallback (f);

    }

    grabber->start ();
//    timer_cal->start(20);
    //    timer_vis->start(200);


    return 1;
}

void TOM::closeKinect()
{
    timer_cal->stop();
    if(grabber){
        if(grabber->isRunning())
            grabber->stop();
    }
}

bool TOM::loadPCD(std::string filename)
{
    closeKinect();

    cloud_outofplane.reset (new Cloud);
    clouds_object.clear();
    clouds_hulls.clear();
    clouds_statistic.clear();
    hulls.clear();
    for(int i=0;i<clouds_object_sampled.size();i++)
        clouds_object_sampled.at(i).reset();
    clouds_object_sampled.clear();

    if (pcl::io::loadPCDFile (filename, *cloud_outofplane) == -1)
    {
        //        cerr << "Was not able to open file \""<<filename<<"\".\n";
        return 0;
    }
    std::cout <<"1"<<std::endl;
    // plane extraction
    filter->segmentation(cloud_outofplane, clouds_object, 0.02, 10, 25000);
    std::cout <<"2"<<std::endl;
    filter->convexHull(clouds_object, clouds_hulls, hulls);
std::cout <<"3"<<std::endl;
    // down sampling
    for(int i=0;i<clouds_object.size();i++){
        CloudPtr cloud_obect_sampled;
        cloud_obect_sampled.reset (new Cloud);
        CloudConstPtr cloudConstPtr = (CloudConstPtr)clouds_object.at(i);

        filter->downSampling(cloudConstPtr, cloud_obect_sampled, 0.01, 0.01, 0.01);
        clouds_object_sampled.push_back(cloud_obect_sampled);
    }
std::cout <<"4"<<std::endl;

    filter->statistic(clouds_object_sampled, clouds_statistic);

std::cout <<"5"<<std::endl;

    updateViewer();
    return 1;
}

void TOM::savePCD()
{
    // save data
    pcl::PointCloud<pcl::PointXYZ>& point_cloud = *cloud_outofplane;
    pcl::io::savePCDFileASCII ("tom.pcd", point_cloud);

}

void TOM::testDistance(std::string file1, std::string file2)
{
    std::cout <<"Open file1 : " << file1 << std::endl;
    std::cout <<"Open file2 : " << file2 << std::endl;

    CloudPtr cloud1, cloud2;
    std::vector<CloudPtr> clouds_object1, clouds_object2;
    std::vector<CloudPtr> clouds_hulls1, clouds_hulls2;
    std::vector<TOM_OBJECT> clouds_statistic1, clouds_statistic2;
    VecHull hulls1, hulls2;

    cloud1.reset(new Cloud);
    cloud2.reset(new Cloud);
    clouds_object1.clear();
    clouds_object2.clear();
    clouds_hulls1.clear();
    clouds_hulls2.clear();
    clouds_statistic1.clear();
    clouds_statistic2.clear();
    hulls1.clear();
    hulls2.clear();


    pcl::io::loadPCDFile (file1, *cloud1);
    pcl::io::loadPCDFile (file2, *cloud2);

    filter->segmentation(cloud1, clouds_object1, 0.02, 10, 25000);
    filter->convexHull(clouds_object1, clouds_hulls1, hulls1);
    filter->statistic(clouds_hulls1, clouds_statistic1);

    filter->segmentation(cloud2, clouds_object2, 0.02, 10, 25000);
    filter->convexHull(clouds_object2, clouds_hulls2, hulls2);
    filter->statistic(clouds_hulls2, clouds_statistic2);
    filter->similarity(clouds_statistic1, clouds_statistic2); // for test
}

void TOM::occlusion(bool isOn)
{
    m_isOcclusionOn = isOn;
}

void TOM::cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_)
{
//    cloud = cloud_;
//    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    //    FPS_CALC ("capture");
    static int cnt=0;
    cnt ++;
    //    if(cnt > 100)
//            calSlot();

}

void TOM::calSlot()
{
    if(!grabber->isRunning ()) return;
    if(cloud == 0) return;

    static int cnt = 0;
    cnt ++;
    if(cnt < 10) return;


    FPS_CALC ("computation");

    cloud_sampled.reset (new Cloud);    // output
    cloud_transformed.reset (new Cloud);
    cloud_cut.reset (new Cloud);
    cloud_outofplane.reset (new Cloud);

    for(int i=0;i<clouds_plane.size();i++)
        clouds_plane.at(i).reset();
    clouds_plane.clear();

    for(int i=0;i<clouds_object.size();i++)
        clouds_object.at(i).reset();
    clouds_object.clear();

    for(int i=0;i<clouds_object_sampled.size();i++)
        clouds_object_sampled.at(i).reset();
    clouds_object_sampled.clear();

    for(int i=0;i<clouds_object_sampled_cut.size();i++)
        clouds_object_sampled_cut.at(i).reset();
    clouds_object_sampled_cut.clear();

    for(int i=0;i<clouds_hulls.size();i++)
        clouds_hulls.at(i).reset();
    clouds_hulls.clear();

    clouds_statistic.clear();
    clouds_statistic_cut.clear();
    hulls.clear();
    objects.clear();

    // down sampling
    filter->downSampling(cloud, cloud_sampled, 0.01, 0.01, 0.01);

    // transformation
    TransformList transformList;
    Transform transform;
    transform.type = TOM_TRANS_Z;
    transform.value = offsetZ;
    transformList.push_back(transform);
    transform.type = TOM_ROT_X;
    transform.value = rotX;
    transformList.push_back(transform);
    transform.type = TOM_REV_Z;
    transform.value = 0;    // any value
    transformList.push_back(transform);
    filter->transform(cloud_sampled, cloud_transformed, transformList);

    // cutting points out of workspace
    if(m_isOcclusionOn){
        ws.left = 0;
    }
    else ws.left = -0.4;
    filter->cut(cloud_transformed, cloud_cut, ws);

    // plane extraction
    filter->extractPlane(cloud_cut, cloud_outofplane, clouds_plane, 1);
    filter->segmentation(cloud_outofplane, clouds_object, 0.02, 20, 25000);
    filter->convexHull(clouds_object, clouds_hulls, hulls);

    // down sampling
    for(int i=0;i<clouds_object.size();i++){
        CloudPtr cloud_obect_sampled;
        cloud_obect_sampled.reset (new Cloud);
        CloudConstPtr cloudConstPtr = (CloudConstPtr)clouds_object.at(i);

        filter->downSampling(cloudConstPtr, cloud_obect_sampled, 0.01, 0.01, 0.01);
        clouds_object_sampled.push_back(cloud_obect_sampled);
    }


    filter->statistic(clouds_object_sampled, clouds_statistic);

//    if(m_isOcclusionOn){
//        ws.left = 0;
//    }
//    else  ws.left = -0.4;

    for (int i=0;i<clouds_statistic.size();i++){
        if(clouds_statistic.at(i).mean[0]<ws.right && clouds_statistic.at(i).mean[0]>ws.left && clouds_statistic.at(i).mean[1]<ws.top && clouds_statistic.at(i).mean[1]>ws.bottom){

            clouds_object_sampled_cut.push_back(clouds_object_sampled.at(i));
            clouds_statistic_cut.push_back(clouds_statistic.at(i));
        }
    }

    // tracking
    // making objects


    tracks = tracker->tracking(clouds_statistic_cut, clouds_object_sampled_cut, tracksID, tCloudA, tCloudB);
    if(m_isCBRegistered) m_pfStoreData(tracks);

    updateViewer();

//    for(int i=0;i<tracks.size();i++)
//        std::cout << tracks.at(i) << std::endl;


}

void TOM::showRawdata()
{
    m_showMode = SHOW_RAWDATA;
    updateViewer();
    viewer->updateShowMode(SHOW_RAWDATA);
}

void TOM::showSegments()
{
    m_showMode = SHOW_SEGMENTS;
    updateViewer();
    viewer->updateShowMode(SHOW_SEGMENTS);
}

void TOM::showObjects()
{
    m_showMode = SHOW_OBJECTS;
    updateViewer();
    viewer->updateShowMode(SHOW_OBJECTS);
}

void TOM::showTracks()
{
    m_showMode = SHOW_TRACKS;
    updateViewer();
    viewer->updateShowMode(SHOW_TRACKS);
}

void TOM::showTrackClouds()
{
    m_showMode = SHOW_TRACKCLOUDS;
    updateViewer();
    viewer->updateShowMode(SHOW_TRACKCLOUDS);
}

void TOM::showBackground()
{
    if(m_showBGMode == SHOW_BACKGROUND_ON)
        m_showBGMode = SHOW_BACKGROUND_OFF;
    else m_showBGMode = SHOW_BACKGROUND_ON;
    updateViewer();
    viewer->updateShowBGMode(m_showBGMode);
}

void TOM::updateViewer()
{
    switch(m_showMode){
    case SHOW_RAWDATA:
        viewer->updatePointCloud(cloud_cut);
        break;
    case SHOW_SEGMENTS :
        viewer->updatePointCloudObjects (clouds_object);
        break;
    case SHOW_OBJECTS :
        viewer->updatePointCloudObjects (clouds_object);
        viewer->updateStatistics(clouds_statistic_cut);
        break;
    case SHOW_TRACKS :
        viewer->updateTracks(tracks);
        break;
    case SHOW_TRACKCLOUDS :
        viewer->updateTracks(tracks);
        break;
    }
    switch(m_showBGMode){
    case SHOW_BACKGROUND_ON:
        viewer->updatePointCloudPlane (clouds_plane.at(0));
        break;
    }

//    viewer->updatePointCloudObjects (clouds_object);
//    viewer->updateTracks(tracks);

    //            viewer->updateConvexHulls(hulls);
    //    viewer->updateStatistics(clouds_statistic);
    //    viewer->updateTCloud(tCloudA, tCloudB);
}

void TOM::savingPCDs(bool isOn)
{
    m_isSavingPCDs = isOn;
}
