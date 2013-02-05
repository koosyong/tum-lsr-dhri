#include "tomtracking.h"
#include <math.h>
#include <pcl/features/principal_curvatures.h>

TOMTracking::TOMTracking(TOM_TYPE_WS ws)
{
    m_ws = ws;
    tracker = new Tracker(5,0);
    m_isDebug = 0;
}

TOMTracking::~TOMTracking()
{
    delete tracker;
}

TOMObjectTracker<TOM_OBJECT>::VecTrack& TOMTracking::tracking(std::vector<TOM_OBJECT> &objects, std::vector<CloudPtr> &clouds, std::vector<int>& tracksID, Cloud& tCloudPointA, Cloud &tCloudPointB)
{
    m_objects = objects;
    m_clouds = clouds;

    m_tracks = tracker->getTracks();
    // object merging
    if(m_objects.size() > m_tracks->size() && m_tracks->size() > 0){
        // two frame matching between objects and last point of tracks
        Tracker *objectTracker = new Tracker(2,0);
        std::vector<TOM_OBJECT> frame1, frame2;
        // make frame1 and frame2
        for(int i=0;i<m_tracks->size();i++){
            frame1.push_back(m_tracks->at(i).ptrLast());
        }
        for(int i=0;i<m_objects.size();i++){
            frame2.push_back(m_objects.at(i));
        }
        objectTracker->twoFrameMatching(frame1, frame2);
        // assigne point cloud on new tracks from m_tracks
        TOMObjectTracker<TOM_OBJECT>::VecTrack *objectTracks = objectTracker->getMatchedTracks();
        if(m_isDebug) std::cout << "# of Matched tracks " << objectTracks->size() << std::endl;
        std::vector<Cloud> trackClouds;
        for(int i=0;i<m_tracks->size();i++){
            trackClouds.push_back(m_tracks->at(i).getCloud());
        }
        for(int i=0;i<objectTracks->size();i++){
            for(int j=0;j<m_tracks->size();j++){
                if(objectTracks->at(i).ptrAt(0).id == m_tracks->at(j).ptrLast().id){
                    CloudPtr cloudPtr (new Cloud);
                    *cloudPtr = trackClouds.at(j);
                    objectTracks->at(i).setCloud(cloudPtr);
                    cloudPtr.reset();
                }
            }
        }

        Tracker::VecPtr vecUnmatchedObjects = objectTracker->getUnmatchedPtrs();
        if(m_isDebug) std::cout << "# of Unmatched objects " << vecUnmatchedObjects.size() << std::endl;
        // unmatched object is out of the boundary --> merge objects
        for(int i=0;i<vecUnmatchedObjects.size();i++){
            // merge objects
            TOM_OBJECT object = vecUnmatchedObjects.at(i);
            if(m_isDebug) std::cout << "Object " << object.id << " is new" << std::endl;
            // check whether the pos is inside of the boundary
            if(object.mean[0] > m_ws.left+m_ws.margin && object.mean[0] < m_ws.right-m_ws.margin
                    && object.mean[1] > m_ws.bottom+m_ws.margin && object.mean[1] < m_ws.top-m_ws.margin)
            {
                if(m_isDebug) std::cout << "Object " << object.id << " is deleted" << std::endl;

                CloudPtr pCloudPtrA (new Cloud);
                Cloud &pCloudA = *pCloudPtrA;
                CloudPtr pCloudPtrB (new Cloud);
                Cloud &pCloudB = *pCloudPtrB;
                pCloudPtrB = m_clouds.at(object.id);
                // point matching with all tracks and find the most matched track and merge
                double maxScore = 0;
                int maxTrack = 0;
                for(int j=0;j<objectTracks->size();j++){
                    pCloudA = objectTracks->at(j).getCloud();
                    TOMPointTracker<TrackPoint> *pointTracker = new TOMPointTracker<TrackPoint>(2,0);
                    pointTracker->setMode(MEG);
                    pcl::PointXYZ velA;
                    velA.x = objectTracks->at(j).ptrLast().mean[0] - objectTracks->at(j).ptrAtBack(1).mean[0];
                    velA.y = objectTracks->at(j).ptrLast().mean[1] - objectTracks->at(j).ptrAtBack(1).mean[1];
                    velA.z = objectTracks->at(j).ptrLast().mean[2] - objectTracks->at(j).ptrAtBack(1).mean[2];
                    double score = pointTracker->twoGroupMatchingScore(pCloudPtrA, velA, pCloudPtrB);
                    if(score > maxScore){
                        maxScore = score;
                        maxTrack = j;
                    }
                    delete pointTracker;
                    if(m_isDebug) std::cout << "score of track " << objectTracks->at(j).num() << " is " << score << std::endl;

                }
                pCloudPtrA.reset();
                pCloudPtrB.reset();

                // merge track i into the track maxTrack
                int cloudid_part = object.id;
                int cloudid_body = objectTracks->at(maxTrack).ptrLast().id;

                if(m_isDebug) std::cout << "B : # of body " << m_clouds.at(cloudid_body)->size() << " # of part " << m_clouds.at(cloudid_part)->size() << std::endl;
                *m_clouds.at(cloudid_body) += *m_clouds.at(cloudid_part);
                if(m_isDebug) std::cout << "A : # of body " << m_clouds.at(cloudid_body)->size() << " # of part " << m_clouds.at(cloudid_part)->size() << std::endl;
                std::vector<CloudPtr>::iterator it_clouds = m_clouds.begin();
                it_clouds += cloudid_part;
                m_clouds.erase(it_clouds);

                // make statistics
                TOMPCFilter<pcl::PointXYZ>* filter = new TOMPCFilter<pcl::PointXYZ>();
                m_objects.clear();
                filter->statistic(m_clouds, m_objects);
                delete filter;
                // erase object in the tracker
//                tracker->deleteNodeInLastFrame(object);
                //            tracker->updatePtrsInLastFrame(m_objects);
                // erase track of i
                //                vecDelTracks.push_back(m_tracks->at(i).num());
            }
        }
        delete objectTracker;

    }
    // TOMObjectTracker
    tracker->setFrame(m_objects);
    m_tracks = tracker->getTracks();

    Cloud tempCloudA, tempCloudB;
    bool isModifiable = 0;
    // search tracks not updated
    for(int i=0;i<m_tracks->size();i++){
        // seperate objects
        if(!m_tracks->at(i).isExtended()){
            // check whether the pos is in the margin of boundary
            if(m_tracks->at(i).ptrLast().mean[0] <= m_ws.left+m_ws.margin || m_tracks->at(i).ptrLast().mean[0] >= m_ws.right-m_ws.margin
                    || m_tracks->at(i).ptrLast().mean[1] <= m_ws.bottom+m_ws.margin || m_tracks->at(i).ptrLast().mean[1] >= m_ws.top-m_ws.margin)
            {
                // delete the track in the boundary
                if(m_isDebug) std::cout << "track " << m_tracks->at(i).num() << " is out" << std::endl;
                if(m_isDebug) std::cout << "11" << std::endl;
                tracker->deleteTrack(m_tracks->at(i).num());
                if(m_isDebug) std::cout << "12" << std::endl;
                m_tracks = tracker->getTracks();
                if(m_isDebug) std::cout << "13" << std::endl;
            }
            else{
                if(m_isDebug) std::cout << "track " << m_tracks->at(i).num() << " is not extended" << std::endl;
                CloudPtr pCloudPtrA (new Cloud);
                CloudPtr pCloudPtrB (new Cloud);
                CloudPtr fCloudPtr (new Cloud);
                Cloud &pCloudA = *pCloudPtrA;
                Cloud &pCloudB = *pCloudPtrB;
                Cloud &fCloud = *fCloudPtr;
                pCloudA = m_tracks->at(i).getCloud();

                // find the most probable cloud for matching
                double max = 0.0;
                int maxMatchObject = -1;
                int maxTrack = -1;
                TOM_OBJECT tPtr = m_tracks->at(i).ptrLast();
                for(int j=0;j<m_objects.size();j++){
                    TOM_OBJECT cPtr = m_objects.at(j);
                    double gain = tracker->gain(tPtr, cPtr, 1.0);
                    if(gain > max){
                        max = gain;
//                        maxMatchObject = m_objects.at(j).id;
                        maxMatchObject = j;
                    }
                }
                if(m_isDebug) std::cout << "max gain" << max <<  " maxMatchObject" << maxMatchObject<< " Size" << m_clouds.size()<<std::endl;
                if(maxMatchObject != -1) fCloudPtr = m_clouds.at(maxMatchObject);
                //                else maxMatchObject = -1;                
                for(int j=0;j<m_tracks->size();j++){
                    if(m_tracks->at(j).isExtended()){
                        if(m_tracks->at(j).ptrLast().id == maxMatchObject){
                            maxTrack = j;
                        }
                    }
                }
                if(maxTrack != -1) pCloudB = m_tracks->at(maxTrack).getCloud();
                if(m_isDebug)  std::cout << "maxObject : " << maxMatchObject << " maxTrack : " << maxTrack << std::endl;
                TOMPointTracker<TrackPoint> *pointTracker = new TOMPointTracker<TrackPoint>(2,0);
                CloudPtr tCloudPtrA (new Cloud);
                CloudPtr tCloudPtrB (new Cloud);
                Cloud &tCloudA = *tCloudPtrA;
                Cloud &tCloudB = *tCloudPtrB;
                pointTracker->setMode(SEP);
                pcl::PointXYZ velA, velB;
                velA.x = m_tracks->at(i).ptrLast().mean[0] - m_tracks->at(i).ptrAtBack(1).mean[0];
                velA.y = m_tracks->at(i).ptrLast().mean[1] - m_tracks->at(i).ptrAtBack(1).mean[1];
                velA.z = m_tracks->at(i).ptrLast().mean[2] - m_tracks->at(i).ptrAtBack(1).mean[2];
                velB.x = m_tracks->at(maxTrack).ptrLast().mean[0] - m_tracks->at(maxTrack).ptrAtBack(1).mean[0];
                velB.y = m_tracks->at(maxTrack).ptrLast().mean[1] - m_tracks->at(maxTrack).ptrAtBack(1).mean[1];
                velB.z = m_tracks->at(maxTrack).ptrLast().mean[2] - m_tracks->at(maxTrack).ptrAtBack(1).mean[2];
                if(maxTrack != -1 && maxMatchObject != -1 && pointTracker->twoGroupMatching(pCloudPtrA, velA, pCloudPtrB, velB, fCloudPtr, tCloudA, tCloudB)){
                    //                if(pointTracker->twoGroupMatching(pCloudPtrA, pCloudPtrB, fCloudPtr, tCloudA, tCloudB)){
                    // if tCloudA is enought to be made as an object, modify track and frames.
                    if (tCloudA.size() >= pCloudA.size()*0.5){
                        if(m_isDebug) cout << "make an object" << std::endl;
                        // delete fCloud
                        std::vector<CloudPtr>::iterator it_clouds = m_clouds.begin();
                        //                        std::vector<TOM_OBJECT>::iterator it_objects = m_objects.begin();
                        it_clouds += maxMatchObject;
                        //                        it_objects += maxMatchObject;

                        m_clouds.erase(it_clouds);
                        //                        m_objects.erase(it_objects);

                        // make clouds

                        //                        std::vector<CloudPtr> clouds_hulls;
                        //                        clouds_hulls.push_back(tCloudPtrA);
                        //                        clouds_hulls.push_back(tCloudPtrB);
                        m_clouds.push_back(tCloudPtrA);
                        m_clouds.push_back(tCloudPtrB);

                        // make statistics
                        TOMPCFilter<pcl::PointXYZ>* filter = new TOMPCFilter<pcl::PointXYZ>();
                        //                        std::vector<TOM_OBJECT> clouds_statistic;
                        m_objects.clear();
                        filter->statistic(m_clouds, m_objects);
                        //                        m_objects = clouds_statistic;
                        //                        m_objects.push_back(clouds_statistic.at(0));
                        //                        m_objects.push_back(clouds_statistic.at(1));

                        isModifiable = 1;
                        delete filter;
                        if(m_isDebug) std::cout << "00" << std::endl;
                        tCloudPtrA.reset();
                        tCloudPtrB.reset();

                    }
                    if(m_isDebug) std::cout << "01" << std::endl;
                    delete pointTracker;
                    if(m_isDebug) std::cout << "02" << std::endl;
                    pCloudPtrA.reset();
                    if(m_isDebug) std::cout << "03" << std::endl;
                    pCloudPtrB.reset();
                    if(m_isDebug) std::cout << "04" << std::endl;
                    fCloudPtr.reset();
                    if(m_isDebug) std::cout << "05" << std::endl;
                }
            }
        }
    }
    // modify tracks
    if(isModifiable){
        // erase the last frame and tracks

        tracker->eraseLastFrame();
        // update frame
        tracker->setFrame(m_objects);
        m_tracks = tracker->getTracks();
    }


    for(int i=0;i<m_tracks->size();i++){
        if(m_tracks->at(i).size() <= 2){
            if(m_tracks->at(i).ptrLast().mean[0] > m_ws.left+m_ws.margin && m_tracks->at(i).ptrLast().mean[0] < m_ws.right-m_ws.margin
                    && m_tracks->at(i).ptrLast().mean[1] > m_ws.bottom+m_ws.margin && m_tracks->at(i).ptrLast().mean[1] < m_ws.top-m_ws.margin)
            {
                if(m_isDebug) std::cout << "New Track # " << m_tracks->at(i).num() << std::endl;
            }
        }
    }

    // assign cloud on the updated track
    for(int i=0;i<m_objects.size();i++){
        for(int j=0;j<m_tracks->size();j++){
            if(m_tracks->at(j).isExtended()){
                TOM_OBJECT ptr = m_tracks->at(j).ptrLast();
                if(ptr.id == m_objects.at(i).id){
                    m_tracks->at(j).setCloud(m_clouds.at(i));
//                    m_tracks->at(j).setPtr(m_objects.at(ptr.id));
                }
            }
        }
    }
    //

    return *m_tracks;
}

int TOMTracking::findMaxMatchedCloud(TOM_OBJECT tPtr)
{
    double max = 0.0;
    int maxCloud;
    for(int i=0;i<m_objects.size();i++){
        TOM_OBJECT cPtr = m_objects.at(i);
        double gain = tracker->gain(tPtr, cPtr, 1.0);
        if(gain > max){
            max = gain;
            maxCloud = i;
        }
    }
    return maxCloud;
    //    tNode
}
