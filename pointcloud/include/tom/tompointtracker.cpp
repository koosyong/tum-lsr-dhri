#include "tompointtracker.h"

template<typename Ptr>
TOMPointTracker<Ptr>::TOMPointTracker(int nWindow, bool isDebug) : Parent(nWindow, isDebug)
{
    m_isDebug = isDebug;
}


template<typename Ptr>
bool TOMPointTracker<Ptr>::twoGroupMatching(CloudPtr &pGroupA, pcl::PointXYZ velA, CloudPtr &pGroupB, pcl::PointXYZ velB, CloudPtr &fGroup, Cloud &tGroupA, Cloud &tGroupB)
{
    // make frame 1
    VecPtr frame1, frame2;
    for(int i=0;i<pGroupA->size();i++){
        TrackPoint point;
        point.measured = pGroupA->points[i];
        point.prediction.x = pGroupA->points[i].x + 2*velA.x;
        point.prediction.y = pGroupA->points[i].y + 2*velA.y;
        point.prediction.z = pGroupA->points[i].z + 2*velA.z;
        frame1.push_back(point);
    }
    for(int i=0;i<pGroupB->size();i++){
        TrackPoint point;
        point.measured = pGroupB->points[i];
        point.prediction.x = pGroupB->points[i].x + 2*velB.x;
        point.prediction.y = pGroupB->points[i].y + 2*velB.y;
        point.prediction.z = pGroupB->points[i].z + 2*velB.z;
        frame1.push_back(point);
    }
//    std::cout << velA << velB << std::endl;
    for(int i=0;i<fGroup->size();i++){
        TrackPoint point;
        point.measured = fGroup->points[i];
        frame2.push_back(point);
    }
    Parent::cnt = 1;
    Parent::addToDGraph(frame1);
    Parent::tracking();
    Parent::cnt = 2;
    Parent::addToDGraph(frame2);
    Parent::tracking();
    Parent::backtracking();
    Parent::makeTracks();
    if(Parent::m_isDebug) confirmDGraph();

    // get tGroupA
    //    id : 0~frame1.size()*2

    for(int i=0;i<Parent::m_tracks.size();i++){
        if(Parent::m_tracks.at(i).size() == 2){
            int node = Parent::m_tracks.at(i).nodeAt(0);
//            if(Parent::m_isDebug) std::cout << " Node " << node << " pGroupA 1 " << pGroupA->size()*2 << std::endl;
            if(node < pGroupA->size()*2)  // tGroupA
                tGroupA.points.push_back(Parent::m_tracks.at(i).ptrLast().measured);
            else
                tGroupB.points.push_back(Parent::m_tracks.at(i).ptrLast().measured);
        }
    }
    if(Parent::m_isDebug) std::cout << "tGroupA : " << tGroupA.size() << " tGroupB : " << tGroupB.size() << std::endl;
    if(tGroupA.size() == 0 || tGroupB.size() == 0)
        return 0;
    else return 1;

}

template<typename Ptr>
double TOMPointTracker<Ptr>::twoGroupMatchingScore(CloudPtr &pGroupA,  pcl::PointXYZ velA, CloudPtr &pGroupB)
{
    // make frame 1
    VecPtr frame1, frame2;
    for(int i=0;i<pGroupA->size();i++){
        TrackPoint point;
        point.measured = pGroupA->points[i];
        point.prediction.x = pGroupA->points[i].x + 2*velA.x;
        point.prediction.y = pGroupA->points[i].y + 2*velA.y;
        point.prediction.z = pGroupA->points[i].z + 2*velA.z;
        frame1.push_back(point);
    }
    for(int i=0;i<pGroupB->size();i++){
        TrackPoint point;
        point.measured = pGroupB->points[i];
        frame2.push_back(point);
    }

    Parent::cnt = 1;
    Parent::addToDGraph(frame1);
    Parent::tracking();
    Parent::cnt = 2;
    Parent::addToDGraph(frame2);
    Parent::tracking();
    Parent::backtracking();
    Parent::makeTracks();

    return Parent::getWeightSum();
}

template<typename Ptr>
void TOMPointTracker<Ptr>::confirmDGraph()
{
    if(Parent::m_isDebug) printf("-----------------------Graph----------------------\n");
    for(ListGraph::NodeIt n(Parent::m_g); n != INVALID; ++n)
        if(Parent::m_isDebug) printf("<Node> id:%d nFrame:%d isIn:%d x:%f y:%f z:%f edge:%d IsTrack:%d, nTrack:%d\n", Parent::m_g.id(n), (*Parent::m_gNodeMap)[n].nFrame, (*Parent::m_gNodeMap)[n].isIn, (*Parent::m_gNodeMap)[n].ptr.measured.x, (*Parent::m_gNodeMap)[n].ptr.measured.y, (*Parent::m_gNodeMap)[n].ptr.measured.z, (*Parent::m_gNodeMap)[n].edgeID, (*Parent::m_gNodeMap)[n].isTrack, (*Parent::m_gNodeMap)[n].nTrack);
    for(ListGraph::EdgeIt a(Parent::m_g); a != INVALID; ++a)
       if(Parent::m_isDebug) printf("<Edge> id:%d weight:%f uNode:%d vNode:%d\n", Parent::m_g.id(a), (*Parent::m_gEdgeMap)[a], Parent::m_g.id(Parent::m_g.u(a)), Parent::m_g.id(Parent::m_g.v(a)));
    if(Parent::m_isDebug) printf("--------------------------------------------------\n");
}

template<typename Ptr>
double TOMPointTracker<Ptr>::gain(typename Parent::V pNode, typename Parent::V cNode, double alpha)
{

    double maxX = 1;
    double maxY = 1;
    double maxZ = 1;
    //    double alpha = 0.3; // weighting for mean

    Ptr ptr1 = pNode.ptr;
    Ptr ptr2 = cNode.ptr;

    double meanDist = 1 - sqrt(pow(ptr1.measured.x-ptr2.measured.x,2) + pow(ptr1.measured.y-ptr2.measured.y,2) + pow(ptr1.measured.z-ptr2.measured.z,2))
            / sqrt(maxX*maxX + maxY*maxY + maxZ*maxZ);

    double weight = meanDist;

    return weight;
}
