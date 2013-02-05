#include "tomobjecttracker.h"
#include <Eigen/Dense>

template<typename Ptr>
TOMObjectTracker<Ptr>::TOMObjectTracker(int nWindow, bool isDebug) : Parent(nWindow, isDebug)
{
}

template<typename Ptr>
double TOMObjectTracker<Ptr>::gain(V pNode, V cNode, double alpha)
{

    double maxX = 0.5;
    double maxY = 0.2;
    double maxZ = 0.2;
    double maxCovDist = 10;
    //    double alpha = 0.3; // weighting for mean

    Ptr statistic1 = pNode.ptr;
    Ptr statistic2 = cNode.ptr;

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

    double weight = alpha * meanDist + (1-alpha) * covDist;

    return weight;
}

template<typename Ptr>
double TOMObjectTracker<Ptr>::gain(Ptr pPtr, Ptr cPtr, double alpha)
{

    double maxX = 0.2;
    double maxY = 0.2;
    double maxZ = 0.2;
    double maxCovDist = 5;
    //    double alpha = 0.3; // weighting for mean

    Ptr statistic1 = pPtr;
    Ptr statistic2 = cPtr;

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

    double weight = alpha * meanDist + (1-alpha) * covDist;

    return weight;
}

template<typename Ptr>
void TOMObjectTracker<Ptr>::eraseLastFrame()
{
    int cnt = Parent::cnt;
    if(Parent::m_isDebug) std::cout<<"erase frame of "<<cnt << std::endl;
    vector<ListGraph::Node> vecDelNode;
    for(ListGraph::NodeIt n(Parent::m_g); n != INVALID; ++n){
        if((*Parent::m_gNodeMap)[n].nFrame == cnt){
            ListGraph::Node node = Parent::m_g.nodeFromId(Parent::m_g.id(n)+1);
            if(!(*Parent::m_gNodeMap)[node].isIn){
                if((*Parent::m_gNodeMap)[n].edgeID >= 0){
                    // make edgeID of connected node -1
                    ListGraph::Edge e = Parent::m_g.edgeFromId((*Parent::m_gNodeMap)[n].edgeID);
                    ListGraph::Node v = Parent::m_g.u(e);
                    (*Parent::m_gNodeMap)[v].edgeID = -1;
                    if(Parent::m_isDebug)   printf("edge initialization (-1) of node # %d\n", Parent::m_g.id(v));
                    // erase edge connected with the node
                    Parent::m_g.erase(e);
                }
                vecDelNode.push_back(node);
                vecDelNode.push_back(Parent::m_g.nodeFromId(Parent::m_g.id(node)-1));
            }
        }
    }
    if(Parent::m_isDebug)   printf("deleting %d of nodes\n", vecDelNode.size());
    for(int i=0;i<vecDelNode.size();i++){
        ListGraph::Node n = vecDelNode.at(i);
        Parent::m_g.erase(n);
        if(Parent::m_isDebug)   printf("node %d is deleted\n", (*Parent::m_gNodeMap)[n].id);
    }
    // delete last object in all extended tracks
    if(Parent::m_tracks.size() != 0){
        for(int i=0;i<Parent::m_tracks.size();i++){

            Parent::m_tracks.at(i).pop_back();

        }
    }
    Parent::cnt = cnt-1;
    if(Parent::m_isDebug) confirmDGraph();

}

template<typename Ptr>
void TOMObjectTracker<Ptr>::twoFrameMatching(VecPtr frame1, VecPtr frame2)
{
    Parent::cnt = 1;
    Parent::addToDGraph(frame1);
    Parent::tracking();
    Parent::cnt = 2;
    Parent::addToDGraph(frame2);
    Parent::tracking();
    Parent::backtracking();
    Parent::makeTracks();
    m_matchedTracks.clear();
    for(int i=0;i<Parent::m_tracks.size();i++){
        if(Parent::m_tracks.at(i).size() > 1)
            m_matchedTracks.push_back(Parent::m_tracks.at(i));
    }
    if(Parent::m_isDebug) confirmDGraph();
}


template<typename Ptr>
void TOMObjectTracker<Ptr>::confirmDGraph()
{
    if(Parent::m_isDebug) printf("-----------------------Graph----------------------\n");
    if(Parent::m_isDebug) printf("# of tracks : %d\n", Parent::m_tracks.size());
    for(ListGraph::NodeIt n(Parent::m_g); n != INVALID; ++n)
        if(Parent::m_isDebug) printf("<Node> id:%d nFrame:%d isIn:%d x:%f y:%f z:%f edge:%d IsTrack:%d, nTrack:%d, ptrID:%d\n", Parent::m_g.id(n), (*Parent::m_gNodeMap)[n].nFrame, (*Parent::m_gNodeMap)[n].isIn, (*Parent::m_gNodeMap)[n].ptr.mean[0], (*Parent::m_gNodeMap)[n].ptr.mean[1], (*Parent::m_gNodeMap)[n].ptr.mean[2], (*Parent::m_gNodeMap)[n].edgeID, (*Parent::m_gNodeMap)[n].isTrack, (*Parent::m_gNodeMap)[n].nTrack, (*Parent::m_gNodeMap)[n].ptr.id);
    for(ListGraph::EdgeIt a(Parent::m_g); a != INVALID; ++a)
        if(Parent::m_isDebug) printf("<Edge> id:%d weight:%f uNode:%d vNode:%d\n", Parent::m_g.id(a), (*Parent::m_gEdgeMap)[a], Parent::m_g.id(Parent::m_g.u(a)), Parent::m_g.id(Parent::m_g.v(a)));
    if(Parent::m_isDebug) printf("--------------------------------------------------\n");
}
